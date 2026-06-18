//! GLB (binary glTF 2.0) serialiser for the three output modes: [`GlbWriter::build`]
//! (merged — one mesh per colour + web3d `extras`), [`GlbWriter::build_instanced`]
//! (shared mesh per shape-key + native node tree), and [`GlbWriter::build_standard`]
//! (one mesh per component + native node tree). Shared helpers do the precision weld,
//! optional meshopt simplify + vertex-cache pass (feature `optimize`), degenerate-triangle
//! cull, vertex compaction, Z-up→Y-up rotation, and chunk framing.

use crate::instancing::tessellate_local;
use crate::linalg::Mat3x4f;
use crate::rvm_parser::{BBox3, MetaNode};
use serde_json::{Value, json};
use std::collections::{HashMap, HashSet};

// ── meshopt wrappers, gated behind the `optimize` feature ────────────────────
// Native (cli/capi) build with `optimize` on (default); the wasm shell builds
// `--no-default-features`, dropping the C++ meshoptimizer.
// Without it, weld + degenerate cull + compaction still run (pure Rust); only the
// simplify and vertex-cache-reorder passes are skipped — output stays geometrically
// identical at the default `--meshopt-target-error 0` (just a different vertex order).

/// Simplify `indices` toward `len*threshold` (exact target + LockBorder, matching C++).
#[cfg(feature = "optimize")]
fn meshopt_simplify(
    indices: &[u32],
    positions: &[f32],
    threshold: f32,
    target_error: f32,
) -> Vec<u32> {
    use meshopt::{SimplifyOptions, VertexDataAdapter};
    let target = (indices.len() as f32 * threshold) as usize;
    match VertexDataAdapter::new(
        bytemuck::cast_slice(positions),
        3 * std::mem::size_of::<f32>(),
        0,
    ) {
        Ok(adapter) => meshopt::simplify(
            indices,
            &adapter,
            target,
            target_error,
            SimplifyOptions::LockBorder,
            None,
        ),
        Err(_) => indices.to_vec(),
    }
}

#[cfg(not(feature = "optimize"))]
fn meshopt_simplify(
    indices: &[u32],
    _positions: &[f32],
    _threshold: f32,
    _target_error: f32,
) -> Vec<u32> {
    indices.to_vec()
}

/// Reorder `indices` for GPU vertex-cache locality (no geometry/count change).
#[cfg(feature = "optimize")]
fn meshopt_cache(indices: &[u32], vertex_count: usize) -> Vec<u32> {
    meshopt::optimize_vertex_cache(indices, vertex_count)
}

#[cfg(not(feature = "optimize"))]
fn meshopt_cache(indices: &[u32], _vertex_count: usize) -> Vec<u32> {
    indices.to_vec()
}

/// Ids whose subtree contains geometry — every node with primitives plus all its
/// ancestors. Drops empty leaves and wholly-empty branches: the tree-mode equivalent
/// of the merged path's `remove_empty`. Preserves the order of `sorted`.
fn nodes_with_geometry(nodes: &HashMap<u32, MetaNode>, sorted: &[u32]) -> Vec<u32> {
    let has_geo = |n: &MetaNode| {
        n.primitives
            .iter()
            .any(|p| p.triangles_n > 0 || p.vertices_n > 0)
    };
    let mut keep: HashSet<u32> = HashSet::new();
    for id in sorted {
        if has_geo(&nodes[id]) {
            // Mark this node, then walk up marking ancestors until one is already kept.
            let mut cur = *id;
            while keep.insert(cur) {
                let pid = nodes[&cur].parent_id;
                if nodes.contains_key(&pid) {
                    cur = pid;
                } else {
                    break;
                }
            }
        }
    }
    sorted
        .iter()
        .copied()
        .filter(|id| keep.contains(id))
        .collect()
}

fn rotate_z_up_to_y_up(x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    (x, z, -y)
}

// ─── GLB binary serialiser ────────────────────────────────────────────────

pub struct GlbWriter;

impl GlbWriter {
    /// Build GLB bytes for one "root" export.
    /// `nodes`: all MetaNodes in this root
    /// `colors`: ordered list of unique colors
    /// Returns (glb_bytes, filename_base, bbox)
    pub fn build(
        nodes: &mut HashMap<u32, MetaNode>,
        colors: &[u32],
        remove_empty: bool,
        cleanup_positions: bool,
        cleanup_precision: u8,
        meshopt_threshold: f32,
        meshopt_target_error: f32,
    ) -> (Vec<u8>, BBox3) {
        // ── Optionally prune leafless nodes ──────────────────────────────
        if remove_empty {
            loop {
                let parents: std::collections::HashSet<u32> =
                    nodes.values().map(|n| n.parent_id).collect();
                let to_delete: Vec<u32> = nodes
                    .iter()
                    .filter(|(id, n)| {
                        let has_prims = n
                            .primitives
                            .iter()
                            .any(|p| p.triangles_n > 0 || p.vertices_n > 0);
                        !has_prims && !parents.contains(*id)
                    })
                    .map(|(id, _)| *id)
                    .collect();
                if to_delete.is_empty() {
                    break;
                }
                let removed = to_delete.len();
                for id in to_delete {
                    nodes.remove(&id);
                }
                eprintln!("Removed {} empty elements", removed);
            }
        }

        // ── Build glTF JSON + binary buffer ──────────────────────────────
        let mut bin: Vec<u8> = Vec::new();
        let mut buffer_views: Vec<Value> = Vec::new();
        let mut accessors: Vec<Value> = Vec::new();
        let mut materials: Vec<Value> = Vec::new();
        let mut meshes: Vec<Value> = Vec::new();
        let mut gltf_nodes: Vec<Value> = Vec::new();
        let mut scene_nodes: Vec<u32> = Vec::new();
        let mut scene_extras: serde_json::Map<String, Value> = serde_json::Map::new();
        let mut global_bbox = BBox3::default();

        let mut accessor_count = 0usize;
        let mut node_count = 0u32;

        // Single deterministic node ordering shared by every pass below.
        // The draw-range/start-cursor pass and the buffer-fill pass MUST
        // walk nodes in the same order. If they disagree, `node.start` indexes the
        // wrong slice of the merged index buffer, scrambling per-component
        // `draw_ranges` (selection/highlight) and the per-node dedup window.
        let mut sorted_ids: Vec<u32> = nodes.keys().copied().collect();
        sorted_ids.sort();

        for &color in colors {
            // ── compute draw ranges & collect sizes per color ─────────────
            let mut start_cursor = 0u32;
            let mut tri_size = 0usize;
            let mut vert_size = 0usize;

            for id in &sorted_ids {
                let node = nodes.get_mut(id).unwrap();
                if node.color_with_alpha != color || node.primitives.is_empty() {
                    continue;
                }
                node.start = start_cursor;
                node.count = 0;
                for p in &node.primitives {
                    let cnt = p.triangles_n * 3;
                    node.count += cnt;
                    tri_size += cnt as usize;
                    vert_size += p.vertices_n as usize * 3;
                    start_cursor += cnt;
                }
            }

            if tri_size == 0 && vert_size == 0 {
                continue;
            }
            println!("Adding mesh with color id: {}", color);

            // ── flatten all triangulation data ────────────────────────────
            let mut raw_indices: Vec<u32> = Vec::with_capacity(tri_size);
            let mut raw_positions: Vec<f32> = Vec::with_capacity(vert_size);
            let mut offset = 0u32;
            let mut max_index = 0u32;

            let mut min = [f32::MAX; 3];
            let mut max = [f32::MIN; 3];
            let mut first = true;

            for id in &sorted_ids {
                let node = nodes.get(id).unwrap();
                if node.color_with_alpha != color || node.primitives.is_empty() {
                    continue;
                }

                for prim in &node.primitives {
                    for &idx in &prim.indices {
                        let shifted = idx + offset;
                        if shifted > max_index {
                            max_index = shifted;
                        }
                        raw_indices.push(shifted);
                    }
                    offset = max_index + 1;

                    let vn = prim.vertices_n as usize;
                    for i in 0..vn {
                        let u = i * 3;
                        let (rx, ry, rz) = rotate_z_up_to_y_up(
                            prim.vertices[u],
                            prim.vertices[u + 1],
                            prim.vertices[u + 2],
                        );
                        if first {
                            min = [rx, ry, rz];
                            max = [rx, ry, rz];
                            first = false;
                        }
                        for k in 0..3 {
                            let v = [rx, ry, rz][k];
                            if v < min[k] {
                                min[k] = v;
                            }
                            if v > max[k] {
                                max[k] = v;
                            }
                        }
                        raw_positions.push(rx);
                        raw_positions.push(ry);
                        raw_positions.push(rz);
                    }
                }
            }

            // ── dedup (precision-keyed) + optional meshopt simplify + cache optimise ──
            let (final_indices, final_positions, vert_count) = if cleanup_positions {
                let mut new_positions: Vec<f32> = Vec::new();
                let mut new_indices: Vec<u32> = Vec::new();
                let mut new_starts: HashMap<u32, u32> = HashMap::new();
                let mut new_counts: HashMap<u32, u32> = HashMap::new();

                // Scale factor for precision-keyed dedup — same as C++ generate_position_id()
                let scale = 10i64.pow(cleanup_precision as u32);

                for id in &sorted_ids {
                    let node = nodes.get(id).unwrap();
                    if node.color_with_alpha != color || node.primitives.is_empty() {
                        continue;
                    }

                    let local_start = node.start as usize;
                    let local_end = (node.start + node.count) as usize;
                    if local_start >= local_end {
                        continue;
                    }

                    // ── Step 1: precision-keyed dedup (C++ original approach) ──────────
                    // Rounds each coordinate to `cleanup_precision` decimal places and
                    // uses a (i64,i64,i64) key to merge near-coincident vertices.
                    // This is what closes seams between adjacent tessellated primitives.
                    let mut key_map: HashMap<(i64, i64, i64), u32> = HashMap::new();
                    let mut dedup_positions: Vec<f32> = Vec::new();
                    let mut dedup_indices: Vec<u32> = Vec::new();

                    for i in local_start..local_end {
                        let vi = raw_indices[i] as usize * 3;
                        let (x, y, z) = (
                            raw_positions[vi],
                            raw_positions[vi + 1],
                            raw_positions[vi + 2],
                        );
                        let key = (
                            (x * scale as f32).round() as i64,
                            (y * scale as f32).round() as i64,
                            (z * scale as f32).round() as i64,
                        );
                        if let Some(&existing) = key_map.get(&key) {
                            dedup_indices.push(existing);
                        } else {
                            let new_idx = (dedup_positions.len() / 3) as u32;
                            key_map.insert(key, new_idx);
                            dedup_indices.push(new_idx);
                            dedup_positions.push(x);
                            dedup_positions.push(y);
                            dedup_positions.push(z);
                        }
                    }

                    let vert_count_dedup = dedup_positions.len() / 3;
                    if dedup_indices.is_empty() || vert_count_dedup == 0 {
                        continue;
                    }

                    // ── Step 2: optional meshopt simplify (no-op without `optimize`) ──
                    let simplified = if meshopt_threshold < 1.0 && dedup_indices.len() >= 3 {
                        meshopt_simplify(
                            &dedup_indices,
                            &dedup_positions,
                            meshopt_threshold,
                            meshopt_target_error,
                        )
                    } else {
                        dedup_indices
                    };

                    // ── Step 3: drop degenerate/sliver triangles (C++ cleanDegenerateTriangles) ──
                    let cleaned = clean_degenerate_triangles(&dedup_positions, &simplified);

                    // ── Step 4: cache-optimise, then compact to ONLY the referenced vertices ──
                    // (C++'s second position_index_map pass — drops verts orphaned by
                    // simplify / degenerate removal instead of appending the whole weld.)
                    let optimized = if cleaned.is_empty() {
                        Vec::new()
                    } else {
                        meshopt_cache(&cleaned, vert_count_dedup)
                    };

                    new_starts.insert(*id, new_indices.len() as u32);
                    let mut remap: HashMap<u32, u32> = HashMap::new();
                    for &idx in &optimized {
                        let g = *remap.entry(idx).or_insert_with(|| {
                            let n = (new_positions.len() / 3) as u32;
                            let v = idx as usize * 3;
                            new_positions.extend_from_slice(&dedup_positions[v..v + 3]);
                            n
                        });
                        new_indices.push(g);
                    }
                    new_counts.insert(*id, new_indices.len() as u32 - new_starts[id]);
                }

                // Update node draw ranges
                for (id, node) in nodes.iter_mut() {
                    if node.color_with_alpha == color && !node.primitives.is_empty() {
                        if let Some(&s) = new_starts.get(id) {
                            node.start = s;
                        }
                        if let Some(&c) = new_counts.get(id) {
                            node.count = c;
                        }
                    }
                }

                let vc = new_positions.len() / 3;
                (new_indices, new_positions, vc)
            } else {
                let vc = raw_positions.len() / 3;
                (raw_indices, raw_positions, vc)
            };

            // Recompute min/max after dedup
            let (mut fx_min, mut fy_min, mut fz_min) = (f32::MAX, f32::MAX, f32::MAX);
            let (mut fx_max, mut fy_max, mut fz_max) = (f32::MIN, f32::MIN, f32::MIN);
            for i in 0..vert_count {
                let (x, y, z) = (
                    final_positions[i * 3],
                    final_positions[i * 3 + 1],
                    final_positions[i * 3 + 2],
                );
                if x < fx_min {
                    fx_min = x;
                }
                if x > fx_max {
                    fx_max = x;
                }
                if y < fy_min {
                    fy_min = y;
                }
                if y > fy_max {
                    fy_max = y;
                }
                if z < fz_min {
                    fz_min = z;
                }
                if z > fz_max {
                    fz_max = z;
                }
            }

            // Update global bbox
            update_bbox(
                &mut global_bbox,
                fx_min,
                fy_min,
                fz_min,
                fx_max,
                fy_max,
                fz_max,
            );

            // ── append to binary buffer ───────────────────────────────────
            let bv1_offset = bin.len();
            for &v in &final_indices {
                bin.extend_from_slice(&v.to_le_bytes());
            }
            let bv1_len = bin.len() - bv1_offset;

            let bv2_offset = bin.len();
            for &v in &final_positions {
                bin.extend_from_slice(&v.to_le_bytes());
            }
            let bv2_len = bin.len() - bv2_offset;

            // ── bufferViews ───────────────────────────────────────────────
            let bv1_idx = buffer_views.len();
            buffer_views.push(json!({
                "buffer": 0,
                "byteOffset": bv1_offset,
                "byteLength": bv1_len,
                "target": 34963  // ELEMENT_ARRAY_BUFFER
            }));
            let bv2_idx = buffer_views.len();
            buffer_views.push(json!({
                "buffer": 0,
                "byteOffset": bv2_offset,
                "byteLength": bv2_len,
                "target": 34962  // ARRAY_BUFFER
            }));

            // ── accessors ────────────────────────────────────────────────
            let acc1_idx = accessor_count;
            accessors.push(json!({
                "bufferView": bv1_idx,
                "byteOffset": 0,
                "componentType": 5125, // UNSIGNED_INT
                "count": final_indices.len(),
                "type": "SCALAR",
                "min": [0],
                "max": [vert_count.saturating_sub(1)]
            }));
            accessor_count += 1;

            let acc2_idx = accessor_count;
            accessors.push(json!({
                "bufferView": bv2_idx,
                "byteOffset": 0,
                "componentType": 5126, // FLOAT
                "count": vert_count,
                "type": "VEC3",
                "min": [fx_min, fy_min, fz_min],
                "max": [fx_max, fy_max, fz_max]
            }));
            accessor_count += 1;

            // ── material ─────────────────────────────────────────────────
            let mat_idx = materials.len();
            materials.push(material_json(color));

            // ── mesh + node ───────────────────────────────────────────────
            let mesh_idx = meshes.len();
            meshes.push(json!({
                "primitives": [{
                    "attributes": { "POSITION": acc2_idx },
                    "indices": acc1_idx,
                    "material": mat_idx,
                    "mode": 4  // TRIANGLES
                }]
            }));

            let n_idx = gltf_nodes.len();
            gltf_nodes.push(json!({
                "mesh": mesh_idx,
                "name": format!("node{}", node_count)
            }));
            scene_nodes.push(n_idx as u32);

            // ── draw_ranges extras ────────────────────────────────────────
            let mut record = serde_json::Map::new();
            for id in &sorted_ids {
                let node = nodes.get(id).unwrap();
                if node.color_with_alpha != color || node.primitives.is_empty() || node.count == 0 {
                    continue;
                }
                record.insert(node.id.to_string(), json!([node.start, node.count]));
            }
            scene_extras.insert(
                format!("draw_ranges_node{}", node_count),
                Value::Object(record),
            );

            node_count += 1;
        }

        // ── id_hierarchy extra ────────────────────────────────────────────
        let mut hier = serde_json::Map::new();
        for id in &sorted_ids {
            let node = nodes.get(id).unwrap();
            let parent = if node.parent_id == 0 {
                "*".to_string()
            } else {
                node.parent_id.to_string()
            };
            hier.insert(node.id.to_string(), json!([node.name, parent]));
        }
        scene_extras.insert("id_hierarchy".to_string(), Value::Object(hier));

        // ── assemble glTF JSON ────────────────────────────────────────────
        // Pad binary buffer to 4-byte alignment
        while bin.len() % 4 != 0 {
            bin.push(0);
        }

        let gltf_json = json!({
            "asset": {
                "version": "2.0",
                "generator": "rvm2glb",
                "extras": { "web3dversion": 2 }
            },
            "scene": 0,
            "scenes": [{
                "nodes": scene_nodes,
                "extras": Value::Object(scene_extras)
            }],
            "nodes": gltf_nodes,
            "meshes": meshes,
            "materials": materials,
            "accessors": accessors,
            "bufferViews": buffer_views,
            "buffers": [{
                "byteLength": bin.len()
            }]
        });

        let glb = frame_glb(gltf_json.to_string().as_bytes(), &bin);
        (glb, global_bbox)
    }

    /// Build GLB bytes for one "root" in INSTANCED mode: each unique
    /// `shape_key` is triangulated once in local space and emitted as one mesh per
    /// (shape, colour); every occurrence becomes one glTF node carrying its world
    /// placement as `matrix` (with the Z-up→Y-up swap folded in). Plain glTF mesh
    /// reuse — no extension. Geometry is stored once per shape instead of baked per
    /// occurrence. Returns (glb_bytes, bbox).
    pub fn build_instanced(
        nodes: &HashMap<u32, MetaNode>,
        tolerance: f32,
        line_width: f32,
        align_segments: bool,
        highlight: bool,
        remove_empty: bool,
        cleanup: &Cleanup,
    ) -> (Vec<u8>, BBox3) {
        let mut bin: Vec<u8> = Vec::new();
        let mut buffer_views: Vec<Value> = Vec::new();
        let mut accessors: Vec<Value> = Vec::new();
        let mut materials: Vec<Value> = Vec::new();
        let mut meshes: Vec<Value> = Vec::new();
        let mut global_bbox = BBox3::default();

        let mut sorted_ids: Vec<u32> = nodes.keys().copied().collect();
        sorted_ids.sort();
        // Honour remove_empty (default): drop empty leaves / wholly-empty branches.
        let sorted_ids = if remove_empty {
            nodes_with_geometry(nodes, &sorted_ids)
        } else {
            sorted_ids
        };

        // One glTF node per RVM node (containers + leaves), in sorted order; each
        // primitive becomes an extra child node holding the shared mesh + its matrix.
        let node_index: HashMap<u32, u32> = sorted_ids
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, i as u32))
            .collect();
        let k = sorted_ids.len() as u32; // index of the first prim node

        let name_of = |node: &MetaNode| {
            if node.name.is_empty() {
                node.id.to_string()
            } else {
                node.name.clone()
            }
        };

        let mut mat_for_color: HashMap<u32, usize> = HashMap::new();
        let mut shape_geo: HashMap<u64, ShapeGeo> = HashMap::new();
        let mut mesh_for: HashMap<(u64, u32), usize> = HashMap::new();
        let mut total_instances = 0usize;

        // --highlight-instance: count occurrences per shape so a shape shared by ≥2
        // occurrences (actually instanced) renders yellow and one-offs render grey.
        const HL_GREY: u32 = 0xFF_80_80_80;
        const HL_YELLOW: u32 = 0xFF_FF_FF_00;
        let mut occ_count: HashMap<u64, usize> = HashMap::new();
        if highlight {
            for id in &sorted_ids {
                for prim in &nodes[id].primitives {
                    *occ_count.entry(prim.shape_key).or_insert(0) += 1;
                }
            }
        }

        // Prim instance nodes (mesh + matrix), grouped by their owning RVM node.
        let mut prim_nodes: Vec<Value> = Vec::new();
        let mut node_prims: HashMap<u32, Vec<u32>> = HashMap::new();

        for id in &sorted_ids {
            let node = nodes.get(id).unwrap();
            for prim in &node.primitives {
                let key = prim.shape_key;
                let color = if highlight {
                    if occ_count.get(&key).copied().unwrap_or(0) >= 2 {
                        HL_YELLOW
                    } else {
                        HL_GREY
                    }
                } else {
                    node.color_with_alpha
                };

                // Triangulate this shape once, building its shared accessors.
                if !shape_geo.contains_key(&key) {
                    match build_shape_geo(
                        &prim.shape,
                        tolerance,
                        line_width,
                        prim.world_transform.get_scale(),
                        align_segments,
                        cleanup,
                        &mut bin,
                        &mut buffer_views,
                        &mut accessors,
                    ) {
                        Some(sg) => {
                            shape_geo.insert(key, sg);
                        }
                        None => continue,
                    }
                }
                let sg = shape_geo.get(&key).unwrap();
                let (pos_acc, idx_acc, sg_min, sg_max) =
                    (sg.pos_acc, sg.idx_acc, sg.aabb_min, sg.aabb_max);

                // Material per colour (deduped).
                let mat_idx = match mat_for_color.get(&color) {
                    Some(&m) => m,
                    None => {
                        materials.push(material_json(color));
                        let m = materials.len() - 1;
                        mat_for_color.insert(color, m);
                        m
                    }
                };

                // Mesh per (shape, colour) — shares the shape's accessors, but glTF
                // pins the material on the primitive so colour variants need a mesh.
                let mesh_idx = match mesh_for.get(&(key, color)) {
                    Some(&m) => m,
                    None => {
                        meshes.push(json!({
                            "primitives": [{
                                "attributes": { "POSITION": pos_acc },
                                "indices": idx_acc,
                                "material": mat_idx,
                                "mode": 4
                            }]
                        }));
                        let m = meshes.len() - 1;
                        mesh_for.insert((key, color), m);
                        m
                    }
                };

                // Each primitive is a child node (shared mesh + its own matrix) under
                // its component's node. Unnamed — the component node carries the name.
                let matrix = node_matrix_zup_to_yup(&prim.world_transform);
                let gltf_idx = k + prim_nodes.len() as u32;
                prim_nodes.push(json!({ "mesh": mesh_idx, "matrix": matrix }));
                node_prims.entry(*id).or_default().push(gltf_idx);
                total_instances += 1;

                // Expand global bbox via the 8 transformed corners of the local AABB.
                for &cx in &[sg_min[0], sg_max[0]] {
                    for &cy in &[sg_min[1], sg_max[1]] {
                        for &cz in &[sg_min[2], sg_max[2]] {
                            let (x, y, z) = apply_matrix(&matrix, cx, cy, cz);
                            update_bbox(&mut global_bbox, x, y, z, x, y, z);
                        }
                    }
                }
            }
        }

        // Build the RVM-node tree (parents inside this exported subtree).
        let mut children: HashMap<u32, Vec<u32>> = HashMap::new();
        for id in &sorted_ids {
            let pid = nodes.get(id).unwrap().parent_id;
            if node_index.contains_key(&pid) {
                children.entry(pid).or_default().push(node_index[id]);
            }
        }

        // Emit one node per RVM node (name + children: child RVM nodes + own prim
        // nodes), then append the prim nodes. Roots = nodes whose parent is outside.
        let mut gltf_nodes: Vec<Value> = Vec::with_capacity(sorted_ids.len() + prim_nodes.len());
        let mut scene_nodes: Vec<u32> = Vec::new();
        for id in &sorted_ids {
            let node = nodes.get(id).unwrap();
            let mut nj = serde_json::Map::new();
            nj.insert("name".to_string(), json!(name_of(node)));
            let mut ch: Vec<u32> = children.get(id).cloned().unwrap_or_default();
            if let Some(pn) = node_prims.get(id) {
                ch.extend_from_slice(pn);
            }
            if !ch.is_empty() {
                nj.insert("children".to_string(), json!(ch));
            }
            gltf_nodes.push(Value::Object(nj));
            if !node_index.contains_key(&node.parent_id) {
                scene_nodes.push(node_index[id]);
            }
        }
        gltf_nodes.extend(prim_nodes);

        println!(
            "Instanced: {} unique shapes, {} instances, {} meshes, {} nodes",
            shape_geo.len(),
            total_instances,
            meshes.len(),
            gltf_nodes.len()
        );

        // Instanced output is plain glTF: no web3dversion / id_hierarchy / draw_ranges
        // extras (those are the merged web3d contract). The RVM hierarchy is the glTF
        // node tree; component nodes carry the RVM name (duplicates are expected).
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        let gltf_json = json!({
            "asset": { "version": "2.0", "generator": "rvm2glb" },
            "scene": 0,
            "scenes": [{ "nodes": scene_nodes }],
            "nodes": gltf_nodes,
            "meshes": meshes,
            "materials": materials,
            "accessors": accessors,
            "bufferViews": buffer_views,
            "buffers": [{ "byteLength": bin.len() }]
        });
        let glb = frame_glb(gltf_json.to_string().as_bytes(), &bin);
        (glb, global_bbox)
    }

    /// Build GLB bytes for one "root" in STANDARD mode: neither merged nor instanced.
    /// Emits the native glTF node tree mirroring the RVM hierarchy — one node per RVM
    /// node (container *and* leaf), wired parent→`children`, with a mesh on the nodes
    /// that have geometry (that component's world-space geometry, no merge, no dedup).
    /// Plain glTF (no extras); every node's `name` is the component's actual RVM name,
    /// so the hierarchy is navigable without id_hierarchy. Returns (glb_bytes, bbox).
    pub fn build_standard(
        nodes: &HashMap<u32, MetaNode>,
        remove_empty: bool,
        cleanup: &Cleanup,
    ) -> (Vec<u8>, BBox3) {
        let mut bin: Vec<u8> = Vec::new();
        let mut buffer_views: Vec<Value> = Vec::new();
        let mut accessors: Vec<Value> = Vec::new();
        let mut materials: Vec<Value> = Vec::new();
        let mut meshes: Vec<Value> = Vec::new();
        let mut global_bbox = BBox3::default();
        let mut mat_for_color: HashMap<u32, usize> = HashMap::new();

        let mut sorted_ids: Vec<u32> = nodes.keys().copied().collect();
        sorted_ids.sort();
        // Honour remove_empty (default): drop empty leaves / wholly-empty branches.
        let sorted_ids = if remove_empty {
            nodes_with_geometry(nodes, &sorted_ids)
        } else {
            sorted_ids
        };

        // One glTF node per RVM node, in sorted order → index = position in gltf_nodes.
        let node_index: HashMap<u32, u32> = sorted_ids
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, i as u32))
            .collect();

        let name_of = |node: &MetaNode| {
            if node.name.is_empty() {
                node.id.to_string()
            } else {
                node.name.clone()
            }
        };

        // Pass 1: build a mesh for every node that has geometry.
        let mut node_mesh: HashMap<u32, usize> = HashMap::new();
        for id in &sorted_ids {
            let node = nodes.get(id).unwrap();
            if node.primitives.is_empty() {
                continue;
            }
            // Concatenate this component's primitives into one world-space (Y-up) mesh.
            let mut positions: Vec<f32> = Vec::new();
            let mut indices: Vec<u32> = Vec::new();
            let mut offset = 0u32;
            for prim in &node.primitives {
                for &idx in &prim.indices {
                    indices.push(idx + offset);
                }
                for i in 0..prim.vertices_n as usize {
                    let u = i * 3;
                    let (x, y, z) = rotate_z_up_to_y_up(
                        prim.vertices[u],
                        prim.vertices[u + 1],
                        prim.vertices[u + 2],
                    );
                    positions.extend_from_slice(&[x, y, z]);
                }
                offset += prim.vertices_n;
            }
            let sg = match push_mesh(
                &positions,
                &indices,
                cleanup,
                &mut bin,
                &mut buffer_views,
                &mut accessors,
            ) {
                Some(sg) => sg,
                None => continue,
            };
            let mat_idx = *mat_for_color
                .entry(node.color_with_alpha)
                .or_insert_with(|| {
                    materials.push(material_json(node.color_with_alpha));
                    materials.len() - 1
                });
            let mesh_idx = meshes.len();
            meshes.push(json!({
                "name": name_of(node),
                "primitives": [{
                    "attributes": { "POSITION": sg.pos_acc },
                    "indices": sg.idx_acc,
                    "material": mat_idx,
                    "mode": 4
                }]
            }));
            node_mesh.insert(*id, mesh_idx);
            update_bbox(
                &mut global_bbox,
                sg.aabb_min[0],
                sg.aabb_min[1],
                sg.aabb_min[2],
                sg.aabb_max[0],
                sg.aabb_max[1],
                sg.aabb_max[2],
            );
        }

        // Pass 2: child lists (only for parents that live in this exported subtree).
        let mut children: HashMap<u32, Vec<u32>> = HashMap::new();
        for id in &sorted_ids {
            let pid = nodes.get(id).unwrap().parent_id;
            if node_index.contains_key(&pid) {
                children.entry(pid).or_default().push(node_index[id]);
            }
        }

        // Pass 3: emit one node per RVM node; scene roots are nodes whose parent is not
        // part of this subtree (parent_id 0, or an ancestor above the export level).
        let mut gltf_nodes: Vec<Value> = Vec::with_capacity(sorted_ids.len());
        let mut scene_nodes: Vec<u32> = Vec::new();
        for id in &sorted_ids {
            let node = nodes.get(id).unwrap();
            let mut nj = serde_json::Map::new();
            nj.insert("name".to_string(), json!(name_of(node)));
            if let Some(&m) = node_mesh.get(id) {
                nj.insert("mesh".to_string(), json!(m));
            }
            if let Some(ch) = children.get(id) {
                nj.insert("children".to_string(), json!(ch));
            }
            gltf_nodes.push(Value::Object(nj));
            if !node_index.contains_key(&node.parent_id) {
                scene_nodes.push(node_index[id]);
            }
        }

        println!(
            "Standard: {} nodes ({} with geometry), {} roots",
            gltf_nodes.len(),
            node_mesh.len(),
            scene_nodes.len()
        );

        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        let gltf_json = json!({
            "asset": { "version": "2.0", "generator": "rvm2glb" },
            "scene": 0,
            "scenes": [{ "nodes": scene_nodes }],
            "nodes": gltf_nodes,
            "meshes": meshes,
            "materials": materials,
            "accessors": accessors,
            "bufferViews": buffer_views,
            "buffers": [{ "byteLength": bin.len() }]
        });
        let glb = frame_glb(gltf_json.to_string().as_bytes(), &bin);
        (glb, global_bbox)
    }
}

/// Shared per-shape geometry: accessor indices + the local-space AABB.
struct ShapeGeo {
    pos_acc: usize,
    idx_acc: usize,
    aabb_min: [f32; 3],
    aabb_max: [f32; 3],
}

/// Cleanup/optimisation parameters shared by both writers (the `--cleanup-*` and
/// `--meshopt-*` CLI flags).
#[derive(Clone, Copy)]
pub struct Cleanup {
    pub cleanup_positions: bool,
    pub cleanup_precision: u8,
    pub meshopt_threshold: f32,
    pub meshopt_target_error: f32,
}

/// Drop degenerate triangles — a direct port of the C++ `cleanDegenerateTriangles`.
/// Removes any triangle with a repeated index, two coincident vertices, or zero area
/// (`< 1e-8`); the tessellator emits many such slivers at caps, poles and seams.
fn clean_degenerate_triangles(positions: &[f32], indices: &[u32]) -> Vec<u32> {
    const EPSILON: f32 = 1e-8;
    let pos = |i: u32| -> [f32; 3] {
        let v = i as usize * 3;
        [positions[v], positions[v + 1], positions[v + 2]]
    };
    let mut out = Vec::with_capacity(indices.len());
    for tri in indices.chunks_exact(3) {
        let (a, b, c) = (tri[0], tri[1], tri[2]);
        if a == b || b == c || c == a {
            continue;
        }
        let (p0, p1, p2) = (pos(a), pos(b), pos(c));
        if p0 == p1 || p1 == p2 || p2 == p0 {
            continue;
        }
        let ab = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let ac = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
        let cross = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let area = 0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
        if area < EPSILON {
            continue;
        }
        out.extend_from_slice(&[a, b, c]);
    }
    out
}

/// Precision-keyed vertex weld + optional meshopt simplify + degenerate-triangle cull +
/// cache/fetch optimise on a standalone mesh — the same steps the merged path applies per
/// node, applied once per unique instanced shape. Drops vertices left unreferenced.
fn weld_and_simplify(positions: &[f32], indices: &[u32], c: &Cleanup) -> (Vec<f32>, Vec<u32>) {
    // Step 1: precision-keyed weld (rounds coords to `cleanup_precision` decimals).
    let (mut pos, mut idx) = if c.cleanup_positions {
        let scale = 10i64.pow(c.cleanup_precision as u32) as f32;
        let mut key_map: HashMap<(i64, i64, i64), u32> = HashMap::new();
        let mut new_pos: Vec<f32> = Vec::new();
        let mut new_idx: Vec<u32> = Vec::with_capacity(indices.len());
        for &i in indices {
            let v = i as usize * 3;
            let (x, y, z) = (positions[v], positions[v + 1], positions[v + 2]);
            let key = (
                (x * scale).round() as i64,
                (y * scale).round() as i64,
                (z * scale).round() as i64,
            );
            let ni = *key_map.entry(key).or_insert_with(|| {
                let n = (new_pos.len() / 3) as u32;
                new_pos.extend_from_slice(&[x, y, z]);
                n
            });
            new_idx.push(ni);
        }
        (new_pos, new_idx)
    } else {
        (positions.to_vec(), indices.to_vec())
    };

    // Step 2: optional meshopt simplify (no-op without `optimize`). Match C++: exact
    // target + LockBorder so primitive-boundary verts survive.
    if c.meshopt_threshold < 1.0 && idx.len() >= 3 && pos.len() >= 3 {
        idx = meshopt_simplify(&idx, &pos, c.meshopt_threshold, c.meshopt_target_error);
    }

    // Step 3: drop degenerate/sliver triangles (matches C++ cleanDegenerateTriangles).
    idx = clean_degenerate_triangles(&pos, &idx);

    // Step 4: vertex-cache optimise, then compact to only the referenced vertices
    // (a vertex-fetch optimise that also drops verts simplify/cleanup left unused).
    if !idx.is_empty() && !pos.is_empty() {
        idx = meshopt_cache(&idx, pos.len() / 3);
        let mut remap: HashMap<u32, u32> = HashMap::new();
        let mut cpos: Vec<f32> = Vec::new();
        for i in idx.iter_mut() {
            let ni = *remap.entry(*i).or_insert_with(|| {
                let n = (cpos.len() / 3) as u32;
                let v = *i as usize * 3;
                cpos.extend_from_slice(&pos[v..v + 3]);
                n
            });
            *i = ni;
        }
        pos = cpos;
    }
    (pos, idx)
}

/// Weld+simplify a standalone mesh (per the CLI flags) and append its index + POSITION
/// buffers, bufferViews and accessors. Returns the accessor indices + AABB, or None if
/// the mesh is empty. Shared by the instanced (local) and standard (world) writers.
fn push_mesh(
    positions: &[f32],
    indices: &[u32],
    cleanup: &Cleanup,
    bin: &mut Vec<u8>,
    buffer_views: &mut Vec<Value>,
    accessors: &mut Vec<Value>,
) -> Option<ShapeGeo> {
    let (vertices, indices) = weld_and_simplify(positions, indices, cleanup);
    if vertices.is_empty() || indices.is_empty() {
        return None;
    }

    // Index buffer (u32) — offsets stay 4-aligned (all data is f32/u32).
    let idx_off = bin.len();
    for &v in &indices {
        bin.extend_from_slice(&v.to_le_bytes());
    }
    let idx_len = bin.len() - idx_off;

    // Position buffer.
    let pos_off = bin.len();
    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];
    for v in vertices.chunks_exact(3) {
        for k in 0..3 {
            if v[k] < min[k] {
                min[k] = v[k];
            }
            if v[k] > max[k] {
                max[k] = v[k];
            }
            bin.extend_from_slice(&v[k].to_le_bytes());
        }
    }
    let pos_len = bin.len() - pos_off;
    let vert_count = vertices.len() / 3;

    let bv_idx = buffer_views.len();
    buffer_views.push(
        json!({ "buffer": 0, "byteOffset": idx_off, "byteLength": idx_len, "target": 34963 }),
    );
    let bv_pos = buffer_views.len();
    buffer_views.push(
        json!({ "buffer": 0, "byteOffset": pos_off, "byteLength": pos_len, "target": 34962 }),
    );

    let idx_acc = accessors.len();
    accessors.push(json!({
        "bufferView": bv_idx, "byteOffset": 0, "componentType": 5125,
        "count": indices.len(), "type": "SCALAR", "min": [0], "max": [vert_count.saturating_sub(1)]
    }));
    let pos_acc = accessors.len();
    accessors.push(json!({
        "bufferView": bv_pos, "byteOffset": 0, "componentType": 5126,
        "count": vert_count, "type": "VEC3", "min": min, "max": max
    }));

    Some(ShapeGeo {
        pos_acc,
        idx_acc,
        aabb_min: min,
        aabb_max: max,
    })
}

/// Triangulate `shape` once in local space, then weld+emit it (instanced path).
/// Returns its accessor indices + local AABB, or None for shapes with no geometry.
fn build_shape_geo(
    shape: &crate::geometry::GeometryShape,
    tolerance: f32,
    line_width: f32,
    seg_scale: f32,
    align_segments: bool,
    cleanup: &Cleanup,
    bin: &mut Vec<u8>,
    buffer_views: &mut Vec<Value>,
    accessors: &mut Vec<Value>,
) -> Option<ShapeGeo> {
    let tri = tessellate_local(shape, tolerance, line_width, seg_scale, align_segments)?;
    if tri.vertices_n == 0 || tri.indices.is_empty() {
        return None;
    }
    push_mesh(
        &tri.vertices,
        &tri.indices,
        cleanup,
        bin,
        buffer_views,
        accessors,
    )
}

/// glTF node `matrix` (column-major 4×4) = Rzy · world_transform, where Rzy is the
/// Z-up→Y-up swap (x,y,z)→(x,z,−y). Applying the swap on the left keeps the shared
/// mesh in local space while reproducing the merged path's world Y-up positions.
fn node_matrix_zup_to_yup(m: &Mat3x4f) -> [f64; 16] {
    let d = &m.data; // column-major 3×4: col0,col1,col2,translation
    [
        d[0] as f64,
        d[2] as f64,
        -(d[1] as f64),
        0.0, // col0
        d[3] as f64,
        d[5] as f64,
        -(d[4] as f64),
        0.0, // col1
        d[6] as f64,
        d[8] as f64,
        -(d[7] as f64),
        0.0, // col2
        d[9] as f64,
        d[11] as f64,
        -(d[10] as f64),
        1.0, // translation
    ]
}

/// Transform a point by a column-major 4×4 matrix.
fn apply_matrix(m: &[f64; 16], x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    let (x, y, z) = (x as f64, y as f64, z as f64);
    let px = m[0] * x + m[4] * y + m[8] * z + m[12];
    let py = m[1] * x + m[5] * y + m[9] * z + m[13];
    let pz = m[2] * x + m[6] * y + m[10] * z + m[14];
    (px as f32, py as f32, pz as f32)
}

/// PBR material JSON from a packed `0xAARRGGBB` colour (shared by both writers).
fn material_json(color: u32) -> Value {
    let r = ((color >> 16) & 0xff) as f64 / 255.0;
    let g = ((color >> 8) & 0xff) as f64 / 255.0;
    let b = (color & 0xff) as f64 / 255.0;
    let a = ((color >> 24) & 0xff) as f64 / 255.0;
    let (alpha_mode, final_a) = if (a - 1.0).abs() > 1e-4 {
        ("BLEND", 1.0 - a)
    } else {
        ("OPAQUE", 1.0)
    };
    let mut mat = json!({
        "pbrMetallicRoughness": {
            "baseColorFactor": [r, g, b, final_a],
            "metallicFactor": 0.0,
            "roughnessFactor": 1.0
        },
        "doubleSided": true
    });
    if alpha_mode != "OPAQUE" {
        mat["alphaMode"] = json!(alpha_mode);
    }
    mat
}

/// Frame a glTF JSON byte string + already-4-byte-padded binary buffer into GLB
/// bytes (12-byte header + JSON chunk + optional BIN chunk). Shared by both writers.
fn frame_glb(json_bytes: &[u8], bin: &[u8]) -> Vec<u8> {
    let json_padded_len = (json_bytes.len() + 3) & !3;
    let json_padding = json_padded_len - json_bytes.len();
    let total_len = 12 + 8 + json_padded_len + if bin.is_empty() { 0 } else { 8 + bin.len() };

    let mut glb = Vec::with_capacity(total_len);
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2u32.to_le_bytes());
    glb.extend_from_slice(&(total_len as u32).to_le_bytes());

    glb.extend_from_slice(&(json_padded_len as u32).to_le_bytes());
    glb.extend_from_slice(&0x4E4F534Au32.to_le_bytes()); // "JSON"
    glb.extend_from_slice(json_bytes);
    for _ in 0..json_padding {
        glb.push(0x20); // space padding
    }

    if !bin.is_empty() {
        glb.extend_from_slice(&(bin.len() as u32).to_le_bytes());
        glb.extend_from_slice(&0x004E4942u32.to_le_bytes()); // "BIN\0"
        glb.extend_from_slice(bin);
    }
    glb
}

fn update_bbox(
    b: &mut BBox3,
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
) {
    if min_x < b.min_x {
        b.min_x = min_x;
    }
    if min_y < b.min_y {
        b.min_y = min_y;
    }
    if min_z < b.min_z {
        b.min_z = min_z;
    }
    if max_x > b.max_x {
        b.max_x = max_x;
    }
    if max_y > b.max_y {
        b.max_y = max_y;
    }
    if max_z > b.max_z {
        b.max_z = max_z;
    }
}
