// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 tess.c/h + sweep.c/h + tesselator.h
//
// This module is the complete tessellator: public API + full sweep line algorithm.
// The C code is split across tess.c and sweep.c; they're merged here since both
// share the same internal state (TESStesselator).

mod api;
mod geometry;
mod output;
#[cfg(test)]
mod tests;

pub use api::TessellatorApi;

use geometry::{
    check_orientation, compute_intersect_coords, compute_normal, dot, is_valid_coord, long_axis,
};

use crate::dict::{Dict, NodeIdx, DICT_HEAD};
use crate::geom::{edge_intersect, edge_sign, vert_eq, vert_leq, Real};
use crate::mesh::{EdgeIdx, Mesh, VertIdx, E_HEAD, INVALID, V_HEAD};
use crate::priorityq::INVALID_HANDLE;
use crate::sweep::ActiveRegion;

// ─────────────────────────────── Public types ──────────────────────────────────

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum WindingRule {
    Odd,
    NonZero,
    Positive,
    Negative,
    AbsGeqTwo,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ElementType {
    Polygons,
    ConnectedPolygons,
    BoundaryContours,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TessOption {
    ConstrainedDelaunayTriangulation,
    ReverseContours,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TessStatus {
    Ok,
    OutOfMemory,
    InvalidInput,
}

pub const TESS_UNDEF: u32 = u32::MAX;
// Max magnitude that input coordinates can safely take without losing
// precision in the sweep.  f64's 52-bit mantissa keeps integer coords
// exact up to 2^53; we keep a conservative margin.
const MAX_VALID_COORD: Real = (1u64 << 50) as Real;
const MIN_VALID_COORD: Real = -MAX_VALID_COORD;

type RegionIdx = u32;

// ─────────────────────────── Tessellator ──────────────────────────────────────

pub struct Tessellator {
    mesh: Option<Mesh>,
    pub status: TessStatus,
    normal: [Real; 3],
    s_unit: [Real; 3],
    t_unit: [Real; 3],
    bmin: [Real; 2],
    bmax: [Real; 2],
    process_cdt: bool,
    reverse_contours: bool,
    winding_rule: WindingRule,

    // Sweep state
    dict: Dict,
    /// Intersection vertices inserted during sweep (heap replacement).
    /// Each entry is a VertIdx; ordering is done by coordinate lookup.
    intersection_verts: Vec<VertIdx>,
    next_isect_handle: i32,
    event: VertIdx,
    event_s: Real,
    event_t: Real,

    // Region arena
    regions: Vec<Option<ActiveRegion>>,
    region_free: Vec<RegionIdx>,

    // STEP2GLB PATCH: set when the sweep references a freed/invalid region;
    // makes the run fail soft (return false) instead of panicking. `Cell` so the
    // `&self` `region()` accessor can flag it. `dummy_region` is handed back on
    // the failing access so no `unwrap()` panics.
    aborted: std::cell::Cell<bool>,
    dummy_region: ActiveRegion,

    // Output
    pub out_vertices: Vec<Real>,
    pub out_vertex_indices: Vec<u32>,
    pub out_elements: Vec<u32>,
    /// Per triangle-vertex edge-flag (parallel to `out_elements`).
    ///
    /// `1` when the polygon edge starting at this vertex (going to the next
    /// vertex in CCW order within the same triangle) is an **original
    /// boundary edge** of the input polygon; `0` when the edge is a new
    /// interior edge added by the tessellation sweep.
    ///
    /// Parallels the C libtess2 / agg-sharp `EdgeFlagCallback` mechanism —
    /// consumers that want analytic per-edge anti-aliasing (halo strips,
    /// conservative outlines) look at each triangle's three flags and only
    /// expand the sides that are actual polygon boundaries.
    ///
    /// Populated for `ElementType::Polygons` and `ElementType::ConnectedPolygons`;
    /// empty for `ElementType::BoundaryContours` (no triangles are emitted in
    /// that mode).  Length equals `poly_size × element_count`.
    pub out_edge_flags: Vec<u8>,
    pub out_vertex_count: usize,
    pub out_element_count: usize,
    vertex_index_counter: u32,

    // Primary event queue: pre-sorted vertices for the initial sweep phase
    sorted_events: Vec<VertIdx>,
    sorted_event_pos: usize,
    sweep_event_num: u32,
    trace_enabled: bool,
}

impl Tessellator {
    pub fn new() -> Self {
        Tessellator {
            mesh: None,
            status: TessStatus::Ok,
            normal: [0.0; 3],
            s_unit: [0.0; 3],
            t_unit: [0.0; 3],
            bmin: [0.0; 2],
            bmax: [0.0; 2],
            process_cdt: false,
            reverse_contours: false,
            winding_rule: WindingRule::Odd,
            dict: Dict::new(),
            intersection_verts: Vec::new(),
            next_isect_handle: 0,
            event: INVALID,
            event_s: 0.0,
            event_t: 0.0,
            regions: Vec::new(),
            region_free: Vec::new(),
            aborted: std::cell::Cell::new(false),
            dummy_region: ActiveRegion::default(),
            out_vertices: Vec::new(),
            out_vertex_indices: Vec::new(),
            out_elements: Vec::new(),
            out_edge_flags: Vec::new(),
            out_vertex_count: 0,
            out_element_count: 0,
            vertex_index_counter: 0,
            sorted_events: Vec::new(),
            sorted_event_pos: 0,
            sweep_event_num: 0,
            trace_enabled: std::env::var("TESS_TRACE").is_ok(),
        }
    }

    pub fn set_option(&mut self, option: TessOption, value: bool) {
        match option {
            TessOption::ConstrainedDelaunayTriangulation => self.process_cdt = value,
            TessOption::ReverseContours => self.reverse_contours = value,
        }
    }

    /// Add a contour. `size` = 2 or 3 (coords per vertex). `vertices` is flat.
    ///
    /// Input type is `Real` — currently `f64` — to avoid losing precision on
    /// coordinate input.  Callers holding `f32` data should cast element-wise
    /// at the call site.
    pub fn add_contour(&mut self, size: usize, vertices: &[Real]) {
        if self.status != TessStatus::Ok {
            return;
        }
        let size = size.min(3).max(2);
        let count = vertices.len() / size;
        if self.mesh.is_none() {
            self.mesh = Some(Mesh::new());
        }

        let mut e = INVALID;
        for i in 0..count {
            let cx = vertices[i * size];
            let cy = vertices[i * size + 1];
            let cz = if size > 2 {
                vertices[i * size + 2]
            } else {
                0.0
            };

            if !is_valid_coord(cx) || !is_valid_coord(cy) || (size > 2 && !is_valid_coord(cz)) {
                self.status = TessStatus::InvalidInput;
                return;
            }

            let mesh = self.mesh.as_mut().unwrap();
            if e == INVALID {
                let new_e = match mesh.make_edge() {
                    Some(v) => v,
                    None => {
                        self.status = TessStatus::OutOfMemory;
                        return;
                    }
                };
                e = new_e;
                if !mesh.splice(e, e ^ 1) {
                    self.status = TessStatus::OutOfMemory;
                    return;
                }
            } else {
                if mesh.split_edge(e).is_none() {
                    self.status = TessStatus::OutOfMemory;
                    return;
                }
                e = mesh.edges[e as usize].lnext;
            }

            let org = mesh.edges[e as usize].org;
            mesh.verts[org as usize].coords[0] = cx;
            mesh.verts[org as usize].coords[1] = cy;
            mesh.verts[org as usize].coords[2] = cz;
            mesh.verts[org as usize].idx = self.vertex_index_counter;
            self.vertex_index_counter += 1;

            let w = if self.reverse_contours { -1 } else { 1 };
            mesh.edges[e as usize].winding = w;
            mesh.edges[(e ^ 1) as usize].winding = -w;
        }
    }

    pub fn tessellate(
        &mut self,
        winding_rule: WindingRule,
        element_type: ElementType,
        poly_size: usize,
        vertex_size: usize,
        normal: Option<[Real; 3]>,
    ) -> bool {
        if self.status != TessStatus::Ok {
            return false;
        }
        self.winding_rule = winding_rule;
        self.out_vertices.clear();
        self.out_vertex_indices.clear();
        self.out_elements.clear();
        self.out_edge_flags.clear();
        self.out_vertex_count = 0;
        self.out_element_count = 0;
        self.normal = normal.unwrap_or([0.0, 0.0, 0.0]);

        if self.mesh.is_none() {
            self.mesh = Some(Mesh::new());
        }

        if !self.project_polygon() {
            self.status = TessStatus::OutOfMemory;
            return false;
        }

        if !self.compute_interior() {
            if self.status == TessStatus::Ok {
                self.status = TessStatus::OutOfMemory;
            }
            return false;
        }

        let vertex_size = vertex_size.min(3).max(2);
        if element_type == ElementType::BoundaryContours {
            self.output_contours(vertex_size);
        } else {
            self.output_polymesh(element_type, poly_size, vertex_size);
        }

        self.mesh = None;
        self.status == TessStatus::Ok
    }

    // ─────── Accessors ────────────────────────────────────────────────────────

    pub fn vertex_count(&self) -> usize {
        self.out_vertex_count
    }
    pub fn element_count(&self) -> usize {
        self.out_element_count
    }
    pub fn vertices(&self) -> &[Real] {
        &self.out_vertices
    }
    pub fn vertex_indices(&self) -> &[u32] {
        &self.out_vertex_indices
    }
    pub fn elements(&self) -> &[u32] {
        &self.out_elements
    }
    /// Per triangle-vertex edge flags (see [`Tessellator::out_edge_flags`]).
    ///
    /// Returns an empty slice for `ElementType::BoundaryContours`.
    pub fn edge_flags(&self) -> &[u8] {
        &self.out_edge_flags
    }
    pub fn get_status(&self) -> TessStatus {
        self.status
    }

    // ─────── Projection ───────────────────────────────────────────────────────

    fn project_polygon(&mut self) -> bool {
        let mut norm = self.normal;
        let mut computed_normal = false;
        if norm[0] == 0.0 && norm[1] == 0.0 && norm[2] == 0.0 {
            if let Some(ref m) = self.mesh {
                compute_normal(m, &mut norm);
            }
            computed_normal = true;
        }

        let i = long_axis(&norm);
        self.s_unit = [0.0; 3];
        self.t_unit = [0.0; 3];
        self.s_unit[(i + 1) % 3] = 1.0;
        self.t_unit[(i + 2) % 3] = if norm[i] > 0.0 { 1.0 } else { -1.0 };
        let su = self.s_unit;
        let tu = self.t_unit;

        if let Some(ref mut mesh) = self.mesh {
            let mut v = mesh.verts[V_HEAD as usize].next;
            while v != V_HEAD {
                let c = mesh.verts[v as usize].coords;
                mesh.verts[v as usize].s = dot(&c, &su);
                mesh.verts[v as usize].t = dot(&c, &tu);
                v = mesh.verts[v as usize].next;
            }
            if computed_normal {
                check_orientation(mesh);
            }

            let mut first = true;
            let mut v = mesh.verts[V_HEAD as usize].next;
            while v != V_HEAD {
                let vs = mesh.verts[v as usize].s;
                let vt = mesh.verts[v as usize].t;
                if first {
                    self.bmin = [vs, vt];
                    self.bmax = [vs, vt];
                    first = false;
                } else {
                    if vs < self.bmin[0] {
                        self.bmin[0] = vs;
                    }
                    if vs > self.bmax[0] {
                        self.bmax[0] = vs;
                    }
                    if vt < self.bmin[1] {
                        self.bmin[1] = vt;
                    }
                    if vt > self.bmax[1] {
                        self.bmax[1] = vt;
                    }
                }
                v = mesh.verts[v as usize].next;
            }
        }
        true
    }

    // ─────── Main interior computation ───────────────────────────────────────

    fn compute_interior(&mut self) -> bool {
        self.sweep_event_num = 0;

        if !self.remove_degenerate_edges() {
            return false;
        }
        if !self.init_priority_queue() {
            return false;
        }
        if !self.init_edge_dict() {
            return false;
        }

        loop {
            // STEP2GLB PATCH: a region accessor hit a freed/invalid slot; bail
            // out of the sweep with a clean failure instead of continuing on
            // corrupt state (which would eventually panic on wasm).
            if self.aborted.get() {
                return false;
            }
            if self.pq_is_empty() {
                break;
            }

            let v = self.pq_extract_min();
            if v == INVALID {
                break;
            }

            // Coalesce coincident vertices
            loop {
                if self.pq_is_empty() {
                    break;
                }
                let next_v = self.pq_minimum();
                if next_v == INVALID {
                    break;
                }
                let (v_s, v_t) = {
                    let mesh = self.mesh.as_ref().unwrap();
                    (mesh.verts[v as usize].s, mesh.verts[v as usize].t)
                };
                let (nv_s, nv_t) = {
                    let mesh = self.mesh.as_ref().unwrap();
                    (mesh.verts[next_v as usize].s, mesh.verts[next_v as usize].t)
                };
                if !vert_eq(v_s, v_t, nv_s, nv_t) {
                    break;
                }
                let next_v = self.pq_extract_min();
                // Merge next_v into v
                let an1 = self.mesh.as_ref().unwrap().verts[v as usize].an_edge;
                let an2 = self.mesh.as_ref().unwrap().verts[next_v as usize].an_edge;
                if an1 != INVALID && an2 != INVALID {
                    if !self.mesh.as_mut().unwrap().splice(an1, an2) {
                        return false;
                    }
                }
            }

            self.event = v;
            let (v_s, v_t) = {
                let m = self.mesh.as_ref().unwrap();
                (m.verts[v as usize].s, m.verts[v as usize].t)
            };
            self.event_s = v_s;
            self.event_t = v_t;

            if !self.sweep_event(v) {
                return false;
            }
        }

        // STEP2GLB PATCH: also catch a poison set during the final event.
        if self.aborted.get() {
            return false;
        }

        self.done_edge_dict();

        let trace = self.trace_enabled;
        if let Some(ref mut mesh) = self.mesh {
            if trace {
                let mut inside = 0u32;
                let mut outside = 0u32;
                let mut f = mesh.faces[crate::mesh::F_HEAD as usize].next;
                while f != crate::mesh::F_HEAD {
                    let an = mesh.faces[f as usize].an_edge;
                    let mut edge_count = 0u32;
                    if an != INVALID {
                        let mut e = an;
                        loop {
                            edge_count += 1;
                            e = mesh.edges[e as usize].lnext;
                            if e == an { break; }
                            if edge_count > 10000 { break; }
                        }
                    }
                    if mesh.faces[f as usize].inside {
                        inside += 1;
                        eprintln!("R FACE inside edges={}", edge_count);
                    } else {
                        outside += 1;
                    }
                    f = mesh.faces[f as usize].next;
                }
                eprintln!("R FACES inside={} outside={}", inside, outside);
            }
            if !mesh.tessellate_interior() {
                return false;
            }
            if self.process_cdt {
                mesh.refine_delaunay();
            }
        }
        true
    }

    fn remove_degenerate_edges(&mut self) -> bool {
        // Mirrors C RemoveDegenerateEdges exactly
        let mesh = match self.mesh.as_mut() {
            Some(m) => m,
            None => return true,
        };
        let mut e = mesh.edges[E_HEAD as usize].next;
        while e != E_HEAD {
            let mut e_next = mesh.edges[e as usize].next;
            let mut e_lnext = mesh.edges[e as usize].lnext;

            let org = mesh.edges[e as usize].org;
            let dst = mesh.dst(e);
            let valid = org != INVALID
                && dst != INVALID
                && (org as usize) < mesh.verts.len()
                && (dst as usize) < mesh.verts.len();

            if valid {
                let (os, ot) = (mesh.verts[org as usize].s, mesh.verts[org as usize].t);
                let (ds, dt) = (mesh.verts[dst as usize].s, mesh.verts[dst as usize].t);

                if vert_eq(os, ot, ds, dt) && mesh.edges[e_lnext as usize].lnext != e {
                    // Zero-length edge, contour has at least 3 edges
                    mesh.splice(e_lnext, e);
                    if !mesh.delete_edge(e) {
                        return false;
                    }
                    e = e_lnext;
                    e_lnext = mesh.edges[e as usize].lnext;
                }
            }

            // Degenerate contour (one or two edges): e_lnext->lnext == e
            let e_lnext_lnext = mesh.edges[e_lnext as usize].lnext;
            if e_lnext_lnext == e {
                if e_lnext != e {
                    // Advance e_next past e_lnext or its sym
                    if e_lnext == e_next || e_lnext == (e_next ^ 1) {
                        e_next = mesh.edges[e_next as usize].next;
                    }
                    let w1 = mesh.edges[e_lnext as usize].winding;
                    let w2 = mesh.edges[(e_lnext ^ 1) as usize].winding;
                    mesh.edges[e as usize].winding += w1;
                    mesh.edges[(e ^ 1) as usize].winding += w2;
                    if !mesh.delete_edge(e_lnext) {
                        return false;
                    }
                }
                // Advance e_next past e or its sym
                if e == e_next || e == (e_next ^ 1) {
                    e_next = mesh.edges[e_next as usize].next;
                }
                if !mesh.delete_edge(e) {
                    return false;
                }
            }

            e = e_next;
        }
        true
    }

    fn init_priority_queue(&mut self) -> bool {
        let mesh = match self.mesh.as_ref() {
            Some(m) => m,
            None => return true,
        };
        let mut count = 0usize;
        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            count += 1;
            v = mesh.verts[v as usize].next;
        }

        // Collect (s,t,vert_idx) and sort ascending by vert_leq.
        let mut vert_coords: Vec<(Real, Real, VertIdx)> = Vec::with_capacity(count);
        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            vert_coords.push((mesh.verts[v as usize].s, mesh.verts[v as usize].t, v));
            v = mesh.verts[v as usize].next;
        }
        drop(mesh);

        vert_coords.sort_unstable_by(|a, b| {
            if vert_leq(a.0, a.1, b.0, b.1) {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        });

        // Build the sorted event queue. Store each vertex's position as a negative
        // handle (convention: -(index+1)) so that pq_delete can invalidate it.
        self.sorted_events = vert_coords.iter().map(|&(_, _, v)| v).collect();
        self.sorted_event_pos = 0;
        self.intersection_verts.clear();
        self.next_isect_handle = 0;

        // Assign each initial vertex a handle encoding its sorted_events index.
        for (idx, &(_, _, v)) in vert_coords.iter().enumerate() {
            let handle = -(idx as i32 + 1); // negative → sorted_events slot
            self.mesh.as_mut().unwrap().verts[v as usize].pq_handle = handle;
        }

        true
    }

    fn pq_is_empty(&self) -> bool {
        self.sorted_events_min() == INVALID && self.intersection_verts.is_empty()
    }

    fn sorted_events_min(&self) -> VertIdx {
        let mut pos = self.sorted_event_pos;
        while pos < self.sorted_events.len() {
            let v = self.sorted_events[pos];
            if v != INVALID {
                return v;
            }
            pos += 1;
        }
        INVALID
    }

    /// Find the minimum intersection vertex by scanning with coordinate comparison.
    fn isect_minimum(&self) -> VertIdx {
        if self.intersection_verts.is_empty() {
            return INVALID;
        }
        let mesh = self.mesh.as_ref().unwrap();
        let mut best = INVALID;
        for &v in &self.intersection_verts {
            if best == INVALID {
                best = v;
            } else {
                let (bs, bt) = (mesh.verts[best as usize].s, mesh.verts[best as usize].t);
                let (vs, vt) = (mesh.verts[v as usize].s, mesh.verts[v as usize].t);
                if vert_leq(vs, vt, bs, bt) {
                    best = v;
                }
            }
        }
        best
    }

    fn pq_minimum(&self) -> VertIdx {
        let sort_min = self.sorted_events_min();
        let isect_min = self.isect_minimum();

        match (sort_min, isect_min) {
            (INVALID, INVALID) => INVALID,
            (INVALID, h) => h,
            (s, INVALID) => s,
            (s, h) => {
                let mesh = self.mesh.as_ref().unwrap();
                let (ss, st) = (mesh.verts[s as usize].s, mesh.verts[s as usize].t);
                let (hs, ht) = (mesh.verts[h as usize].s, mesh.verts[h as usize].t);
                if vert_leq(ss, st, hs, ht) {
                    s
                } else {
                    h
                }
            }
        }
    }

    fn pq_extract_min(&mut self) -> VertIdx {
        let v = self.pq_minimum();
        if v == INVALID {
            return INVALID;
        }

        if self.sorted_events_min() == v {
            while self.sorted_event_pos < self.sorted_events.len() {
                let s = self.sorted_events[self.sorted_event_pos];
                self.sorted_event_pos += 1;
                if s != INVALID {
                    break;
                }
            }
        } else {
            // Remove from intersection_verts
            if let Some(pos) = self.intersection_verts.iter().position(|&x| x == v) {
                self.intersection_verts.swap_remove(pos);
            }
        }
        v
    }

    fn pq_delete(&mut self, handle: i32) {
        if handle >= 0 {
            // Intersection vertex handle: scan and remove by handle index
            let vert_idx = handle as u32;
            if let Some(pos) = self.intersection_verts.iter().position(|&x| x == vert_idx) {
                self.intersection_verts.swap_remove(pos);
            }
        } else {
            // Sorted-events handle: mark the slot as INVALID
            let idx = (-(handle + 1)) as usize;
            if idx < self.sorted_events.len() {
                self.sorted_events[idx] = INVALID;
            }
        }
    }

    fn pq_insert(&mut self, v: VertIdx) -> i32 {
        self.intersection_verts.push(v);
        // Return the VertIdx itself as the handle (positive, so pq_delete knows it's an intersection vertex)
        v as i32
    }

    // ─────── Edge dictionary initialization ──────────────────────────────────

    fn add_sentinel(&mut self, smin: Real, smax: Real, t: Real) -> bool {
        // Mirror C AddSentinel: create a horizontal edge at height t,
        // going from Org=(smax,t) to Dst=(smin,t), and insert as a sentinel region.
        let e = match self.mesh.as_mut().unwrap().make_edge() {
            Some(e) => e,
            None => return false,
        };
        {
            let mesh = self.mesh.as_mut().unwrap();
            let org = mesh.edges[e as usize].org;
            let dst = mesh.dst(e);
            mesh.verts[org as usize].s = smax;
            mesh.verts[org as usize].t = t;
            mesh.verts[dst as usize].s = smin;
            mesh.verts[dst as usize].t = t;
        }
        // Set the event to Dst (as C does) so edge_leq works during insertion
        let dst = self.mesh.as_ref().unwrap().dst(e);
        let (dst_s, dst_t) = {
            let m = self.mesh.as_ref().unwrap();
            (m.verts[dst as usize].s, m.verts[dst as usize].t)
        };
        self.event = dst;
        self.event_s = dst_s;
        self.event_t = dst_t;

        let reg = self.alloc_region();
        {
            let r = self.region_mut(reg);
            r.e_up = e;
            r.winding_number = 0;
            r.inside = false;
            r.sentinel = true;
            r.dirty = false;
            r.fix_upper_edge = false;
        }

        // Insert the region into the dict using edge_leq ordering
        let node = self.dict_insert_region(reg);
        if node == INVALID {
            return false;
        }
        self.region_mut(reg).node_up = node;

        // Set the edge's active_region so it's recognized as a sentinel edge
        self.mesh.as_mut().unwrap().edges[e as usize].active_region = reg;
        true
    }

    /// Insert a region into the dict at the sorted position (using edge_leq).
    /// Starts search from DICT_HEAD (tail). Returns the new node index.
    fn dict_insert_region(&mut self, reg: RegionIdx) -> NodeIdx {
        self.dict_insert_before(reg, DICT_HEAD)
    }

    /// Insert a region before `start_node` in the dict, walking backward
    /// until the correct sorted position is found. Mirrors C's dictInsertBefore.
    fn dict_insert_before(&mut self, reg: RegionIdx, start_node: NodeIdx) -> NodeIdx {
        let max_dict_iters = self.dict.nodes.len() + 2;
        let mut node = start_node;
        let mut dict_iter = 0usize;
        loop {
            node = self.dict.nodes[node as usize].prev;
            let key = self.dict.nodes[node as usize].key;
            if key == INVALID {
                break; // hit head sentinel
            }
            if self.edge_leq(key, reg) {
                break;
            }
            dict_iter += 1;
            if dict_iter > max_dict_iters {
                break; // degenerate dict list — avoid infinite walk
            }
        }
        // Insert after `node`
        let after = node;
        let before = self.dict.nodes[after as usize].next;
        let new_node = self.dict.nodes.len() as NodeIdx;
        use crate::dict::DictNode;
        let new_dict_node = DictNode {
            key: reg,
            next: before,
            prev: after,
        };
        self.dict.nodes.push(new_dict_node);
        self.dict.nodes[after as usize].next = new_node;
        self.dict.nodes[before as usize].prev = new_node;
        new_node
    }

    fn init_edge_dict(&mut self) -> bool {
        self.dict = Dict::new();

        // Compute sentinel bounds from bounding box + margin (mirrors C InitEdgeDict)
        let w = (self.bmax[0] - self.bmin[0]) + 0.01;
        let h = (self.bmax[1] - self.bmin[1]) + 0.01;
        let smin = self.bmin[0] - w;
        let smax = self.bmax[0] + w;
        let tmin = self.bmin[1] - h;
        let tmax = self.bmax[1] + h;

        // Add bottom sentinel first (at tmin), then top sentinel (at tmax).
        // After insertion with EdgeLeq ordering, top ends up before bottom in the dict.
        if !self.add_sentinel(smin, smax, tmin) {
            return false;
        }
        if !self.add_sentinel(smin, smax, tmax) {
            return false;
        }

        true
    }

    fn done_edge_dict(&mut self) {
        // Remove all sentinel regions
        let mut node = self.dict.min();
        while node != DICT_HEAD {
            let key = self.dict.key(node);
            let next = self.dict.succ(node);
            if key != INVALID {
                let is_sentinel = self.region(key).sentinel;
                if is_sentinel {
                    self.dict.delete(node);
                    self.free_region(key);
                }
            }
            node = next;
        }
    }

    // ─────── Region operations ────────────────────────────────────────────────

    fn alloc_region(&mut self) -> RegionIdx {
        if let Some(idx) = self.region_free.pop() {
            self.regions[idx as usize] = Some(ActiveRegion::default());
            idx
        } else {
            let idx = self.regions.len() as RegionIdx;
            self.regions.push(Some(ActiveRegion::default()));
            idx
        }
    }

    fn free_region(&mut self, idx: RegionIdx) {
        if idx != INVALID {
            self.regions[idx as usize] = None;
            self.region_free.push(idx);
        }
    }

    // STEP2GLB PATCH: fail soft instead of `unwrap()`-ing a freed/invalid
    // region. A degenerate contour can leave the sweep referencing a region
    // slot that is `None`; upstream this panics, which is fine on native
    // (caught by catch_unwind) but aborts the whole wasm module (panic=abort,
    // no unwinding). Here we flag `aborted` and hand back a benign default
    // region (all-zero indices = valid head sentinels, so no out-of-bounds),
    // and the sweep loop bails to a clean `false` (= "tessellation failed",
    // face skipped) on the next event.
    fn region(&self, idx: RegionIdx) -> &ActiveRegion {
        match self.regions.get(idx as usize).and_then(|r| r.as_ref()) {
            Some(r) => r,
            None => {
                self.aborted.set(true);
                &self.dummy_region
            }
        }
    }

    fn region_mut(&mut self, idx: RegionIdx) -> &mut ActiveRegion {
        if self
            .regions
            .get(idx as usize)
            .map_or(true, |r| r.is_none())
        {
            self.aborted.set(true);
            return &mut self.dummy_region;
        }
        self.regions[idx as usize].as_mut().unwrap()
    }

    /// Returns the region index of the dict node's successor region.
    fn region_above(&self, reg: RegionIdx) -> RegionIdx {
        let node = self.region(reg).node_up;
        self.dict.key(self.dict.succ(node))
    }

    /// Returns the region index of the dict node's predecessor region.
    fn region_below(&self, reg: RegionIdx) -> RegionIdx {
        let node = self.region(reg).node_up;
        self.dict.key(self.dict.pred(node))
    }

    /// EdgeLeq: Returns reg1 <= reg2 at the current sweep position (event).
    fn edge_leq(&self, reg1: RegionIdx, reg2: RegionIdx) -> bool {
        let e1 = self.region(reg1).e_up;
        let e2 = self.region(reg2).e_up;
        if e1 == INVALID {
            return true;
        }
        if e2 == INVALID {
            return false;
        }
        let mesh = self.mesh.as_ref().unwrap();

        let e1_dst = mesh.dst(e1);
        let e2_dst = mesh.dst(e2);
        let e1_org = mesh.edges[e1 as usize].org;
        let e2_org = mesh.edges[e2 as usize].org;

        let ev_s = self.event_s;
        let ev_t = self.event_t;

        let (e1ds, e1dt) = (mesh.verts[e1_dst as usize].s, mesh.verts[e1_dst as usize].t);
        let (e2ds, e2dt) = (mesh.verts[e2_dst as usize].s, mesh.verts[e2_dst as usize].t);
        let (e1os, e1ot) = (mesh.verts[e1_org as usize].s, mesh.verts[e1_org as usize].t);
        let (e2os, e2ot) = (mesh.verts[e2_org as usize].s, mesh.verts[e2_org as usize].t);

        if vert_eq(e1ds, e1dt, ev_s, ev_t) {
            if vert_eq(e2ds, e2dt, ev_s, ev_t) {
                if vert_leq(e1os, e1ot, e2os, e2ot) {
                    return edge_sign(e2ds, e2dt, e1os, e1ot, e2os, e2ot) <= 0.0;
                }
                return edge_sign(e1ds, e1dt, e2os, e2ot, e1os, e1ot) >= 0.0;
            }
            return edge_sign(e2ds, e2dt, ev_s, ev_t, e2os, e2ot) <= 0.0;
        }
        if vert_eq(e2ds, e2dt, ev_s, ev_t) {
            return edge_sign(e1ds, e1dt, ev_s, ev_t, e1os, e1ot) >= 0.0;
        }
        let t1 = crate::geom::edge_eval(e1ds, e1dt, ev_s, ev_t, e1os, e1ot);
        let t2 = crate::geom::edge_eval(e2ds, e2dt, ev_s, ev_t, e2os, e2ot);
        t1 >= t2
    }

    /// Insert a new region below `reg_above` with upper edge `e_new_up`.
    /// Mirrors C's AddRegionBelow + ComputeWinding.
    fn add_region_below(&mut self, _reg_above: RegionIdx, e_new_up: EdgeIdx) -> RegionIdx {
        let reg_new = self.alloc_region();
        {
            let r = self.region_mut(reg_new);
            r.e_up = e_new_up;
            r.fix_upper_edge = false;
            r.sentinel = false;
            r.dirty = false;
        }

        let new_node_idx = self.dict_insert_region(reg_new);
        if new_node_idx == INVALID {
            self.free_region(reg_new);
            return INVALID;
        }
        self.region_mut(reg_new).node_up = new_node_idx;

        // Link the edge to the region.  Defensive invariant check
        // (debug-only): the SYM of the edge we're about to bind must
        // not already be the e_up of another active region.  If it is,
        // we'd end up with both halves of the same edge pair owned by
        // two regions, which the degenerate-2-edge-loop branch in
        // `walk_dirty_regions` then collapses by `delete_edge`-ing the
        // pair from under the OTHER region — exactly the chain we saw
        // surface in wasm as `mesh.verts[INVALID]` in
        // `check_for_right_splice`.
        debug_assert_eq!(
            self.mesh.as_ref().unwrap().edges[(e_new_up ^ 1) as usize].active_region,
            INVALID,
            "add_region_below({}): sym {} already bound to active region {}",
            e_new_up,
            e_new_up ^ 1,
            self.mesh.as_ref().unwrap().edges[(e_new_up ^ 1) as usize].active_region,
        );
        self.mesh.as_mut().unwrap().edges[e_new_up as usize].active_region = reg_new;

        self.compute_winding(reg_new);

        reg_new
    }

    fn delete_region(&mut self, reg: RegionIdx) {
        if self.region(reg).fix_upper_edge {
            // Was created with zero winding - must be deleted with zero winding
        }
        let e_up = self.region(reg).e_up;
        if e_up != INVALID {
            self.mesh.as_mut().unwrap().edges[e_up as usize].active_region = INVALID;
        }
        let node = self.region(reg).node_up;
        self.dict.delete(node);
        self.free_region(reg);
    }

    fn fix_upper_edge(&mut self, reg: RegionIdx, new_edge: EdgeIdx) -> bool {
        let old_edge = self.region(reg).e_up;
        if old_edge != INVALID {
            // Sever the back-pointer from the old half-edge pair to
            // `reg` BEFORE handing it to `delete_edge`.  In libtess2's
            // C original this isn't necessary because the edge's
            // memory is freed and the dangling pointer can never be
            // dereferenced; our `Vec`-backed mesh keeps the slot
            // alive, so a stale `active_region` field would cause the
            // sweep's invariant-validator (`delete_edge`'s
            // `debug_assert!`) to flag a false leak here.
            let mesh = self.mesh.as_mut().unwrap();
            mesh.edges[old_edge as usize].active_region = INVALID;
            mesh.edges[(old_edge ^ 1) as usize].active_region = INVALID;
            if !mesh.delete_edge(old_edge) {
                return false;
            }
        }
        self.region_mut(reg).fix_upper_edge = false;
        self.region_mut(reg).e_up = new_edge;
        self.mesh.as_mut().unwrap().edges[new_edge as usize].active_region = reg;
        true
    }

    fn is_winding_inside(&self, n: i32) -> bool {
        match self.winding_rule {
            WindingRule::Odd => n & 1 != 0,
            WindingRule::NonZero => n != 0,
            WindingRule::Positive => n > 0,
            WindingRule::Negative => n < 0,
            WindingRule::AbsGeqTwo => n >= 2 || n <= -2,
        }
    }

    fn compute_winding(&mut self, reg: RegionIdx) {
        let above = self.region_above(reg);
        let above_winding = if above != INVALID {
            self.region(above).winding_number
        } else {
            0
        };
        let e_up = self.region(reg).e_up;
        let e_winding = if e_up != INVALID {
            self.mesh.as_ref().unwrap().edges[e_up as usize].winding
        } else {
            0
        };
        let new_winding = above_winding + e_winding;
        let inside = self.is_winding_inside(new_winding);
        if self.trace_enabled {
            eprintln!(
                "R   COMPUTE_WINDING winding={} inside={} edge_winding={}",
                new_winding, inside as i32, e_winding
            );
        }
        self.region_mut(reg).winding_number = new_winding;
        self.region_mut(reg).inside = inside;
    }

    fn finish_region(&mut self, reg: RegionIdx) {
        let e = self.region(reg).e_up;
        if e != INVALID {
            let lface = self.mesh.as_ref().unwrap().edges[e as usize].lface;
            if lface != INVALID {
                let inside = self.region(reg).inside;
                if self.trace_enabled {
                    let mesh = self.mesh.as_ref().unwrap();
                    let mut edge_count = 0u32;
                    let an = mesh.faces[lface as usize].an_edge;
                    if an != INVALID {
                        let mut iter = an;
                        loop {
                            edge_count += 1;
                            iter = mesh.edges[iter as usize].lnext;
                            if iter == an || edge_count > 10000 { break; }
                        }
                    }
                    let org = mesh.edges[e as usize].org;
                    let (os, ot) = if org != INVALID {
                        (mesh.verts[org as usize].s, mesh.verts[org as usize].t)
                    } else {
                        (0.0, 0.0)
                    };
                    eprintln!(
                        "R   FINISH_REGION inside={} winding={} face_edges={} eUp_org=({:.2},{:.2})",
                        inside as i32,
                        self.region(reg).winding_number,
                        edge_count,
                        os, ot
                    );
                }
                self.mesh.as_mut().unwrap().faces[lface as usize].inside = inside;
                self.mesh.as_mut().unwrap().faces[lface as usize].an_edge = e;
            }
        }
        self.delete_region(reg);
    }

    /// Find topmost region with same Org as reg->eUp->Org.
    fn top_left_region(&mut self, reg: RegionIdx) -> RegionIdx {
        let org = {
            let e = self.region(reg).e_up;
            if e == INVALID {
                return INVALID;
            }
            self.mesh.as_ref().unwrap().edges[e as usize].org
        };
        let max_region_iters = self.regions.len() + 2;
        let mut r = reg;
        let mut region_iter = 0usize;
        loop {
            r = self.region_above(r);
            if r == INVALID {
                return INVALID;
            }
            let e = self.region(r).e_up;
            if e == INVALID {
                return INVALID;
            }
            let e_org = self.mesh.as_ref().unwrap().edges[e as usize].org;
            if e_org != org {
                break;
            }
            region_iter += 1;
            if region_iter > max_region_iters {
                return INVALID; // degenerate region chain
            }
        }
        // r is now above the topmost region with same origin
        // Check if we need to fix it
        if self.region(r).fix_upper_edge {
            let below = self.region_below(r);
            let below_e = self.region(below).e_up;
            let below_e_sym = below_e ^ 1;
            let r_e = self.region(r).e_up;
            let r_e_lnext = self.mesh.as_ref().unwrap().edges[r_e as usize].lnext;
            let new_e = match self.mesh.as_mut().unwrap().connect(below_e_sym, r_e_lnext) {
                Some(e) => e,
                None => return INVALID,
            };
            if !self.fix_upper_edge(r, new_e) {
                return INVALID;
            }
            r = self.region_above(r);
        }
        r
    }

    fn top_right_region(&self, reg: RegionIdx) -> RegionIdx {
        let dst = {
            let e = self.region(reg).e_up;
            if e == INVALID {
                return INVALID;
            }
            self.mesh.as_ref().unwrap().dst(e)
        };
        let max_region_iters = self.regions.len() + 2;
        let mut r = reg;
        let mut region_iter = 0usize;
        loop {
            r = self.region_above(r);
            if r == INVALID {
                return INVALID;
            }
            let e = self.region(r).e_up;
            if e == INVALID {
                return INVALID;
            }
            let e_dst = self.mesh.as_ref().unwrap().dst(e);
            if e_dst != dst {
                break;
            }
            region_iter += 1;
            if region_iter > max_region_iters {
                return INVALID; // degenerate region chain
            }
        }
        r
    }

    fn finish_left_regions(&mut self, reg_first: RegionIdx, reg_last: RegionIdx) -> EdgeIdx {
        let mut reg_prev = reg_first;
        let mut e_prev = self.region(reg_first).e_up;

        while reg_prev != reg_last {
            self.region_mut(reg_prev).fix_upper_edge = false;
            let reg = self.region_below(reg_prev);
            if reg == INVALID {
                break;
            }
            let e = self.region(reg).e_up;

            let e_org = if e != INVALID {
                self.mesh.as_ref().unwrap().edges[e as usize].org
            } else {
                INVALID
            };
            let ep_org = if e_prev != INVALID {
                self.mesh.as_ref().unwrap().edges[e_prev as usize].org
            } else {
                INVALID
            };

            if e_org != ep_org {
                if !self.region(reg).fix_upper_edge {
                    self.finish_region(reg_prev);
                    break;
                }
                let ep_lprev = if e_prev != INVALID {
                    self.mesh.as_ref().unwrap().lprev(e_prev)
                } else {
                    INVALID
                };
                let e_sym = if e != INVALID { e ^ 1 } else { INVALID };
                let new_e = if ep_lprev != INVALID && e_sym != INVALID {
                    self.mesh.as_mut().unwrap().connect(ep_lprev, e_sym)
                } else {
                    None
                };
                if let Some(ne) = new_e {
                    if !self.fix_upper_edge(reg, ne) {
                        return INVALID;
                    }
                }
            }

            if e_prev != INVALID && e != INVALID {
                let ep_onext = self.mesh.as_ref().unwrap().edges[e_prev as usize].onext;
                if ep_onext != e {
                    let e_oprev = self.mesh.as_ref().unwrap().oprev(e);
                    self.mesh.as_mut().unwrap().splice(e_oprev, e);
                    self.mesh.as_mut().unwrap().splice(e_prev, e);
                }
            }

            self.finish_region(reg_prev);
            e_prev = self.region(reg).e_up;
            reg_prev = reg;
        }
        e_prev
    }

    fn add_right_edges(
        &mut self,
        reg_up: RegionIdx,
        e_first: EdgeIdx,
        e_last: EdgeIdx,
        e_top_left: EdgeIdx,
        clean_up: bool,
    ) {
        // Insert right-going edges into the dictionary.  Guard: the
        // onext ring must contain e_last; if it doesn't (degenerate
        // mesh), break early rather than looping forever.  libtess2's
        // C original asserts `VertLeq(e->Org, e->Dst)` — i.e., `e` is
        // right-going from the event vertex — and our previous Rust
        // port silently accepted any orientation, so a degenerate
        // input could push an edge whose SYM was already an active
        // region's `e_up` into the ring.  `add_region_below` then
        // bound both halves of the same edge pair to two different
        // regions; `walk_dirty_regions`'s degenerate-2-edge-loop
        // branch later `delete_edge`d the pair from under one of
        // them, leaving its e_up dangling and producing the wasm-only
        // `mesh.verts[INVALID]` panic in `check_for_right_splice` /
        // `walk_dirty_regions`.  See `tests/wasm_glyph_repro.rs`.
        let max_edge_iters = self.mesh.as_ref().unwrap().edges.len() + 2;
        let mut e = e_first;
        let mut edge_iter = 0usize;
        loop {
            // Right-going invariant + duplicate-pair guard.  Either
            // condition means the edge isn't a fresh right-going
            // edge of the event vertex and must be skipped.
            let skip = {
                let mesh = self.mesh.as_ref().unwrap();
                let org = mesh.edges[e as usize].org;
                let dst = mesh.dst(e);
                let not_right_going = org != INVALID
                    && dst != INVALID
                    && !vert_leq(
                        mesh.verts[org as usize].s,
                        mesh.verts[org as usize].t,
                        mesh.verts[dst as usize].s,
                        mesh.verts[dst as usize].t,
                    );
                let sym_already_bound =
                    mesh.edges[(e ^ 1) as usize].active_region != INVALID;
                not_right_going || sym_already_bound
            };
            if !skip {
                self.add_region_below(reg_up, e ^ 1);
            }
            e = self.mesh.as_ref().unwrap().edges[e as usize].onext;
            if e == e_last {
                break;
            }
            edge_iter += 1;
            if edge_iter > max_edge_iters {
                break; // degenerate onext ring — skip remaining edges
            }
        }

        // Determine e_top_left
        let e_top_left = if e_top_left == INVALID {
            let reg_below = self.region_below(reg_up);
            if reg_below == INVALID {
                return;
            }
            let rb_e = self.region(reg_below).e_up;
            if rb_e == INVALID {
                return;
            }
            self.mesh.as_ref().unwrap().rprev(rb_e)
        } else {
            e_top_left
        };

        let mut reg_prev = reg_up;
        let mut e_prev = e_top_left;
        let mut first_time = true;
        let max_reg_iters = self.regions.len() + 2;
        let mut reg_iter2 = 0usize;

        loop {
            let reg = self.region_below(reg_prev);
            if reg == INVALID {
                break;
            }
            let e = {
                let re = self.region(reg).e_up;
                if re == INVALID {
                    break;
                }
                re ^ 1 // e = reg->eUp->Sym
            };
            let e_org = self.mesh.as_ref().unwrap().edges[e as usize].org;
            let ep_org = if e_prev != INVALID {
                self.mesh.as_ref().unwrap().edges[e_prev as usize].org
            } else {
                INVALID
            };
            if e_org != ep_org {
                break;
            }
            reg_iter2 += 1;
            if reg_iter2 > max_reg_iters {
                break; // degenerate region chain
            }

            if e_prev != INVALID {
                // C: if( e->Onext != ePrev ) { splice(e->Oprev, e); splice(ePrev->Oprev, e); }
                let e_onext = self.mesh.as_ref().unwrap().edges[e as usize].onext;
                if e_onext != e_prev {
                    let e_oprev = self.mesh.as_ref().unwrap().oprev(e);
                    self.mesh.as_mut().unwrap().splice(e_oprev, e);
                    let ep_oprev = self.mesh.as_ref().unwrap().oprev(e_prev);
                    self.mesh.as_mut().unwrap().splice(ep_oprev, e);
                }
            }

            let above_winding = self.region(reg_prev).winding_number;
            let e_winding = self.mesh.as_ref().unwrap().edges[e as usize].winding;
            let new_winding = above_winding - e_winding;
            let inside = self.is_winding_inside(new_winding);
            self.region_mut(reg).winding_number = new_winding;
            self.region_mut(reg).inside = inside;

            self.region_mut(reg_prev).dirty = true;
            if !first_time {
                if self.check_for_right_splice(reg_prev) {
                    // AddWinding
                    let re = self.region(reg).e_up;
                    let rep = self.region(reg_prev).e_up;
                    if re != INVALID && rep != INVALID {
                        let w1 = self.mesh.as_ref().unwrap().edges[re as usize].winding;
                        let w2 = self.mesh.as_ref().unwrap().edges[(re ^ 1) as usize].winding;
                        let wp1 = self.mesh.as_ref().unwrap().edges[rep as usize].winding;
                        let wp2 = self.mesh.as_ref().unwrap().edges[(rep ^ 1) as usize].winding;
                        self.mesh.as_mut().unwrap().edges[re as usize].winding += wp1;
                        self.mesh.as_mut().unwrap().edges[(re ^ 1) as usize].winding += wp2;
                    }
                    self.delete_region(reg_prev);
                    if e_prev != INVALID {
                        self.mesh.as_mut().unwrap().delete_edge(e_prev);
                    }
                }
            }
            first_time = false;
            reg_prev = reg;
            e_prev = e;
        }

        self.region_mut(reg_prev).dirty = true;

        if clean_up {
            self.walk_dirty_regions(reg_prev);
        }
    }

    fn check_for_right_splice(&mut self, reg_up: RegionIdx) -> bool {
        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return false;
        }
        let e_up = self.region(reg_up).e_up;
        let e_lo = self.region(reg_lo).e_up;
        if e_up == INVALID || e_lo == INVALID {
            return false;
        }

        let mesh = self.mesh.as_ref().unwrap();
        let e_up_org = mesh.edges[e_up as usize].org;
        let e_lo_org = mesh.edges[e_lo as usize].org;
        let (euo_s, euo_t) = (
            mesh.verts[e_up_org as usize].s,
            mesh.verts[e_up_org as usize].t,
        );
        let (elo_s, elo_t) = (
            mesh.verts[e_lo_org as usize].s,
            mesh.verts[e_lo_org as usize].t,
        );
        let e_lo_dst = mesh.dst(e_lo);
        let (eld_s, eld_t) = (
            mesh.verts[e_lo_dst as usize].s,
            mesh.verts[e_lo_dst as usize].t,
        );
        let e_up_dst = mesh.dst(e_up);
        let (eud_s, eud_t) = (
            mesh.verts[e_up_dst as usize].s,
            mesh.verts[e_up_dst as usize].t,
        );
        drop(mesh);

        if vert_leq(euo_s, euo_t, elo_s, elo_t) {
            if edge_sign(eld_s, eld_t, euo_s, euo_t, elo_s, elo_t) > 0.0 {
                return false;
            }
            if !vert_eq(euo_s, euo_t, elo_s, elo_t) {
                // Splice eUp->Org into eLo
                self.mesh.as_mut().unwrap().split_edge(e_lo ^ 1);
                let e_lo_oprev = self.mesh.as_ref().unwrap().oprev(e_lo);
                self.mesh.as_mut().unwrap().splice(e_up, e_lo_oprev);
                self.region_mut(reg_up).dirty = true;
                self.region_mut(reg_lo).dirty = true;
            } else if e_up_org != e_lo_org {
                // Merge: delete eUp->Org from PQ and splice
                let handle = self.mesh.as_ref().unwrap().verts[e_up_org as usize].pq_handle;
                self.pq_delete(handle);
                let e_lo_oprev = self.mesh.as_ref().unwrap().oprev(e_lo);
                self.mesh.as_mut().unwrap().splice(e_lo_oprev, e_up);
            }
        } else {
            if edge_sign(eud_s, eud_t, elo_s, elo_t, euo_s, euo_t) < 0.0 {
                return false;
            }
            let reg_above = self.region_above(reg_up);
            if reg_above != INVALID {
                self.region_mut(reg_above).dirty = true;
            }
            self.region_mut(reg_up).dirty = true;
            self.mesh.as_mut().unwrap().split_edge(e_up ^ 1);
            let e_lo_oprev = self.mesh.as_ref().unwrap().oprev(e_lo);
            self.mesh.as_mut().unwrap().splice(e_lo_oprev, e_up);
        }
        true
    }

    fn check_for_left_splice(&mut self, reg_up: RegionIdx) -> bool {
        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return false;
        }
        let e_up = self.region(reg_up).e_up;
        let e_lo = self.region(reg_lo).e_up;
        if e_up == INVALID || e_lo == INVALID {
            return false;
        }

        let mesh = self.mesh.as_ref().unwrap();
        let e_up_dst = mesh.dst(e_up);
        let e_lo_dst = mesh.dst(e_lo);
        if vert_eq(
            mesh.verts[e_up_dst as usize].s,
            mesh.verts[e_up_dst as usize].t,
            mesh.verts[e_lo_dst as usize].s,
            mesh.verts[e_lo_dst as usize].t,
        ) {
            return false;
        } // Same destination

        let (eud_s, eud_t) = (
            mesh.verts[e_up_dst as usize].s,
            mesh.verts[e_up_dst as usize].t,
        );
        let (eld_s, eld_t) = (
            mesh.verts[e_lo_dst as usize].s,
            mesh.verts[e_lo_dst as usize].t,
        );
        let e_up_org = mesh.edges[e_up as usize].org;
        let e_lo_org = mesh.edges[e_lo as usize].org;
        let (euo_s, euo_t) = (
            mesh.verts[e_up_org as usize].s,
            mesh.verts[e_up_org as usize].t,
        );
        let (elo_s, elo_t) = (
            mesh.verts[e_lo_org as usize].s,
            mesh.verts[e_lo_org as usize].t,
        );
        drop(mesh);

        if vert_leq(eud_s, eud_t, eld_s, eld_t) {
            if edge_sign(eud_s, eud_t, eld_s, eld_t, euo_s, euo_t) < 0.0 {
                return false;
            }
            // eLo->Dst is above eUp: splice eLo->Dst into eUp
            let reg_above = self.region_above(reg_up);
            if reg_above != INVALID {
                self.region_mut(reg_above).dirty = true;
            }
            self.region_mut(reg_up).dirty = true;
            let new_e = match self.mesh.as_mut().unwrap().split_edge(e_up) {
                Some(e) => e,
                None => return false,
            };
            let e_lo_sym = e_lo ^ 1;
            self.mesh.as_mut().unwrap().splice(e_lo_sym, new_e);
            let new_lface = self.mesh.as_ref().unwrap().edges[new_e as usize].lface;
            let inside = self.region(reg_up).inside;
            if new_lface != INVALID {
                self.mesh.as_mut().unwrap().faces[new_lface as usize].inside = inside;
            }
        } else {
            if edge_sign(eld_s, eld_t, eud_s, eud_t, elo_s, elo_t) > 0.0 {
                return false;
            }
            // eUp->Dst is below eLo: splice eUp->Dst into eLo
            self.region_mut(reg_up).dirty = true;
            self.region_mut(reg_lo).dirty = true;
            let new_e = match self.mesh.as_mut().unwrap().split_edge(e_lo) {
                Some(e) => e,
                None => return false,
            };
            let e_up_lnext = self.mesh.as_ref().unwrap().edges[e_up as usize].lnext;
            let e_lo_sym = e_lo ^ 1;
            self.mesh.as_mut().unwrap().splice(e_up_lnext, e_lo_sym);
            let new_rface = self.mesh.as_ref().unwrap().rface(new_e);
            let inside = self.region(reg_up).inside;
            if new_rface != INVALID {
                self.mesh.as_mut().unwrap().faces[new_rface as usize].inside = inside;
            }
        }
        true
    }

    fn check_for_intersect(&mut self, reg_up: RegionIdx) -> bool {
        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return false;
        }
        let e_up = self.region(reg_up).e_up;
        let e_lo = self.region(reg_lo).e_up;
        if e_up == INVALID || e_lo == INVALID {
            return false;
        }
        if self.region(reg_up).fix_upper_edge || self.region(reg_lo).fix_upper_edge {
            return false;
        }

        let mesh = self.mesh.as_ref().unwrap();
        let org_up = mesh.edges[e_up as usize].org;
        let org_lo = mesh.edges[e_lo as usize].org;
        let dst_up = mesh.dst(e_up);
        let dst_lo = mesh.dst(e_lo);

        if vert_eq(
            mesh.verts[dst_up as usize].s,
            mesh.verts[dst_up as usize].t,
            mesh.verts[dst_lo as usize].s,
            mesh.verts[dst_lo as usize].t,
        ) {
            return false;
        }

        let (ou_s, ou_t) = (mesh.verts[org_up as usize].s, mesh.verts[org_up as usize].t);
        let (ol_s, ol_t) = (mesh.verts[org_lo as usize].s, mesh.verts[org_lo as usize].t);
        let (du_s, du_t) = (mesh.verts[dst_up as usize].s, mesh.verts[dst_up as usize].t);
        let (dl_s, dl_t) = (mesh.verts[dst_lo as usize].s, mesh.verts[dst_lo as usize].t);
        // Save coords of all 4 endpoints before the mesh is mutated by split_edge.
        let ou_coords = mesh.verts[org_up as usize].coords;
        let du_coords = mesh.verts[dst_up as usize].coords;
        let ol_coords = mesh.verts[org_lo as usize].coords;
        let dl_coords = mesh.verts[dst_lo as usize].coords;
        let ev_s = self.event_s;
        let ev_t = self.event_t;
        drop(mesh);

        // Quick rejection tests
        let t_min_up = ou_t.min(du_t);
        let t_max_lo = ol_t.max(dl_t);
        if t_min_up > t_max_lo {
            return false;
        }

        if vert_leq(ou_s, ou_t, ol_s, ol_t) {
            if edge_sign(dl_s, dl_t, ou_s, ou_t, ol_s, ol_t) > 0.0 {
                return false;
            }
        } else {
            if edge_sign(du_s, du_t, ol_s, ol_t, ou_s, ou_t) < 0.0 {
                return false;
            }
        }

        // Compute intersection
        let (isect_s, isect_t) = edge_intersect(du_s, du_t, ou_s, ou_t, dl_s, dl_t, ol_s, ol_t);

        // Clamp intersection to sweep event position
        let (isect_s, isect_t) = if vert_leq(isect_s, isect_t, ev_s, ev_t) {
            (ev_s, ev_t)
        } else {
            (isect_s, isect_t)
        };

        // Clamp to rightmost origin
        let (org_min_s, org_min_t) = if vert_leq(ou_s, ou_t, ol_s, ol_t) {
            (ou_s, ou_t)
        } else {
            (ol_s, ol_t)
        };
        let (isect_s, isect_t) = if vert_leq(org_min_s, org_min_t, isect_s, isect_t) {
            (org_min_s, org_min_t)
        } else {
            (isect_s, isect_t)
        };

        // Check if intersection is at one of the endpoints
        if vert_eq(isect_s, isect_t, ou_s, ou_t) || vert_eq(isect_s, isect_t, ol_s, ol_t) {
            self.check_for_right_splice(reg_up);
            return false;
        }

        if (!vert_eq(du_s, du_t, ev_s, ev_t)
            && edge_sign(du_s, du_t, ev_s, ev_t, isect_s, isect_t) >= 0.0)
            || (!vert_eq(dl_s, dl_t, ev_s, ev_t)
                && edge_sign(dl_s, dl_t, ev_s, ev_t, isect_s, isect_t) <= 0.0)
        {
            if vert_eq(dl_s, dl_t, ev_s, ev_t) {
                // Splice dstLo into eUp
                self.mesh.as_mut().unwrap().split_edge(e_up ^ 1);
                let e_lo_sym = e_lo ^ 1;
                let e_up2 = self.region(reg_up).e_up;
                self.mesh.as_mut().unwrap().splice(e_lo_sym, e_up2);
                let reg_up2 = self.top_left_region(reg_up);
                if reg_up2 == INVALID {
                    return false;
                }
                let rb = self.region_below(reg_up2);
                let rb_e = self.region(rb).e_up;
                let rl2 = self.region_below(rb);
                self.finish_left_regions(self.region_below(reg_up2), reg_lo);
                let e_up_new = self.region(rb).e_up;
                let e_oprev = self.mesh.as_ref().unwrap().oprev(e_up_new);
                self.add_right_edges(reg_up2, e_oprev, e_up_new, e_up_new, true);
                return true;
            }
            if vert_eq(du_s, du_t, ev_s, ev_t) {
                self.mesh.as_mut().unwrap().split_edge(e_lo ^ 1);
                let e_up_lnext = self.mesh.as_ref().unwrap().edges[e_up as usize].lnext;
                let e_lo_oprev = self.mesh.as_ref().unwrap().oprev(e_lo);
                self.mesh.as_mut().unwrap().splice(e_up_lnext, e_lo_oprev);
                let reg_lo2 = reg_up;
                let reg_up2 = self.top_right_region(reg_up);
                if reg_up2 == INVALID {
                    return false;
                }
                let e_finish = self
                    .mesh
                    .as_ref()
                    .unwrap()
                    .rprev(self.region(self.region_below(reg_up2)).e_up);
                self.region_mut(reg_lo2).e_up = self.mesh.as_ref().unwrap().oprev(e_lo);
                let lo_end = self.finish_left_regions(reg_lo2, INVALID);
                let e_lo_onext = if lo_end != INVALID {
                    self.mesh.as_ref().unwrap().edges[lo_end as usize].onext
                } else {
                    INVALID
                };
                let e_up_rprev = self.mesh.as_ref().unwrap().rprev(e_up);
                self.add_right_edges(reg_up2, e_lo_onext, e_up_rprev, e_finish, true);
                return true;
            }
            // Split edges
            if edge_sign(du_s, du_t, ev_s, ev_t, isect_s, isect_t) >= 0.0 {
                let reg_above = self.region_above(reg_up);
                if reg_above != INVALID {
                    self.region_mut(reg_above).dirty = true;
                }
                self.region_mut(reg_up).dirty = true;
                self.mesh.as_mut().unwrap().split_edge(e_up ^ 1);
                let e_up2 = self.region(reg_up).e_up;
                let e_up2_org = self.mesh.as_ref().unwrap().edges[e_up2 as usize].org;
                self.mesh.as_mut().unwrap().verts[e_up2_org as usize].s = ev_s;
                self.mesh.as_mut().unwrap().verts[e_up2_org as usize].t = ev_t;
            }
            if edge_sign(dl_s, dl_t, ev_s, ev_t, isect_s, isect_t) <= 0.0 {
                self.region_mut(reg_up).dirty = true;
                self.region_mut(reg_lo).dirty = true;
                self.mesh.as_mut().unwrap().split_edge(e_lo ^ 1);
                let e_lo2 = self.region(reg_lo).e_up;
                let e_lo2_org = self.mesh.as_ref().unwrap().edges[e_lo2 as usize].org;
                self.mesh.as_mut().unwrap().verts[e_lo2_org as usize].s = ev_s;
                self.mesh.as_mut().unwrap().verts[e_lo2_org as usize].t = ev_t;
            }
            return false;
        }

        // General case: split both edges and splice at intersection
        self.mesh.as_mut().unwrap().split_edge(e_up ^ 1);
        self.mesh.as_mut().unwrap().split_edge(e_lo ^ 1);
        let e_lo2 = self.region(reg_lo).e_up;
        let e_lo2_oprev = self.mesh.as_ref().unwrap().oprev(e_lo2);
        let e_up2 = self.region(reg_up).e_up;
        self.mesh.as_mut().unwrap().splice(e_lo2_oprev, e_up2);

        // Set intersection coordinates
        let e_up2_org = self.mesh.as_ref().unwrap().edges[e_up2 as usize].org;

        // Compute weighted coordinates for the intersection vertex
        let (org_up_s, org_up_t) = (ou_s, ou_t);
        let (dst_up_s, dst_up_t) = (du_s, du_t);
        let (org_lo_s, org_lo_t) = (ol_s, ol_t);
        let (dst_lo_s, dst_lo_t) = (dl_s, dl_t);

        self.mesh.as_mut().unwrap().verts[e_up2_org as usize].s = isect_s;
        self.mesh.as_mut().unwrap().verts[e_up2_org as usize].t = isect_t;
        self.mesh.as_mut().unwrap().verts[e_up2_org as usize].coords = compute_intersect_coords(
            isect_s, isect_t, org_up_s, org_up_t, ou_coords, dst_up_s, dst_up_t, du_coords,
            org_lo_s, org_lo_t, ol_coords, dst_lo_s, dst_lo_t, dl_coords,
        );
        self.mesh.as_mut().unwrap().verts[e_up2_org as usize].idx = TESS_UNDEF;

        // Insert new vertex into priority queue
        let handle = self.pq_insert(e_up2_org);
        if handle == INVALID_HANDLE {
            return false;
        }
        self.mesh.as_mut().unwrap().verts[e_up2_org as usize].pq_handle = handle;

        let reg_above = self.region_above(reg_up);
        if reg_above != INVALID {
            self.region_mut(reg_above).dirty = true;
        }
        self.region_mut(reg_up).dirty = true;
        self.region_mut(reg_lo).dirty = true;

        false
    }

    fn walk_dirty_regions(&mut self, reg_up: RegionIdx) {
        let mut reg_up = reg_up;
        let mut reg_lo = self.region_below(reg_up);

        let max_dirty_iters = self.regions.len() * 4 + 100;
        let mut dirty_iter = 0usize;
        loop {
            dirty_iter += 1;
            if dirty_iter > max_dirty_iters {
                return; // guard against oscillating dirty-flag loops
            }
            // Find lowest dirty region
            while reg_lo != INVALID && self.region(reg_lo).dirty {
                reg_up = reg_lo;
                reg_lo = self.region_below(reg_lo);
            }
            if !self.region(reg_up).dirty {
                reg_lo = reg_up;
                reg_up = self.region_above(reg_up);
                if reg_up == INVALID || !self.region(reg_up).dirty {
                    return;
                }
            }

            self.region_mut(reg_up).dirty = false;
            if reg_lo == INVALID {
                return;
            }
            let e_up = self.region(reg_up).e_up;
            let e_lo = self.region(reg_lo).e_up;

            if e_up != INVALID && e_lo != INVALID {
                let e_up_dst = self.mesh.as_ref().unwrap().dst(e_up);
                let e_lo_dst = self.mesh.as_ref().unwrap().dst(e_lo);
                let (eud_s, eud_t) = (
                    self.mesh.as_ref().unwrap().verts[e_up_dst as usize].s,
                    self.mesh.as_ref().unwrap().verts[e_up_dst as usize].t,
                );
                let (eld_s, eld_t) = (
                    self.mesh.as_ref().unwrap().verts[e_lo_dst as usize].s,
                    self.mesh.as_ref().unwrap().verts[e_lo_dst as usize].t,
                );

                if !vert_eq(eud_s, eud_t, eld_s, eld_t) {
                    if self.check_for_left_splice(reg_up) {
                        let reg_lo_fix = self.region(reg_lo).fix_upper_edge;
                        let reg_up_fix = self.region(reg_up).fix_upper_edge;
                        if reg_lo_fix {
                            let e_lo2 = self.region(reg_lo).e_up;
                            self.delete_region(reg_lo);
                            if e_lo2 != INVALID {
                                self.mesh.as_mut().unwrap().delete_edge(e_lo2);
                            }
                            reg_lo = self.region_below(reg_up);
                        } else if reg_up_fix {
                            let e_up2 = self.region(reg_up).e_up;
                            self.delete_region(reg_up);
                            if e_up2 != INVALID {
                                self.mesh.as_mut().unwrap().delete_edge(e_up2);
                            }
                            reg_up = self.region_above(reg_lo);
                        }
                    }
                }

                let e_up2 = self.region(reg_up).e_up;
                let e_lo2 = self.region(reg_lo).e_up;
                if e_up2 != INVALID && e_lo2 != INVALID {
                    let e_up_org = self.mesh.as_ref().unwrap().edges[e_up2 as usize].org;
                    let e_lo_org = self.mesh.as_ref().unwrap().edges[e_lo2 as usize].org;
                    if e_up_org != e_lo_org {
                        let e_up_dst2 = self.mesh.as_ref().unwrap().dst(e_up2);
                        let e_lo_dst2 = self.mesh.as_ref().unwrap().dst(e_lo2);
                        let fix_up = self.region(reg_up).fix_upper_edge;
                        let fix_lo = self.region(reg_lo).fix_upper_edge;
                        if !vert_eq(
                            self.mesh.as_ref().unwrap().verts[e_up_dst2 as usize].s,
                            self.mesh.as_ref().unwrap().verts[e_up_dst2 as usize].t,
                            self.mesh.as_ref().unwrap().verts[e_lo_dst2 as usize].s,
                            self.mesh.as_ref().unwrap().verts[e_lo_dst2 as usize].t,
                        ) && !fix_up
                            && !fix_lo
                            && (vert_eq(
                                self.mesh.as_ref().unwrap().verts[e_up_dst2 as usize].s,
                                self.mesh.as_ref().unwrap().verts[e_up_dst2 as usize].t,
                                self.event_s,
                                self.event_t,
                            ) || vert_eq(
                                self.mesh.as_ref().unwrap().verts[e_lo_dst2 as usize].s,
                                self.mesh.as_ref().unwrap().verts[e_lo_dst2 as usize].t,
                                self.event_s,
                                self.event_t,
                            ))
                        {
                            if self.check_for_intersect(reg_up) {
                                return;
                            }
                        } else {
                            self.check_for_right_splice(reg_up);
                        }
                    }
                }

                // Check for degenerate 2-edge loop
                let e_up3 = self.region(reg_up).e_up;
                let e_lo3 = self.region(reg_lo).e_up;
                if e_up3 != INVALID && e_lo3 != INVALID {
                    let e_up_org3 = self.mesh.as_ref().unwrap().edges[e_up3 as usize].org;
                    let e_lo_org3 = self.mesh.as_ref().unwrap().edges[e_lo3 as usize].org;
                    let e_up_dst3 = self.mesh.as_ref().unwrap().dst(e_up3);
                    let e_lo_dst3 = self.mesh.as_ref().unwrap().dst(e_lo3);
                    if e_up_org3 == e_lo_org3 && e_up_dst3 == e_lo_dst3 {
                        // Merge winding and delete one region
                        let eu_w = self.mesh.as_ref().unwrap().edges[e_up3 as usize].winding;
                        let eu_sw = self.mesh.as_ref().unwrap().edges[(e_up3 ^ 1) as usize].winding;
                        self.mesh.as_mut().unwrap().edges[e_lo3 as usize].winding += eu_w;
                        self.mesh.as_mut().unwrap().edges[(e_lo3 ^ 1) as usize].winding += eu_sw;
                        self.delete_region(reg_up);
                        self.mesh.as_mut().unwrap().delete_edge(e_up3);
                        reg_up = self.region_above(reg_lo);
                    }
                }
            }
        }
    }

    fn connect_right_vertex(&mut self, reg_up: RegionIdx, e_bottom_left: EdgeIdx) {
        // Mirrors C ConnectRightVertex exactly.
        // eTopLeft = eBottomLeft->Onext
        let e_top_left = self.mesh.as_ref().unwrap().edges[e_bottom_left as usize].onext;

        // Step 1: if eUp->Dst != eLo->Dst, check for intersection
        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return;
        }
        let e_up = self.region(reg_up).e_up;
        let e_lo = self.region(reg_lo).e_up;
        if e_up == INVALID || e_lo == INVALID {
            return;
        }

        let dst_differ = {
            let e_up_dst = self.mesh.as_ref().unwrap().dst(e_up);
            let e_lo_dst = self.mesh.as_ref().unwrap().dst(e_lo);
            let (s1, t1) = (
                self.mesh.as_ref().unwrap().verts[e_up_dst as usize].s,
                self.mesh.as_ref().unwrap().verts[e_up_dst as usize].t,
            );
            let (s2, t2) = (
                self.mesh.as_ref().unwrap().verts[e_lo_dst as usize].s,
                self.mesh.as_ref().unwrap().verts[e_lo_dst as usize].t,
            );
            !vert_eq(s1, t1, s2, t2)
        };
        if dst_differ {
            if self.check_for_intersect(reg_up) {
                return;
            }
        }

        // Step 2: re-read after possible changes from CheckForIntersect
        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return;
        }
        let e_up = self.region(reg_up).e_up;
        let e_lo = self.region(reg_lo).e_up;
        if e_up == INVALID || e_lo == INVALID {
            return;
        }

        // Step 3: degenerate cases
        let mut degenerate = false;
        let mut reg_up = reg_up;
        let mut e_top_left = e_top_left;
        let mut e_bottom_left = e_bottom_left;

        // if(VertEq(eUp->Org, event))
        let e_up_org = self.mesh.as_ref().unwrap().edges[e_up as usize].org;
        if e_up_org != INVALID {
            let (s, t) = (
                self.mesh.as_ref().unwrap().verts[e_up_org as usize].s,
                self.mesh.as_ref().unwrap().verts[e_up_org as usize].t,
            );
            if vert_eq(s, t, self.event_s, self.event_t) {
                // splice(eTopLeft->Oprev, eUp)
                let e_tl_oprev = self.mesh.as_ref().unwrap().oprev(e_top_left);
                self.mesh.as_mut().unwrap().splice(e_tl_oprev, e_up);
                // regUp = TopLeftRegion(regUp)
                let reg_up2 = self.top_left_region(reg_up);
                if reg_up2 == INVALID {
                    return;
                }
                // eTopLeft = RegionBelow(regUp)->eUp
                let rb = self.region_below(reg_up2);
                e_top_left = if rb != INVALID {
                    self.region(rb).e_up
                } else {
                    INVALID
                };
                // FinishLeftRegions(RegionBelow(regUp), regLo)
                self.finish_left_regions(rb, reg_lo);
                reg_up = reg_up2;
                degenerate = true;
            }
        }

        // if(VertEq(eLo->Org, event))
        let e_lo2 = if degenerate {
            let rl = self.region_below(reg_up);
            if rl != INVALID {
                self.region(rl).e_up
            } else {
                INVALID
            }
        } else {
            e_lo
        };
        let reg_lo2 = self.region_below(reg_up);

        let e_lo_org = if e_lo2 != INVALID {
            self.mesh.as_ref().unwrap().edges[e_lo2 as usize].org
        } else {
            INVALID
        };
        if e_lo_org != INVALID {
            let (s, t) = (
                self.mesh.as_ref().unwrap().verts[e_lo_org as usize].s,
                self.mesh.as_ref().unwrap().verts[e_lo_org as usize].t,
            );
            if vert_eq(s, t, self.event_s, self.event_t) {
                // splice(eBottomLeft, eLo->Oprev)
                let e_lo_oprev = self.mesh.as_ref().unwrap().oprev(e_lo2);
                self.mesh
                    .as_mut()
                    .unwrap()
                    .splice(e_bottom_left, e_lo_oprev);
                // eBottomLeft = FinishLeftRegions(regLo, NULL)
                e_bottom_left = self.finish_left_regions(reg_lo2, INVALID);
                degenerate = true;
            }
        }

        if degenerate {
            if e_bottom_left != INVALID && e_top_left != INVALID {
                let e_bl_onext = self.mesh.as_ref().unwrap().edges[e_bottom_left as usize].onext;
                self.add_right_edges(reg_up, e_bl_onext, e_top_left, e_top_left, true);
            }
            return;
        }

        // Step 4: non-degenerate — add temporary fixable edge
        let e_up2 = self.region(reg_up).e_up;
        let rl = self.region_below(reg_up);
        if rl == INVALID {
            return;
        }
        let e_lo3 = self.region(rl).e_up;
        if e_up2 == INVALID || e_lo3 == INVALID {
            return;
        }

        let e_up2_org = self.mesh.as_ref().unwrap().edges[e_up2 as usize].org;
        let e_lo3_org = self.mesh.as_ref().unwrap().edges[e_lo3 as usize].org;
        let e_new_target = if e_up2_org != INVALID && e_lo3_org != INVALID {
            let (euo_s, euo_t) = (
                self.mesh.as_ref().unwrap().verts[e_up2_org as usize].s,
                self.mesh.as_ref().unwrap().verts[e_up2_org as usize].t,
            );
            let (elo_s, elot) = (
                self.mesh.as_ref().unwrap().verts[e_lo3_org as usize].s,
                self.mesh.as_ref().unwrap().verts[e_lo3_org as usize].t,
            );
            // eNew = VertLeq(eLo->Org, eUp->Org) ? eLo->Oprev : eUp
            if vert_leq(elo_s, elot, euo_s, euo_t) {
                self.mesh.as_ref().unwrap().oprev(e_lo3)
            } else {
                e_up2
            }
        } else {
            e_up2
        };

        // eNew = connect(eBottomLeft->Lprev, eNewTarget)
        let e_bl_lprev = self.mesh.as_ref().unwrap().lprev(e_bottom_left);
        let e_new = match self
            .mesh
            .as_mut()
            .unwrap()
            .connect(e_bl_lprev, e_new_target)
        {
            Some(e) => e,
            None => return,
        };

        // AddRightEdges(regUp, eNew, eNew->Onext, eNew->Onext, FALSE)
        let e_new_onext = self.mesh.as_ref().unwrap().edges[e_new as usize].onext;
        self.add_right_edges(reg_up, e_new, e_new_onext, e_new_onext, false);

        // eNew->Sym->activeRegion->fixUpperEdge = TRUE
        let e_new_sym_ar = self.mesh.as_ref().unwrap().edges[(e_new ^ 1) as usize].active_region;
        if e_new_sym_ar != INVALID {
            self.region_mut(e_new_sym_ar).fix_upper_edge = true;
        }
        self.walk_dirty_regions(reg_up);
    }

    /// Mirrors agg-sharp's `ConnectLeftDegenerate` exactly.  Called when the
    /// current sweep event lies on (or coincident with) an already-processed
    /// edge — we have to splice the event into that edge rather than adding
    /// it as a fresh isolated vertex.  Three sub-cases:
    ///
    ///   1. `event == e.Org` — the edge's origin was produced by an earlier
    ///      intersection split and is still in the PQ.  SpliceMergeVertices
    ///      collapses them into a single vertex.
    ///   2. `event` lies strictly on `e` (between Org and Dst) — split the
    ///      edge at `event`, splice, then recurse into `sweep_event` so the
    ///      new vertex is handled properly.
    ///   3. `event == e.Dst` — the event coincides with an already-processed
    ///      destination vertex.  Splice the event's right-going edges into
    ///      `eTopRight` so they join the mesh at the right place.
    ///
    /// The previous Rust port handled only case 1 and fell back to
    /// `check_for_right_splice` for cases 2 and 3, which produced the
    /// "eye" rendering artefacts on the lion's self-intersecting polygons
    /// and occasionally a panic during rotation when the mesh was left in
    /// an inconsistent state.
    fn connect_left_degenerate(&mut self, reg_up: RegionIdx, v_event: VertIdx) {
        let e_up = self.region(reg_up).e_up;
        if e_up == INVALID {
            return;
        }
        let e_up_org = self.mesh.as_ref().unwrap().edges[e_up as usize].org;
        let (euo_s, euo_t) = (
            self.mesh.as_ref().unwrap().verts[e_up_org as usize].s,
            self.mesh.as_ref().unwrap().verts[e_up_org as usize].t,
        );
        let e_up_dst = self.mesh.as_ref().unwrap().dst(e_up);
        let (eud_s, eud_t) = (
            self.mesh.as_ref().unwrap().verts[e_up_dst as usize].s,
            self.mesh.as_ref().unwrap().verts[e_up_dst as usize].t,
        );
        let (ev_s, ev_t) = (self.event_s, self.event_t);

        // Case 1: e.Org == event — unprocessed vertex, merge and let the
        // event come out of the PQ later.
        if vert_eq(euo_s, euo_t, ev_s, ev_t) {
            let v_an = self.mesh.as_ref().unwrap().verts[v_event as usize].an_edge;
            if v_an != INVALID {
                self.splice_merge_vertices(e_up, v_an);
            }
            return;
        }

        // Case 2: event lies strictly on e (not at either endpoint) —
        // split the edge at the event, splice in v_event's edges, recurse.
        if !vert_eq(eud_s, eud_t, ev_s, ev_t) {
            if self.mesh.as_mut().unwrap().split_edge(e_up ^ 1).is_none() {
                return;
            }
            // If the region had a `fix_upper_edge` flag (temporary sweep
            // edge), delete the unused portion.
            if self.region(reg_up).fix_upper_edge {
                let nxt = self.mesh.as_ref().unwrap().edges[e_up as usize].onext;
                if nxt != INVALID {
                    let _ = self.mesh.as_mut().unwrap().delete_edge(nxt);
                }
                self.region_mut(reg_up).fix_upper_edge = false;
            }
            let v_an = self.mesh.as_ref().unwrap().verts[v_event as usize].an_edge;
            if v_an != INVALID {
                self.mesh.as_mut().unwrap().splice(v_an, e_up);
            }
            // Re-process v_event now that the mesh has a new vertex at the
            // event position.  Matches C# `SweepEvent(tess, vEvent);` recurse.
            self.sweep_event(v_event);
            return;
        }

        // Case 3: event == e.Dst — an already-processed destination
        // vertex.  Walk up to the top-right region of reg_up and splice
        // the event's right-going edges into the appropriate Onext ring.
        let reg_up2 = self.top_right_region(reg_up);
        if reg_up2 == INVALID {
            // Fallback: just splice at the current region — better than
            // leaving the event unattached.
            self.check_for_right_splice(reg_up);
            return;
        }
        let reg = self.region_below(reg_up2);
        if reg == INVALID {
            self.check_for_right_splice(reg_up);
            return;
        }
        let reg_e_up = self.region(reg).e_up;
        if reg_e_up == INVALID {
            self.check_for_right_splice(reg_up);
            return;
        }
        let mut e_top_right = reg_e_up ^ 1;
        let mut e_top_left  = self.mesh.as_ref().unwrap().edges[e_top_right as usize].onext;
        let mut e_last      = e_top_left;
        // Temp fixable-edge cleanup — matches C#.
        if self.region(reg).fix_upper_edge {
            if e_top_left != e_top_right {
                self.delete_region(reg);
                let _ = self.mesh.as_mut().unwrap().delete_edge(e_top_right);
                e_top_right = self.mesh.as_ref().unwrap().oprev(e_top_left);
            }
        }
        let v_an = self.mesh.as_ref().unwrap().verts[v_event as usize].an_edge;
        if v_an != INVALID {
            self.mesh.as_mut().unwrap().splice(v_an, e_top_right);
        }
        // C# signals "no left-going edges" by passing null for eTopLeft.
        // Our `add_right_edges` treats INVALID the same way.
        if !self.mesh.as_ref().unwrap().edge_goes_left(e_top_left) {
            e_top_left = INVALID;
        }
        let _ = e_last;
        let e_first = self.mesh.as_ref().unwrap().edges[e_top_right as usize].onext;
        self.add_right_edges(reg_up2, e_first, e_last, e_top_left, true);
    }

    /// Port of agg-sharp's `SpliceMergeVertices` — two vertices that the
    /// sweep has decided are "the same" get their Onext rings spliced
    /// together so the mesh sees a single vertex.  We skip the user-
    /// callback combine (no client vertex merging) and just call
    /// `meshSplice`, matching the no-callback semantics of the C reference.
    fn splice_merge_vertices(&mut self, e1: EdgeIdx, e2: EdgeIdx) {
        if e1 == INVALID || e2 == INVALID { return; }
        // Delete one of the two originals from the PQ if it's still queued
        // — matches `VertexPriorityQue.Delete(eUp.originVertex.priorityQueueHandle)`
        // from the C# call site in `CheckForRightSplice`.  Safe to skip
        // if not queued.
        let v2_org = self.mesh.as_ref().unwrap().edges[e2 as usize].org;
        if v2_org != INVALID {
            let handle = self.mesh.as_ref().unwrap().verts[v2_org as usize].pq_handle;
            if handle != INVALID_HANDLE {
                self.pq_delete(handle);
            }
        }
        self.mesh.as_mut().unwrap().splice(e1, e2);
    }

    /// Mirrors C's dictSearch: walks forward from head.next, returns the key of
    /// the FIRST node where edge_leq(tmp_reg, node.key) is true.
    /// This is exactly how the C code finds the containing region in ConnectLeftVertex.
    fn dict_search_forward(&mut self, tmp_e_up: EdgeIdx) -> RegionIdx {
        let tmp_reg = self.alloc_region();
        self.region_mut(tmp_reg).e_up = tmp_e_up;

        // C dictSearch: walk forward from head.next until key==NULL or edge_leq(tmp, node.key)
        let max_fwd_iters = self.dict.nodes.len() + 2;
        let mut fwd_iter = 0usize;
        let mut node = self.dict.nodes[DICT_HEAD as usize].next;
        let result = loop {
            let key = self.dict.key(node);
            if key == INVALID {
                // hit head sentinel — not found
                break INVALID;
            }
            if self.edge_leq(tmp_reg, key) {
                break key;
            }
            node = self.dict.succ(node);
            fwd_iter += 1;
            if fwd_iter > max_fwd_iters {
                break INVALID; // degenerate dict — stop walking
            }
        };

        self.free_region(tmp_reg);
        result
    }

    fn connect_left_vertex(&mut self, v_event: VertIdx) {
        let an_edge = self.mesh.as_ref().unwrap().verts[v_event as usize].an_edge;
        if an_edge == INVALID {
            return;
        }

        let tmp_e_up = an_edge ^ 1;
        let reg_up = self.dict_search_forward(tmp_e_up);
        if reg_up == INVALID {
            return;
        }

        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return;
        }

        let e_up = self.region(reg_up).e_up;
        let e_lo = self.region(reg_lo).e_up;
        if e_up == INVALID || e_lo == INVALID {
            return;
        }

        let e_up_dst = self.mesh.as_ref().unwrap().dst(e_up);
        let e_up_org = self.mesh.as_ref().unwrap().edges[e_up as usize].org;
        if e_up_dst == INVALID || e_up_org == INVALID {
            return;
        }
        let eud_s = self.mesh.as_ref().unwrap().verts[e_up_dst as usize].s;
        let eud_t = self.mesh.as_ref().unwrap().verts[e_up_dst as usize].t;
        let euo_s = self.mesh.as_ref().unwrap().verts[e_up_org as usize].s;
        let euo_t = self.mesh.as_ref().unwrap().verts[e_up_org as usize].t;

        if crate::geom::edge_sign(eud_s, eud_t, self.event_s, self.event_t, euo_s, euo_t) == 0.0 {
            self.connect_left_degenerate(reg_up, v_event);
            return;
        }

        let e_lo_dst = self.mesh.as_ref().unwrap().dst(e_lo);
        let eld_s = self.mesh.as_ref().unwrap().verts[e_lo_dst as usize].s;
        let eld_t = self.mesh.as_ref().unwrap().verts[e_lo_dst as usize].t;
        let reg = if vert_leq(eld_s, eld_t, eud_s, eud_t) {
            reg_up
        } else {
            reg_lo
        };

        let reg_up_inside = self.region(reg_up).inside;
        let reg_fix = self.region(reg).fix_upper_edge;

        if reg_up_inside || reg_fix {
            if self.trace_enabled {
                eprintln!(
                    "R   LEFT_CONNECT inside={} fixUpper={} reg={}",
                    reg_up_inside as i32,
                    reg_fix as i32,
                    if reg == reg_up { "up" } else { "lo" }
                );
            }
            let e_new = if reg == reg_up {
                // C: eNew = tessMeshConnect(mesh, vEvent->anEdge->Sym, eUp->Lnext)
                let e_up_lnext = self.mesh.as_ref().unwrap().edges[e_up as usize].lnext;
                self.mesh.as_mut().unwrap().connect(an_edge ^ 1, e_up_lnext)
            } else {
                let e_lo_dnext = self.mesh.as_ref().unwrap().dnext(e_lo);
                self.mesh
                    .as_mut()
                    .unwrap()
                    .connect(e_lo_dnext, an_edge)
                    .map(|e| e ^ 1)
            };
            let e_new = match e_new {
                Some(e) => e,
                None => return,
            };

            if reg_fix {
                if !self.fix_upper_edge(reg, e_new) {
                    return;
                }
            } else {
                self.add_region_below(reg_up, e_new);
            }
            self.sweep_event(v_event);
        } else {
            if self.trace_enabled {
                eprintln!("R   LEFT_OUTSIDE");
            }
            self.add_right_edges(reg_up, an_edge, an_edge, INVALID, true);
        }
    }

    /// Dict search: finds the first region where edge_leq(tmp_reg, region) == true.
    /// `tmp_e_up` is the e_up of a temporary region used for comparison.
    /// Returns the matching region index.
    fn dict_search_by_edge(&mut self, tmp_e_up: EdgeIdx) -> RegionIdx {
        // Temporarily allocate a region with tmp_e_up for comparison
        let tmp_reg = self.alloc_region();
        self.region_mut(tmp_reg).e_up = tmp_e_up;

        // Walk forward from head looking for the first node where edge_leq(tmp_reg, node_key)
        let mut node = self.dict.succ(DICT_HEAD);
        let result = loop {
            let key = self.dict.key(node);
            if key == INVALID {
                // Hit head (wrapped around) - not found
                break INVALID;
            }
            if self.edge_leq(tmp_reg, key) {
                break key;
            }
            node = self.dict.succ(node);
        };

        self.free_region(tmp_reg);
        result
    }

    fn sweep_event(&mut self, v_event: VertIdx) -> bool {
        let an_edge = self.mesh.as_ref().unwrap().verts[v_event as usize].an_edge;
        if an_edge == INVALID {
            return true;
        }

        if self.trace_enabled {
            let (vs, vt) = (
                self.mesh.as_ref().unwrap().verts[v_event as usize].s,
                self.mesh.as_ref().unwrap().verts[v_event as usize].t,
            );
            eprintln!(
                "R SWEEP #{} s={:.6} t={:.6}",
                self.sweep_event_num, vs, vt
            );
            self.sweep_event_num += 1;
        }

        // Walk through all edges at v_event (the onext ring).
        // If ANY has active_region != INVALID, it's already in the dict -> "right vertex" case.
        // If NONE has active_region set -> call connect_left_vertex (C: ConnectLeftVertex).
        let e_start = an_edge;
        let mut e = e_start;
        let found_e = loop {
            let ar = self.mesh.as_ref().unwrap().edges[e as usize].active_region;
            if ar != INVALID {
                break Some(e);
            }
            let next = self.mesh.as_ref().unwrap().edges[e as usize].onext;
            e = next;
            if e == e_start {
                break None;
            }
        };

        if found_e.is_none() {
            if self.trace_enabled {
                eprintln!("R   PATH left");
            }
            self.connect_left_vertex(v_event);
            return true;
        }

        // At least one edge is already in the dict.
        let e = found_e.unwrap();
        if self.trace_enabled {
            eprintln!("R   PATH right");
        }
        let reg_up = {
            let ar = self.mesh.as_ref().unwrap().edges[e as usize].active_region;
            self.top_left_region(ar)
        };
        if reg_up == INVALID {
            return false;
        }

        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return true;
        }
        let e_top_left = self.region(reg_lo).e_up;
        let e_bottom_left = self.finish_left_regions(reg_lo, INVALID);

        if e_bottom_left == INVALID {
            return true;
        }
        let e_bottom_left_onext = self.mesh.as_ref().unwrap().edges[e_bottom_left as usize].onext;
        if e_bottom_left_onext == e_top_left {
            if self.trace_enabled {
                eprintln!("R   CONNECT_RIGHT");
            }
            self.connect_right_vertex(reg_up, e_bottom_left);
        } else {
            if self.trace_enabled {
                eprintln!("R   ADD_RIGHT_EDGES");
            }
            self.add_right_edges(reg_up, e_bottom_left_onext, e_top_left, e_top_left, true);
        }
        true
    }

}

