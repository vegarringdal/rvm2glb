//! Builds the `--extract-json` structure dump: one `<site>.json` per exported root
//! (the full RVM tree — hierarchy + per-primitive kind/type/params/matrix) and a
//! `base.json` index (header + site list + per-site file metadata).
//!
//! Pure: each function returns a `String` the caller writes through an `OutputSink`,
//! so the same code serves the CLI, capi, and wasm. No tessellation is involved —
//! parametric shapes are emitted verbatim, except `FacetGroup`, which is reduced to
//! polygon/vertex counts (the raw contour data would dwarf everything else).

use std::collections::HashMap;

use serde_json::{Value, json};

use crate::geometry::GeometryShape;
use crate::rvm_parser::{FileMeta, HeaderBlock, MetaNode, NodePrim};

/// Serialise one exported root's node tree to JSON. The root is the lowest-id node
/// (always `1`, the first CNTB after the per-root counter reset); every other node
/// is attached under its `parent_id`. Returns a single root object, or an array if a
/// root somehow holds more than one top-level node.
pub fn site_json(nodes: &HashMap<u32, MetaNode>) -> String {
    if nodes.is_empty() {
        return "{}".to_string();
    }
    // The root carries a stale `parent_id` (the pre-reset parent's id) when split
    // below site level, which can equal another node's id. Anchoring on the lowest id
    // and excluding it from every child set keeps the walk acyclic regardless.
    let root_id = *nodes.keys().min().unwrap();
    let value = node_value(root_id, root_id, nodes);
    value.to_string()
}

fn node_value(id: u32, root_id: u32, nodes: &HashMap<u32, MetaNode>) -> Value {
    let n = &nodes[&id];

    let primitives: Vec<Value> = n.primitives.iter().map(prim_value).collect();

    let mut child_ids: Vec<u32> = nodes
        .values()
        .filter(|c| c.parent_id == id && c.id != id && c.id != root_id)
        .map(|c| c.id)
        .collect();
    child_ids.sort_unstable();
    let children: Vec<Value> = child_ids
        .iter()
        .map(|cid| node_value(*cid, root_id, nodes))
        .collect();

    json!({
        "id": n.id,
        "name": n.name,
        "opacity": n.opacity,
        "color": format!("{:06x}", n.material_id & 0x00FF_FFFF),
        "primitives": primitives,
        "children": children,
    })
}

fn prim_value(p: &NodePrim) -> Value {
    json!({
        "type": p.geo_type.as_str(),
        "kind": p.shape.kind().as_str(),
        "opacity": p.opacity,
        // RVM column-major 3×4 transform: col0, col1, col2, translation (12 floats).
        "matrix": p.world_transform.data,
        "params": shape_params(&p.shape),
    })
}

/// The parametric parameters of a shape. `FacetGroup` is summarised to counts only;
/// every other kind carries its full field set.
fn shape_params(s: &GeometryShape) -> Value {
    match s {
        GeometryShape::Pyramid {
            bottom,
            top,
            offset,
            height,
        } => json!({ "bottom": bottom, "top": top, "offset": offset, "height": height }),
        GeometryShape::Box { lengths } => json!({ "lengths": lengths }),
        GeometryShape::RectangularTorus {
            inner_radius,
            outer_radius,
            height,
            angle,
        } => json!({
            "inner_radius": inner_radius,
            "outer_radius": outer_radius,
            "height": height,
            "angle": angle,
        }),
        GeometryShape::CircularTorus {
            offset,
            radius,
            angle,
        } => json!({ "offset": offset, "radius": radius, "angle": angle }),
        GeometryShape::EllipticalDish {
            base_radius,
            height,
        } => json!({ "base_radius": base_radius, "height": height }),
        GeometryShape::SphericalDish {
            base_radius,
            height,
        } => json!({ "base_radius": base_radius, "height": height }),
        GeometryShape::Snout {
            offset,
            bshear,
            tshear,
            radius_b,
            radius_t,
            height,
        } => json!({
            "offset": offset,
            "bshear": bshear,
            "tshear": tshear,
            "radius_b": radius_b,
            "radius_t": radius_t,
            "height": height,
        }),
        GeometryShape::Cylinder { radius, height } => json!({ "radius": radius, "height": height }),
        GeometryShape::Sphere { diameter } => json!({ "diameter": diameter }),
        GeometryShape::Line { a, b } => json!({ "a": a, "b": b }),
        GeometryShape::FacetGroup { polygons } => {
            let polygon_count = polygons.len();
            let vertex_count: usize = polygons
                .iter()
                .flat_map(|p| p.contours.iter())
                .map(|c| c.vertices.len() / 3)
                .sum();
            json!({ "polygons": polygon_count, "vertices": vertex_count })
        }
    }
}

/// Build the `base.json` index: RVM header, the source file name, the split level, the
/// site name list, and one metadata entry per emitted `<site>.json` (with world bbox).
pub fn base_json(
    filemeta: &[FileMeta],
    warnings: &[String],
    header: &HeaderBlock,
    source_name: &str,
    export_lvl: u8,
) -> String {
    let sites: Vec<&str> = filemeta.iter().map(|m| m.root_name.as_str()).collect();

    let files: Vec<Value> = filemeta
        .iter()
        .map(|m| {
            json!({
                "root_name": m.root_name,
                "file_name": m.file_name,
                "parent": m.parent,
                "parent_hash": m.parent_hash,
                "min_x": m.bbox.min_x,
                "min_y": m.bbox.min_y,
                "min_z": m.bbox.min_z,
                "max_x": m.bbox.max_x,
                "max_y": m.bbox.max_y,
                "max_z": m.bbox.max_z,
            })
        })
        .collect();

    let doc = json!({
        "header": {
            "date": header.date,
            "encoding": header.encoding,
            "info": header.info,
            "note": header.note,
            "user": header.user,
            "version": header.version,
        },
        "source_file_name": source_name,
        "export_lvl": export_lvl,
        "sites": sites,
        "files": files,
        "warnings": warnings,
    });

    doc.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Contour, GeometryType, Polygon};
    use crate::linalg::Mat3x4f;

    fn node(id: u32, parent_id: u32, name: &str, prims: Vec<NodePrim>) -> MetaNode {
        MetaNode {
            id,
            parent_id,
            name: name.to_string(),
            material_id: 0x00aa_bbcc,
            opacity: 100,
            primitives: prims,
            ..MetaNode::default()
        }
    }

    fn prim(geo_type: GeometryType, shape: GeometryShape) -> NodePrim {
        NodePrim {
            opacity: 100,
            geo_type,
            vertices: Vec::new(),
            normals: Vec::new(),
            indices: Vec::new(),
            vertices_n: 0,
            triangles_n: 0,
            world_transform: Mat3x4f::identity(),
            shape,
            shape_key: 0,
        }
    }

    #[test]
    fn tree_nests_children_and_emits_kind_type_params() {
        // root(1) → child(2, Box prim) → grandchild(3, Insulation FacetGroup).
        let mut nodes = HashMap::new();
        nodes.insert(1, node(1, 0, "SITE", vec![]));
        nodes.insert(
            2,
            node(
                2,
                1,
                "ELBOW",
                vec![prim(
                    GeometryType::Primitive,
                    GeometryShape::Box {
                        lengths: [1.0, 2.0, 3.0],
                    },
                )],
            ),
        );
        nodes.insert(
            3,
            node(
                3,
                2,
                "INS",
                vec![prim(
                    GeometryType::Insulation,
                    GeometryShape::FacetGroup {
                        polygons: vec![Polygon {
                            contours: vec![Contour {
                                vertices: vec![0.0; 9], // 3 vertices
                            }],
                        }],
                    },
                )],
            ),
        );

        let v: Value = serde_json::from_str(&site_json(&nodes)).unwrap();
        assert_eq!(v["name"], "SITE");
        assert_eq!(v["color"], "aabbcc");
        assert_eq!(v["children"].as_array().unwrap().len(), 1);

        let child = &v["children"][0];
        assert_eq!(child["name"], "ELBOW");
        let box_p = &child["primitives"][0];
        assert_eq!(box_p["type"], "Primitive");
        assert_eq!(box_p["kind"], "Box");
        assert_eq!(box_p["params"]["lengths"], json!([1.0, 2.0, 3.0]));
        assert_eq!(box_p["matrix"].as_array().unwrap().len(), 12);

        let grand = &child["children"][0];
        let fg = &grand["primitives"][0];
        assert_eq!(fg["type"], "Insulation");
        assert_eq!(fg["kind"], "FacetGroup");
        // FacetGroup is reduced to counts — no contour data.
        assert_eq!(fg["params"]["polygons"], 1);
        assert_eq!(fg["params"]["vertices"], 3);
    }

    #[test]
    fn self_looping_root_does_not_recurse_forever() {
        // The split-below-site quirk: the root's parent_id can equal its own (or a
        // child's) id. Anchoring on the lowest id must still yield a finite tree.
        let mut nodes = HashMap::new();
        nodes.insert(1, node(1, 1, "ROOT", vec![])); // parent_id == id
        nodes.insert(2, node(2, 1, "KID", vec![]));
        let v: Value = serde_json::from_str(&site_json(&nodes)).unwrap();
        assert_eq!(v["name"], "ROOT");
        assert_eq!(v["children"].as_array().unwrap().len(), 1);
        assert_eq!(v["children"][0]["name"], "KID");
        assert_eq!(v["children"][0]["children"].as_array().unwrap().len(), 0);
    }
}
