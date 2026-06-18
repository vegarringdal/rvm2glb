//! Instance detection: collapse "the same shape in different places" back into a
//! shared definition. RVM has no native instancing — every occurrence of a shape
//! is written as its own `PRIM` with full parameters — so we re-discover identity
//! by keying each primitive on a placement-independent `shape_key`.
//!
//! `primitive -> (shape_key, world_transform)`: the key is invariant to where the
//! thing sits, the transform is what we stripped out to make it so. Grouping by key
//! gives instance sets (one mesh definition + N transforms). See
//! `rvm-instancing-detection.md` for the design rationale.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use crate::geometry::{Geometry, GeometryShape, Polygon, Triangulation};
use crate::linalg::Mat3x4f;
use crate::triangulation_factory::tessellate;

/// Triangulate a shape ONCE in its own local space (identity placement). The Z-up→
/// Y-up axis swap and the world placement are applied later as the glTF node matrix,
/// so the resulting mesh is shareable across every occurrence of this `shape_key`.
/// Reuses the existing tessellator unchanged (connections empty, as in `parse_prim`).
pub fn tessellate_local(
    shape: &GeometryShape,
    tolerance: f32,
    line_width: f32,
    seg_scale: f32,
    align_segments: bool,
) -> Option<Triangulation> {
    let mut geo = Geometry::new(shape.kind(), shape.clone());
    geo.m_3x4 = Mat3x4f::identity();
    // Geometry stays local (identity bake), but tessellate at the representative
    // occurrence scale so density matches what the merged path would produce.
    tessellate(
        &geo,
        &[],
        &[],
        tolerance,
        line_width,
        seg_scale,
        align_segments,
    )
}

/// Quantise a coordinate/parameter to an integer grid at `tol` resolution so that
/// values equal within tolerance hash identically (floats are never bit-equal).
#[inline]
fn q(v: f32, tol: f32) -> i64 {
    (v / tol).round() as i64
}

/// Placement-independent identity for a primitive shape.
///
/// Parametric primitives key on `(variant, quantised params)` — the canonical,
/// tolerance-stable identity (keying on triangles would miss duplicates whose
/// tessellation differs by a vertex). Facet groups have no params, so they key on
/// a canonicalised (translation-removed), quantised, order-stable vertex hash.
pub fn shape_key(shape: &GeometryShape, tol: f32) -> u64 {
    let mut h = DefaultHasher::new();
    // Distinguish variants so a Box never collides with a Cylinder etc.
    std::mem::discriminant(shape).hash(&mut h);

    match shape {
        GeometryShape::Pyramid {
            bottom,
            top,
            offset,
            height,
        } => {
            hash_arr(bottom, tol, &mut h);
            hash_arr(top, tol, &mut h);
            hash_arr(offset, tol, &mut h);
            q(*height, tol).hash(&mut h);
        }
        GeometryShape::Box { lengths } => hash_arr(lengths, tol, &mut h),
        GeometryShape::RectangularTorus {
            inner_radius,
            outer_radius,
            height,
            angle,
        } => {
            for v in [inner_radius, outer_radius, height, angle] {
                q(*v, tol).hash(&mut h);
            }
        }
        GeometryShape::CircularTorus {
            offset,
            radius,
            angle,
        } => {
            for v in [offset, radius, angle] {
                q(*v, tol).hash(&mut h);
            }
        }
        GeometryShape::EllipticalDish {
            base_radius,
            height,
        }
        | GeometryShape::SphericalDish {
            base_radius,
            height,
        } => {
            for v in [base_radius, height] {
                q(*v, tol).hash(&mut h);
            }
        }
        GeometryShape::Snout {
            offset,
            bshear,
            tshear,
            radius_b,
            radius_t,
            height,
        } => {
            hash_arr(offset, tol, &mut h);
            hash_arr(bshear, tol, &mut h);
            hash_arr(tshear, tol, &mut h);
            for v in [radius_b, radius_t, height] {
                q(*v, tol).hash(&mut h);
            }
        }
        GeometryShape::Cylinder { radius, height } => {
            for v in [radius, height] {
                q(*v, tol).hash(&mut h);
            }
        }
        GeometryShape::Sphere { diameter } => q(*diameter, tol).hash(&mut h),
        GeometryShape::Line { a, b } => {
            for v in [a, b] {
                q(*v, tol).hash(&mut h);
            }
        }
        GeometryShape::FacetGroup { polygons } => facet_hash(polygons, tol, &mut h),
    }
    h.finish()
}

fn hash_arr<const N: usize>(a: &[f32; N], tol: f32, h: &mut DefaultHasher) {
    for v in a {
        q(*v, tol).hash(h);
    }
}

/// Facet groups: translation-invariant, order-stable hash of the vertex cloud.
/// Translate so the AABB min-corner is at the origin, quantise to the `tol` grid,
/// then sort the quantised points to a deterministic order before hashing — so two
/// copies of the same mesh (different placement, vertex/winding order) hash equal.
fn facet_hash(polygons: &[Polygon], tol: f32, h: &mut DefaultHasher) {
    let mut pts: Vec<[f32; 3]> = Vec::new();
    for poly in polygons {
        for c in &poly.contours {
            for v in c.vertices.chunks_exact(3) {
                pts.push([v[0], v[1], v[2]]);
            }
        }
    }
    if pts.is_empty() {
        0u64.hash(h);
        return;
    }
    let mut min = [f32::MAX; 3];
    for p in &pts {
        for k in 0..3 {
            if p[k] < min[k] {
                min[k] = p[k];
            }
        }
    }
    let mut quant: Vec<[i64; 3]> = pts
        .iter()
        .map(|p| {
            [
                q(p[0] - min[0], tol),
                q(p[1] - min[1], tol),
                q(p[2] - min[2], tol),
            ]
        })
        .collect();
    quant.sort_unstable();
    quant.len().hash(h);
    for p in &quant {
        p.hash(h);
    }
}
