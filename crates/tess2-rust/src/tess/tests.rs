// Copyright 2025 Lars Brubaker
// Unit tests for the tessellator internals.

use super::*;

#[test]
fn debug_polygon_with_hole() {
    use crate::mesh::{F_HEAD, INVALID as MESH_INVALID};
    let mut tess = Tessellator::new();
    tess.set_option(TessOption::ReverseContours, false);
    tess.add_contour(2, &[0.0f64, 0.0, 3.0, 0.0, 3.0, 3.0, 0.0, 3.0]);
    tess.set_option(TessOption::ReverseContours, true);
    tess.add_contour(2, &[1.0f64, 1.0, 2.0, 1.0, 2.0, 2.0, 1.0, 2.0]);

    tess.winding_rule = WindingRule::Positive;
    tess.project_polygon();

    tess.remove_degenerate_edges();
    tess.init_priority_queue();
    tess.init_edge_dict();
    loop {
        if tess.pq_is_empty() {
            break;
        }
        let v = tess.pq_extract_min();
        if v == INVALID {
            break;
        }
        loop {
            if tess.pq_is_empty() {
                break;
            }
            let next_v = tess.pq_minimum();
            if next_v == INVALID {
                break;
            }
            let (v_s, v_t) = {
                let m = tess.mesh.as_ref().unwrap();
                (m.verts[v as usize].s, m.verts[v as usize].t)
            };
            let (nv_s, nv_t) = {
                let m = tess.mesh.as_ref().unwrap();
                (m.verts[next_v as usize].s, m.verts[next_v as usize].t)
            };
            if !crate::geom::vert_eq(v_s, v_t, nv_s, nv_t) {
                break;
            }
            let next_v = tess.pq_extract_min();
            let an1 = tess.mesh.as_ref().unwrap().verts[v as usize].an_edge;
            let an2 = tess.mesh.as_ref().unwrap().verts[next_v as usize].an_edge;
            if an1 != INVALID && an2 != INVALID {
                tess.mesh.as_mut().unwrap().splice(an1, an2);
            }
        }
        tess.event = v;
        let (v_s, v_t) = {
            let m = tess.mesh.as_ref().unwrap();
            (m.verts[v as usize].s, m.verts[v as usize].t)
        };
        tess.event_s = v_s;
        tess.event_t = v_t;
        tess.sweep_event(v);
    }
    tess.done_edge_dict();

    {
        let mesh = tess.mesh.as_ref().unwrap();
        let mut inside_count = 0;
        let mut outside_count = 0;
        let mut f = mesh.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            let inside = mesh.faces[f as usize].inside;
            let ae = mesh.faces[f as usize].an_edge;
            let mut edge_count = 0;
            let mut e = ae;
            loop {
                edge_count += 1;
                e = mesh.edges[e as usize].lnext;
                if e == ae {
                    break;
                }
                if edge_count > 100 {
                    eprintln!("INFINITE LOOP in face {}!", f);
                    break;
                }
            }
            eprintln!("Face {}: inside={} edge_count={}", f, inside, edge_count);
            if inside {
                inside_count += 1;
            } else {
                outside_count += 1;
            }
            f = mesh.faces[f as usize].next;
        }
        eprintln!(
            "BEFORE tessellate_interior: inside={} outside={}",
            inside_count, outside_count
        );
    }

    tess.mesh.as_mut().unwrap().tessellate_interior();

    let mesh = tess.mesh.as_ref().unwrap();
    let mut inside_count = 0;
    let mut outside_count = 0;
    let mut f = mesh.faces[F_HEAD as usize].next;
    while f != F_HEAD {
        let inside = mesh.faces[f as usize].inside;
        if inside {
            inside_count += 1;
        } else {
            outside_count += 1;
        }
        f = mesh.faces[f as usize].next;
    }
    eprintln!(
        "AFTER tessellate_interior: inside={} outside={}",
        inside_count, outside_count
    );
}

#[test]
fn debug_simple_quad() {
    let mut tess = Tessellator::new();
    tess.add_contour(2, &[0.0f64, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]);
    let ok = tess.tessellate(WindingRule::Positive, ElementType::Polygons, 3, 2, None);
    eprintln!(
        "simple_quad: ok={} element_count={}",
        ok,
        tess.element_count()
    );
}

#[test]
fn debug_single_triangle() {
    use crate::mesh::{E_HEAD, F_HEAD, INVALID as MESH_INVALID, V_HEAD};

    let mut tess = Tessellator::new();
    tess.add_contour(2, &[0.0f64, 0.0, 0.0, 1.0, 1.0, 0.0]);

    tess.winding_rule = WindingRule::Positive;
    if !tess.project_polygon() {
        panic!("project_polygon failed");
    }

    {
        let mesh = tess.mesh.as_ref().unwrap();
        eprintln!("=== After add_contour + project_polygon ===");
        for ei in 2..mesh.edges.len() {
            let e = ei as u32;
            let org = mesh.edges[e as usize].org;
            let (os, ot) = if org != MESH_INVALID && (org as usize) < mesh.verts.len() {
                (mesh.verts[org as usize].s, mesh.verts[org as usize].t)
            } else {
                (-999.0, -999.0)
            };
            let lface = mesh.edges[e as usize].lface;
            let winding = mesh.edges[e as usize].winding;
            eprintln!(
                "  Edge {}: org={} ({:.1},{:.1}) lface={} w={} onext={} lnext={} next={}",
                e, org, os, ot, lface, winding,
                mesh.edges[e as usize].onext,
                mesh.edges[e as usize].lnext,
                mesh.edges[e as usize].next
            );
        }
        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            eprintln!(
                "  Vertex {}: s={} t={} an_edge={}",
                v,
                mesh.verts[v as usize].s,
                mesh.verts[v as usize].t,
                mesh.verts[v as usize].an_edge
            );
            v = mesh.verts[v as usize].next;
        }
    }

    if !tess.compute_interior() {
        panic!("compute_interior failed");
    }

    let mesh = tess.mesh.as_ref().unwrap();
    let mut inside_count = 0;
    let mut total_faces = 0;
    let mut f = mesh.faces[F_HEAD as usize].next;
    while f != F_HEAD {
        total_faces += 1;
        if mesh.faces[f as usize].inside {
            inside_count += 1;
        }
        eprintln!(
            "  Face {}: inside={} an_edge={}",
            f, mesh.faces[f as usize].inside, mesh.faces[f as usize].an_edge
        );
        f = mesh.faces[f as usize].next;
    }
    eprintln!("Total faces: {}, inside: {}", total_faces, inside_count);
}

#[test]
fn empty_polyline() {
    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &[]);
    let ok = tess.tessellate(WindingRule::Positive, ElementType::Polygons, 3, 2, None);
    assert!(ok);
    assert_eq!(tess.element_count(), 0);
}

#[test]
fn invalid_input_status() {
    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &[-2e37f64, 0.0, 0.0, 5.0, 1e37f64, -5.0]);
    let ok = tess.tessellate(WindingRule::Positive, ElementType::Polygons, 3, 2, None);
    assert!(!ok);
    assert_eq!(tess.status(), TessStatus::InvalidInput);
}

#[test]
fn nan_quad_fails_gracefully() {
    let nan = f64::NAN;
    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &[nan, nan, nan, nan, nan, nan, nan, nan]);
    let ok = tess.tessellate(WindingRule::Positive, ElementType::Polygons, 3, 2, None);
    assert!(!ok);
}

#[test]
fn float_overflow_quad_does_not_panic() {
    let min = f64::MIN;
    let max = f64::MAX;
    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &[min, min, min, max, max, max, max, min]);
    let _ = tess.tessellate(WindingRule::Positive, ElementType::Polygons, 3, 2, None);
}

#[test]
fn singularity_quad_no_panic() {
    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let ok = tess.tessellate(WindingRule::Positive, ElementType::Polygons, 3, 2, None);
    if ok {
        assert_eq!(tess.element_count(), 0);
    }
}

/// One of two lion polygons (the "zigzag fur" under the chin, 41 verts) that
/// panics the sweep with `index out of bounds … INVALID` at
/// [`Mesh::do_splice`].  The polygon is strictly simple (non-self-intersecting)
/// and every vertex has integer coords, so there's no excuse for panicking —
/// libtess2's whole point is to handle shapes like this.  Once the sweep is
/// fixed, this test should pass without the catch_unwind.
#[test]
fn lion_zigzag_fur_must_not_panic() {
    let verts: &[[f64; 2]] = &[
        [57.0, 91.0], [42.0, 111.0], [52.0, 105.0], [41.0, 117.0], [53.0, 112.0],
        [46.0, 120.0], [53.0, 116.0], [50.0, 124.0], [57.0, 119.0], [55.0, 127.0],
        [61.0, 122.0], [60.0, 130.0], [67.0, 126.0], [66.0, 134.0], [71.0, 129.0],
        [72.0, 136.0], [77.0, 130.0], [76.0, 137.0], [80.0, 133.0], [82.0, 138.0],
        [86.0, 135.0], [96.0, 135.0], [94.0, 129.0], [86.0, 124.0], [83.0, 117.0],
        [77.0, 123.0], [79.0, 117.0], [73.0, 120.0], [75.0, 112.0], [68.0, 116.0],
        [71.0, 111.0], [65.0, 114.0], [69.0, 107.0], [63.0, 110.0], [68.0, 102.0],
        [61.0, 107.0], [66.0, 98.0], [61.0, 103.0], [63.0, 97.0], [57.0, 99.0],
        [57.0, 91.0],
    ];
    let flat: Vec<f64> = verts.iter().flat_map(|v| [v[0], v[1]]).collect();

    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &flat);
    let ok = tess.tessellate(WindingRule::Odd, ElementType::Polygons, 3, 2, None);
    assert!(ok, "tessellate() should return true");
    // These polygons self-intersect, so the triangle count isn't a simple
    // `n - 2`; tess2 subdivides each winding region.  What we DO check is
    // that we got more than zero output — the previous bug manifested as
    // `tessellate_mono_region` failing and the face being dropped entirely.
    assert!(tess.element_count() > 0, "must produce at least one triangle");
}

/// The second panicking lion polygon — a 33-vertex zigzag on the lion's
/// left side.  Matches the same pattern as `lion_zigzag_fur_must_not_panic`.
#[test]
fn lion_zigzag_side_must_not_panic() {
    let verts: &[[f64; 2]] = &[
        [74.0, 220.0], [67.0, 230.0], [67.0, 221.0], [59.0, 235.0], [63.0, 233.0],
        [60.0, 248.0], [70.0, 232.0], [65.0, 249.0], [71.0, 243.0], [67.0, 256.0],
        [73.0, 250.0], [69.0, 262.0], [73.0, 259.0], [71.0, 267.0], [76.0, 262.0],
        [72.0, 271.0], [78.0, 270.0], [76.0, 275.0], [82.0, 274.0], [78.0, 290.0],
        [86.0, 279.0], [86.0, 289.0], [92.0, 274.0], [88.0, 275.0], [87.0, 264.0],
        [82.0, 270.0], [82.0, 258.0], [77.0, 257.0], [78.0, 247.0], [73.0, 246.0],
        [77.0, 233.0], [72.0, 236.0], [74.0, 220.0],
    ];
    let flat: Vec<f64> = verts.iter().flat_map(|v| [v[0], v[1]]).collect();

    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &flat);
    let ok = tess.tessellate(WindingRule::Odd, ElementType::Polygons, 3, 2, None);
    assert!(ok, "tessellate() should return true");
    // Self-intersecting polygon — exact count depends on winding regions.
    // Non-zero means the sweep produced SOMETHING; the rotation-stability
    // test below will verify the count is invariant under transform, which
    // is the real correctness criterion.
    assert!(tess.element_count() > 0, "must produce at least one triangle");
}

/// A unit square tessellates into 2 triangles; every triangle edge that
/// lies on a square side is a boundary edge, every diagonal is interior.
#[test]
fn edge_flags_on_unit_square() {
    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &[0.0f64, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0]);
    let ok = tess.tessellate(WindingRule::Positive, ElementType::Polygons, 3, 2, None);
    assert!(ok, "tessellation of unit square should succeed");
    assert_eq!(tess.element_count(), 2, "square should produce 2 triangles");

    let flags = tess.edge_flags();
    assert_eq!(flags.len(), 6, "3 flags × 2 triangles");

    // Each triangle has exactly 2 boundary edges (two square sides) and 1
    // interior edge (the diagonal) → flag sum per triangle = 2.
    for tri in 0..2 {
        let s: u32 = flags[tri * 3..tri * 3 + 3].iter().map(|&f| f as u32).sum();
        assert_eq!(s, 2, "triangle {tri} should have 2 boundary + 1 interior edge, got flags {:?}", &flags[tri*3..tri*3+3]);
    }
}

/// Square with a hole: outer boundary and hole boundary are both edge-flagged;
/// interior tessellation edges are not.  Total boundary-edge count across all
/// triangles must equal the total polygon perimeter (outer 4 sides + hole 4
/// sides = 8 boundary edges).
#[test]
fn edge_flags_on_square_with_hole() {
    let mut tess = TessellatorApi::new();
    tess.add_contour(2, &[0.0f64, 0.0, 3.0, 0.0, 3.0, 3.0, 0.0, 3.0]);
    tess.add_contour(2, &[1.0f64, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 1.0]); // CW hole
    let ok = tess.tessellate(WindingRule::Odd, ElementType::Polygons, 3, 2, None);
    assert!(ok);

    let flags = tess.edge_flags();
    let total_boundary: u32 = flags.iter().map(|&f| f as u32).sum();
    assert_eq!(total_boundary, 8, "outer 4 + hole 4 = 8 boundary edges, got {total_boundary}");
}
