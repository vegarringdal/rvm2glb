// Copyright 2025 Lars Brubaker
// Standalone geometry helper functions for the tessellator.

use crate::geom::Real;
use crate::mesh::{Mesh, V_HEAD, INVALID};

pub(crate) fn is_valid_coord(c: Real) -> bool {
    c <= super::MAX_VALID_COORD && c >= super::MIN_VALID_COORD && !c.is_nan()
}

pub(crate) fn dot(u: &[Real; 3], v: &[Real; 3]) -> Real {
    u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
}

pub(crate) fn long_axis(v: &[Real; 3]) -> usize {
    let mut i = 0;
    if v[1].abs() > v[0].abs() {
        i = 1;
    }
    if v[2].abs() > v[i].abs() {
        i = 2;
    }
    i
}

pub(crate) fn short_axis(v: &[Real; 3]) -> usize {
    let mut i = 0;
    if v[1].abs() < v[0].abs() {
        i = 1;
    }
    if v[2].abs() < v[i].abs() {
        i = 2;
    }
    i
}

pub(crate) fn compute_normal(mesh: &Mesh, norm: &mut [Real; 3]) {
    let first_v = mesh.verts[V_HEAD as usize].next;
    if first_v == V_HEAD {
        norm[0] = 0.0;
        norm[1] = 0.0;
        norm[2] = 1.0;
        return;
    }

    let mut max_val = [0.0 as Real; 3];
    let mut min_val = [0.0 as Real; 3];
    let mut max_vert = [V_HEAD; 3];
    let mut min_vert = [V_HEAD; 3];

    for i in 0..3 {
        let c = mesh.verts[first_v as usize].coords[i];
        min_val[i] = c;
        min_vert[i] = first_v;
        max_val[i] = c;
        max_vert[i] = first_v;
    }

    let mut v = mesh.verts[V_HEAD as usize].next;
    while v != V_HEAD {
        for i in 0..3 {
            let c = mesh.verts[v as usize].coords[i];
            if c < min_val[i] {
                min_val[i] = c;
                min_vert[i] = v;
            }
            if c > max_val[i] {
                max_val[i] = c;
                max_vert[i] = v;
            }
        }
        v = mesh.verts[v as usize].next;
    }

    let mut i = 0;
    if max_val[1] - min_val[1] > max_val[0] - min_val[0] {
        i = 1;
    }
    if max_val[2] - min_val[2] > max_val[i] - min_val[i] {
        i = 2;
    }
    if min_val[i] >= max_val[i] {
        norm[0] = 0.0;
        norm[1] = 0.0;
        norm[2] = 1.0;
        return;
    }

    let v1 = min_vert[i];
    let v2 = max_vert[i];
    let d1 = [
        mesh.verts[v1 as usize].coords[0] - mesh.verts[v2 as usize].coords[0],
        mesh.verts[v1 as usize].coords[1] - mesh.verts[v2 as usize].coords[1],
        mesh.verts[v1 as usize].coords[2] - mesh.verts[v2 as usize].coords[2],
    ];

    let mut max_len2 = 0.0 as Real;
    let mut v = mesh.verts[V_HEAD as usize].next;
    while v != V_HEAD {
        let d2 = [
            mesh.verts[v as usize].coords[0] - mesh.verts[v2 as usize].coords[0],
            mesh.verts[v as usize].coords[1] - mesh.verts[v2 as usize].coords[1],
            mesh.verts[v as usize].coords[2] - mesh.verts[v2 as usize].coords[2],
        ];
        let tn = [
            d1[1] * d2[2] - d1[2] * d2[1],
            d1[2] * d2[0] - d1[0] * d2[2],
            d1[0] * d2[1] - d1[1] * d2[0],
        ];
        let tl2 = tn[0] * tn[0] + tn[1] * tn[1] + tn[2] * tn[2];
        if tl2 > max_len2 {
            max_len2 = tl2;
            *norm = tn;
        }
        v = mesh.verts[v as usize].next;
    }

    if max_len2 <= 0.0 {
        norm[0] = 0.0;
        norm[1] = 0.0;
        norm[2] = 0.0;
        norm[short_axis(&d1)] = 1.0;
    }
}

pub(crate) fn check_orientation(mesh: &mut Mesh) {
    let mut area = 0.0 as Real;
    let mut f = mesh.faces[crate::mesh::F_HEAD as usize].next;
    while f != crate::mesh::F_HEAD {
        let an = mesh.faces[f as usize].an_edge;
        if an != INVALID && mesh.edges[an as usize].winding > 0 {
            let mut e = an;
            loop {
                let org = mesh.edges[e as usize].org;
                let dst = mesh.dst(e);
                area += (mesh.verts[org as usize].s - mesh.verts[dst as usize].s)
                    * (mesh.verts[org as usize].t + mesh.verts[dst as usize].t);
                e = mesh.edges[e as usize].lnext;
                if e == an {
                    break;
                }
            }
        }
        f = mesh.faces[f as usize].next;
    }
    if area < 0.0 {
        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            mesh.verts[v as usize].t = -mesh.verts[v as usize].t;
            v = mesh.verts[v as usize].next;
        }
    }
}

/// Mirrors C `GetIntersectData` / `VertexWeights`.
/// Computes the intersection vertex's 3D coords as a weighted combination
/// of the four edge endpoints, where each edge contributes 50% of the weight
/// split between its org/dst proportional to their L1 distance to the intersection.
pub(crate) fn compute_intersect_coords(
    isect_s: Real,
    isect_t: Real,
    org_up_s: Real,
    org_up_t: Real,
    org_up_coords: [Real; 3],
    dst_up_s: Real,
    dst_up_t: Real,
    dst_up_coords: [Real; 3],
    org_lo_s: Real,
    org_lo_t: Real,
    org_lo_coords: [Real; 3],
    dst_lo_s: Real,
    dst_lo_t: Real,
    dst_lo_coords: [Real; 3],
) -> [Real; 3] {
    let l1 =
        |as_: Real, at: Real, bs: Real, bt: Real| -> Real { (as_ - bs).abs() + (at - bt).abs() };

    let mut coords = [0.0 as Real; 3];

    let t1 = l1(org_up_s, org_up_t, isect_s, isect_t);
    let t2 = l1(dst_up_s, dst_up_t, isect_s, isect_t);
    let (w0, w1) = if t1 + t2 > 0.0 {
        (0.5 * t2 / (t1 + t2), 0.5 * t1 / (t1 + t2))
    } else {
        (0.25, 0.25)
    };
    for i in 0..3 {
        coords[i] += w0 * org_up_coords[i] + w1 * dst_up_coords[i];
    }

    let t3 = l1(org_lo_s, org_lo_t, isect_s, isect_t);
    let t4 = l1(dst_lo_s, dst_lo_t, isect_s, isect_t);
    let (w2, w3) = if t3 + t4 > 0.0 {
        (0.5 * t4 / (t3 + t4), 0.5 * t3 / (t3 + t4))
    } else {
        (0.25, 0.25)
    };
    for i in 0..3 {
        coords[i] += w2 * org_lo_coords[i] + w3 * dst_lo_coords[i];
    }

    coords
}
