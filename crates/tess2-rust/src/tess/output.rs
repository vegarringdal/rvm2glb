// Copyright 2025 Lars Brubaker
// Output generation methods for the Tessellator.

use super::{ElementType, TessStatus, Tessellator, TESS_UNDEF};
use crate::mesh::{F_HEAD, INVALID, V_HEAD};

/// Is the half-edge `e` on the boundary between an inside face and an
/// outside face (or the void outside the whole mesh)?  That's the tess2
/// definition of an "original" polygon edge — an edge that came from the
/// input contour rather than being introduced by the sweep.
#[inline]
fn is_boundary_edge(mesh: &crate::mesh::Mesh, e: u32) -> bool {
    let rf = mesh.rface(e);
    // lface is always `inside` here because this helper is only called while
    // walking a face that the caller has already verified is `inside`.
    rf == INVALID || !mesh.faces[rf as usize].inside
}

impl Tessellator {
    pub(crate) fn output_polymesh(&mut self, element_type: ElementType, poly_size: usize, vertex_size: usize) {
        if poly_size > 3 {
            if let Some(ref mut mesh) = self.mesh {
                if !mesh.merge_convex_faces(poly_size) {
                    self.status = TessStatus::OutOfMemory;
                    return;
                }
            }
        }

        let mesh = match self.mesh.as_mut() {
            Some(m) => m,
            None => return,
        };

        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            mesh.verts[v as usize].n = TESS_UNDEF;
            v = mesh.verts[v as usize].next;
        }

        let mut max_vert = 0u32;
        let mut max_face = 0u32;

        let mut f = mesh.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            mesh.faces[f as usize].n = TESS_UNDEF;
            if !mesh.faces[f as usize].inside {
                f = mesh.faces[f as usize].next;
                continue;
            }

            let e_start = mesh.faces[f as usize].an_edge;
            let mut e = e_start;
            loop {
                let org = mesh.edges[e as usize].org;
                if mesh.verts[org as usize].n == TESS_UNDEF {
                    mesh.verts[org as usize].n = max_vert;
                    max_vert += 1;
                }
                e = mesh.edges[e as usize].lnext;
                if e == e_start {
                    break;
                }
            }
            mesh.faces[f as usize].n = max_face;
            max_face += 1;
            f = mesh.faces[f as usize].next;
        }

        self.out_element_count = max_face as usize;
        self.out_vertex_count = max_vert as usize;

        let stride = if element_type == ElementType::ConnectedPolygons {
            poly_size * 2
        } else {
            poly_size
        };
        self.out_elements = vec![TESS_UNDEF; max_face as usize * stride];
        self.out_vertices = vec![0.0; max_vert as usize * vertex_size];
        self.out_vertex_indices = vec![TESS_UNDEF; max_vert as usize];
        // Edge flags run parallel to the *primary* triangle-vertex slice of
        // `out_elements` (length = `max_face * poly_size`), independent of
        // the neighbour-face stride used by `ConnectedPolygons`.
        self.out_edge_flags = vec![0u8; max_face as usize * poly_size];

        let mesh = self.mesh.as_ref().unwrap();
        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            let n = mesh.verts[v as usize].n;
            if n != TESS_UNDEF {
                let base = n as usize * vertex_size;
                self.out_vertices[base] = mesh.verts[v as usize].coords[0];
                self.out_vertices[base + 1] = mesh.verts[v as usize].coords[1];
                if vertex_size > 2 {
                    self.out_vertices[base + 2] = mesh.verts[v as usize].coords[2];
                }
                self.out_vertex_indices[n as usize] = mesh.verts[v as usize].idx;
            }
            v = mesh.verts[v as usize].next;
        }

        let mut ep = 0;
        let mut efp = 0; // parallel edge-flag cursor (stride = poly_size)
        let mut f = mesh.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            if !mesh.faces[f as usize].inside {
                f = mesh.faces[f as usize].next;
                continue;
            }
            let e_start = mesh.faces[f as usize].an_edge;
            let mut e = e_start;
            let mut fv = 0;
            loop {
                // A degenerate contour can leave an edge with an INVALID (or stale,
                // out-of-range) origin vertex. Upstream C libtess2 tolerates this; here
                // a blind `verts[org]` would panic (index 0xFFFFFFFF). Emit TESS_UNDEF
                // for that corner instead — the caller drops any triangle containing it,
                // matching how rvm_parser_glb-main filters TESS_UNDEF in the output.
                let org = mesh.edges[e as usize].org;
                self.out_elements[ep] = if (org as usize) < mesh.verts.len() {
                    mesh.verts[org as usize].n
                } else {
                    TESS_UNDEF
                };
                // Edge flag: "is the edge starting at this vertex (going
                // CCW around this face) a boundary edge of the original
                // polygon?"  In our half-edge representation, the edge
                // starting at the current `org` is `e` itself — so we test
                // `e`'s right face.
                self.out_edge_flags[efp + fv] = if is_boundary_edge(mesh, e) { 1 } else { 0 };
                ep += 1;
                fv += 1;
                e = mesh.edges[e as usize].lnext;
                if e == e_start {
                    break;
                }
            }
            for _ in fv..poly_size {
                self.out_elements[ep] = TESS_UNDEF;
                // Padding slots inside `out_edge_flags` are already zero
                // from the initial `vec![0u8; …]`.
                ep += 1;
            }
            efp += poly_size;

            if element_type == ElementType::ConnectedPolygons {
                let e_start = mesh.faces[f as usize].an_edge;
                let mut e = e_start;
                let mut fv2 = 0;
                loop {
                    let rf = mesh.rface(e);
                    let nf = if rf != INVALID && mesh.faces[rf as usize].inside {
                        mesh.faces[rf as usize].n
                    } else {
                        TESS_UNDEF
                    };
                    self.out_elements[ep] = nf;
                    ep += 1;
                    fv2 += 1;
                    e = mesh.edges[e as usize].lnext;
                    if e == e_start {
                        break;
                    }
                }
                for _ in fv2..poly_size {
                    self.out_elements[ep] = TESS_UNDEF;
                    ep += 1;
                }
            }

            f = mesh.faces[f as usize].next;
        }
    }

    pub(crate) fn output_contours(&mut self, vertex_size: usize) {
        let mesh = match self.mesh.as_ref() {
            Some(m) => m,
            None => return,
        };
        let mut total_verts = 0usize;
        let mut total_elems = 0usize;
        let mut f = mesh.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            if mesh.faces[f as usize].inside {
                let e_start = mesh.faces[f as usize].an_edge;
                let mut e = e_start;
                loop {
                    total_verts += 1;
                    e = mesh.edges[e as usize].lnext;
                    if e == e_start {
                        break;
                    }
                }
                total_elems += 1;
            }
            f = mesh.faces[f as usize].next;
        }
        self.out_element_count = total_elems;
        self.out_vertex_count = total_verts;
        self.out_elements = vec![TESS_UNDEF; total_elems * 2];
        self.out_vertices = vec![0.0; total_verts * vertex_size];
        self.out_vertex_indices = vec![TESS_UNDEF; total_verts];
        // No triangles produced in BoundaryContours mode, so edge flags
        // remain empty (parallel slice would have no meaningful entries).
        self.out_edge_flags = Vec::new();

        let mesh = self.mesh.as_ref().unwrap();
        let mut vp = 0usize;
        let mut ep = 0usize;
        let mut sv = 0usize;
        let mut f = mesh.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            if !mesh.faces[f as usize].inside {
                f = mesh.faces[f as usize].next;
                continue;
            }
            let e_start = mesh.faces[f as usize].an_edge;
            let mut e = e_start;
            let mut vc = 0usize;
            loop {
                let org = mesh.edges[e as usize].org;
                let base = vp * vertex_size;
                self.out_vertices[base] = mesh.verts[org as usize].coords[0];
                self.out_vertices[base + 1] = mesh.verts[org as usize].coords[1];
                if vertex_size > 2 {
                    self.out_vertices[base + 2] = mesh.verts[org as usize].coords[2];
                }
                self.out_vertex_indices[vp] = mesh.verts[org as usize].idx;
                vp += 1;
                vc += 1;
                e = mesh.edges[e as usize].lnext;
                if e == e_start {
                    break;
                }
            }
            self.out_elements[ep] = sv as u32;
            self.out_elements[ep + 1] = vc as u32;
            ep += 2;
            sv += vc;
            f = mesh.faces[f as usize].next;
        }
    }
}
