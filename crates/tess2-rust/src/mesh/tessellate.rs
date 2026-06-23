// Monotone-region and interior tessellation for Mesh.
// Split from mod.rs to keep that file under the 967-line limit.

use super::{Mesh, F_HEAD, INVALID};

impl Mesh {
    /// Tessellate a single monotone region (face).
    /// The face must be a CCW-oriented simple polygon.
    pub fn tessellate_mono_region(&mut self, face: super::FaceIdx) -> bool {
        use crate::geom::{edge_sign, vert_leq};

        let mut up = match self.face_edge(face) {
            Some(edge) => edge,
            None => return false,
        };
        let up_lnext = match self.edge_lnext(up) {
            Some(edge) => edge,
            None => return false,
        };
        let up_lnext_lnext = match self.edge_lnext(up_lnext) {
            Some(edge) => edge,
            None => return false,
        };
        if up_lnext == up || up_lnext_lnext == up {
            return false; // degenerate face (< 3 edges) — skip instead of panic
        }

        // Find the edge whose origin vertex is rightmost (largest s).
        // VertLeq(Dst, Org) means Dst <= Org (going left = bad), so we want
        // to find an edge where the Org is the rightmost.
        let max_ring_iters = self.edges.len() + 2;
        let mut ring_iter = 0usize;
        loop {
            let (up_dst_s, up_dst_t) = match self.dst_coords(up) {
                Some(coords) => coords,
                None => return false,
            };
            let (up_org_s, up_org_t) = match self.org_coords(up) {
                Some(coords) => coords,
                None => return false,
            };
            if !vert_leq(up_dst_s, up_dst_t, up_org_s, up_org_t) {
                break;
            }
            up = match self.lprev_checked(up) {
                Some(edge) => edge,
                None => return false,
            };
            ring_iter += 1;
            if ring_iter > max_ring_iters {
                return false; // degenerate face — all vertices co-sorted
            }
        }
        ring_iter = 0;
        loop {
            let (up_org_s, up_org_t) = match self.org_coords(up) {
                Some(coords) => coords,
                None => return false,
            };
            let (up_dst_s, up_dst_t) = match self.dst_coords(up) {
                Some(coords) => coords,
                None => return false,
            };
            if !vert_leq(up_org_s, up_org_t, up_dst_s, up_dst_t) {
                break;
            }
            up = match self.edge_lnext(up) {
                Some(edge) => edge,
                None => return false,
            };
            ring_iter += 1;
            if ring_iter > max_ring_iters {
                return false; // degenerate face — all vertices co-sorted
            }
        }

        let mut lo = match self.lprev_checked(up) {
            Some(edge) => edge,
            None => return false,
        };

        let max_tess_iters = self.edges.len() * 2 + 4;
        let mut outer_iter = 0usize;
        while match self.edge_lnext(up) {
            Some(edge) => edge != lo,
            None => return false,
        } {
            outer_iter += 1;
            if outer_iter > max_tess_iters {
                return false; // degenerate region — guard against infinite triangulation
            }
            let (up_dst_s, up_dst_t) = match self.dst_coords(up) {
                Some(coords) => coords,
                None => return false,
            };
            let (lo_org_s, lo_org_t) = match self.org_coords(lo) {
                Some(coords) => coords,
                None => return false,
            };
            if vert_leq(up_dst_s, up_dst_t, lo_org_s, lo_org_t) {
                // up->Dst is on the left; make triangles from lo->Org
                let mut inner_iter = 0usize;
                while match self.edge_lnext(lo) {
                    Some(edge) => edge != up,
                    None => return false,
                } {
                    inner_iter += 1;
                    if inner_iter > max_tess_iters {
                        return false;
                    }
                    let lo_lnext = match self.edge_lnext(lo) {
                        Some(edge) => edge,
                        None => return false,
                    };
                    let (lo_lnext_dst_s, lo_lnext_dst_t) = match self.dst_coords(lo_lnext) {
                        Some(coords) => coords,
                        None => return false,
                    };
                    let (lo_org2_s, lo_org2_t) = match self.org_coords(lo) {
                        Some(coords) => coords,
                        None => return false,
                    };
                    let (lo_dst_s, lo_dst_t) = match self.dst_coords(lo) {
                        Some(coords) => coords,
                        None => return false,
                    };
                    let goes_left = match self.edge_goes_left_checked(lo_lnext) {
                        Some(value) => value,
                        None => return false,
                    };
                    let sign_val = edge_sign(
                        lo_org2_s,
                        lo_org2_t,
                        lo_dst_s,
                        lo_dst_t,
                        lo_lnext_dst_s,
                        lo_lnext_dst_t,
                    );
                    if !goes_left && sign_val > 0.0 {
                        break;
                    }
                    let temp = match self.connect(lo_lnext, lo) {
                        Some(e) => e,
                        None => return false,
                    };
                    lo = match self.valid_edge(temp ^ 1) {
                        Some(edge) => edge,
                        None => return false,
                    };
                }
                lo = match self.lprev_checked(lo) {
                    Some(edge) => edge,
                    None => return false,
                };
            } else {
                // lo->Org is on the left; make CCW triangles from up->Dst
                let mut inner_iter = 0usize;
                while match self.edge_lnext(lo) {
                    Some(edge) => edge != up,
                    None => return false,
                } {
                    inner_iter += 1;
                    if inner_iter > max_tess_iters {
                        return false;
                    }
                    let up_lprev = match self.lprev_checked(up) {
                        Some(edge) => edge,
                        None => return false,
                    };
                    let (up_lprev_org_s, up_lprev_org_t) = match self.org_coords(up_lprev) {
                        Some(coords) => coords,
                        None => return false,
                    };
                    let (up_dst2_s, up_dst2_t) = match self.dst_coords(up) {
                        Some(coords) => coords,
                        None => return false,
                    };
                    let (up_org2_s, up_org2_t) = match self.org_coords(up) {
                        Some(coords) => coords,
                        None => return false,
                    };
                    let goes_right = match self.edge_goes_right_checked(up_lprev) {
                        Some(value) => value,
                        None => return false,
                    };
                    let sign_val = edge_sign(
                        up_dst2_s,
                        up_dst2_t,
                        up_org2_s,
                        up_org2_t,
                        up_lprev_org_s,
                        up_lprev_org_t,
                    );
                    if !goes_right && sign_val < 0.0 {
                        break;
                    }
                    let temp = match self.connect(up, up_lprev) {
                        Some(e) => e,
                        None => return false,
                    };
                    up = match self.valid_edge(temp ^ 1) {
                        Some(edge) => edge,
                        None => return false,
                    };
                }
                up = match self.edge_lnext(up) {
                    Some(edge) => edge,
                    None => return false,
                };
            }
        }

        // Tessellate the remaining fan from the leftmost vertex.
        let mut lo_lnext = match self.edge_lnext(lo) {
            Some(edge) => edge,
            None => return false,
        };
        if lo_lnext == up {
            return false; // degenerate — no fan to tessellate
        }
        let mut fan_iter = 0usize;
        while match self.edge_lnext(lo_lnext) {
            Some(edge) => edge != up,
            None => return false,
        } {
            fan_iter += 1;
            if fan_iter > max_tess_iters {
                return false;
            }
            let temp = match self.connect(lo_lnext, lo) {
                Some(e) => e,
                None => return false,
            };
            lo = match self.valid_edge(temp ^ 1) {
                Some(edge) => edge,
                None => return false,
            };
            lo_lnext = match self.edge_lnext(lo) {
                Some(edge) => edge,
                None => return false,
            };
        }

        true
    }

    /// Tessellate all interior monotone regions.
    pub fn tessellate_interior(&mut self) -> bool {
        let mut f = self.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            if f == INVALID || f as usize >= self.faces.len() {
                return false;
            }
            let next = self.faces[f as usize].next;
            if self.faces[f as usize].inside {
                if !self.tessellate_mono_region(f) {
                    // Mark as outside so the output extraction skips this face.
                    // Leaving it inside=true would cause degenerate triangles with
                    // wrong vertices to be emitted (the untriangulated polygon edges
                    // get read as triangle vertices during output).
                    self.faces[f as usize].inside = false;
                }
            }
            f = next;
        }
        true
    }

    fn valid_edge(&self, edge: super::EdgeIdx) -> Option<super::EdgeIdx> {
        if edge != INVALID && (edge as usize) < self.edges.len() {
            Some(edge)
        } else {
            None
        }
    }

    fn valid_vert(&self, vert: super::VertIdx) -> Option<super::VertIdx> {
        if vert != INVALID && (vert as usize) < self.verts.len() {
            Some(vert)
        } else {
            None
        }
    }

    fn face_edge(&self, face: super::FaceIdx) -> Option<super::EdgeIdx> {
        if face == INVALID || (face as usize) >= self.faces.len() {
            return None;
        }
        self.valid_edge(self.faces[face as usize].an_edge)
    }

    fn edge_lnext(&self, edge: super::EdgeIdx) -> Option<super::EdgeIdx> {
        let edge = self.valid_edge(edge)?;
        self.valid_edge(self.edges[edge as usize].lnext)
    }

    fn dst_checked(&self, edge: super::EdgeIdx) -> Option<super::VertIdx> {
        let sym = self.valid_edge(edge ^ 1)?;
        self.valid_vert(self.edges[sym as usize].org)
    }

    fn org_coords(&self, edge: super::EdgeIdx) -> Option<(crate::geom::Real, crate::geom::Real)> {
        let edge = self.valid_edge(edge)?;
        let org = self.valid_vert(self.edges[edge as usize].org)?;
        let vert = &self.verts[org as usize];
        Some((vert.s, vert.t))
    }

    fn dst_coords(&self, edge: super::EdgeIdx) -> Option<(crate::geom::Real, crate::geom::Real)> {
        let dst = self.dst_checked(edge)?;
        let vert = &self.verts[dst as usize];
        Some((vert.s, vert.t))
    }

    fn lprev_checked(&self, edge: super::EdgeIdx) -> Option<super::EdgeIdx> {
        let edge = self.valid_edge(edge)?;
        let onext = self.valid_edge(self.edges[edge as usize].onext)?;
        self.valid_edge(onext ^ 1)
    }

    fn edge_goes_left_checked(&self, edge: super::EdgeIdx) -> Option<bool> {
        let (ds, dt) = self.dst_coords(edge)?;
        let (os, ot) = self.org_coords(edge)?;
        Some(crate::geom::vert_leq(ds, dt, os, ot))
    }

    fn edge_goes_right_checked(&self, edge: super::EdgeIdx) -> Option<bool> {
        let (os, ot) = self.org_coords(edge)?;
        let (ds, dt) = self.dst_coords(edge)?;
        Some(crate::geom::vert_leq(os, ot, ds, dt))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{Face, HalfEdge};

    #[test]
    fn invalid_face_edge_returns_false() {
        let mut mesh = Mesh::new();
        mesh.faces.push(Face {
            an_edge: INVALID,
            inside: true,
            ..Face::default()
        });

        assert!(!mesh.tessellate_mono_region(1));
    }

    #[test]
    fn invalid_destination_vertex_returns_false() {
        let mut mesh = Mesh::new();
        mesh.faces.push(Face {
            an_edge: 2,
            inside: true,
            ..Face::default()
        });
        mesh.edges.resize_with(7, HalfEdge::default);
        mesh.edges[2].lnext = 4;
        mesh.edges[4].lnext = 6;
        mesh.edges[3].org = INVALID;

        assert!(!mesh.tessellate_mono_region(1));
    }
}
