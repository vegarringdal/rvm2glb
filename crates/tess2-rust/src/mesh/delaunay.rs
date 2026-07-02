// Copyright 2025 Lars Brubaker
// Delaunay refinement methods for Mesh.

use super::{EdgeIdx, Mesh, F_HEAD};
use crate::geom::Real;

impl Mesh {
    /// Compute the in-circle predicate for Delaunay refinement.
    pub fn in_circle(
        v_s: Real,
        v_t: Real,
        v0_s: Real,
        v0_t: Real,
        v1_s: Real,
        v1_t: Real,
        v2_s: Real,
        v2_t: Real,
    ) -> Real {
        let adx = v0_s - v_s;
        let ady = v0_t - v_t;
        let bdx = v1_s - v_s;
        let bdy = v1_t - v_t;
        let cdx = v2_s - v_s;
        let cdy = v2_t - v_t;

        let ab_det = adx * bdy - bdx * ady;
        let bc_det = bdx * cdy - cdx * bdy;
        let ca_det = cdx * ady - adx * cdy;

        let a_lift = adx * adx + ady * ady;
        let b_lift = bdx * bdx + bdy * bdy;
        let c_lift = cdx * cdx + cdy * cdy;

        a_lift * bc_det + b_lift * ca_det + c_lift * ab_det
    }

    /// Check if an edge is locally Delaunay.
    pub fn edge_is_locally_delaunay(&self, e: EdgeIdx) -> bool {
        let e_sym = e ^ 1;
        let e_sym_lnext = self.edges[e_sym as usize].lnext;
        let e_sym_lnext_lnext = self.edges[e_sym_lnext as usize].lnext;
        let e_lnext = self.edges[e as usize].lnext;
        let e_lnext_lnext = self.edges[e_lnext as usize].lnext;

        let v = self.edges[e_sym_lnext_lnext as usize].org;
        let v0 = self.edges[e_lnext as usize].org;
        let v1 = self.edges[e_lnext_lnext as usize].org;
        let v2 = self.edges[e as usize].org;

        Self::in_circle(
            self.verts[v as usize].s,
            self.verts[v as usize].t,
            self.verts[v0 as usize].s,
            self.verts[v0 as usize].t,
            self.verts[v1 as usize].s,
            self.verts[v1 as usize].t,
            self.verts[v2 as usize].s,
            self.verts[v2 as usize].t,
        ) < 0.0
    }

    /// Refine a valid triangulation into a Constrained Delaunay Triangulation.
    pub fn refine_delaunay(&mut self) {
        let mut stack: Vec<EdgeIdx> = Vec::new();

        let mut f = self.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            if self.faces[f as usize].inside {
                let e_start = self.faces[f as usize].an_edge;
                let mut e = e_start;
                loop {
                    let is_internal = self.edge_is_internal(e);
                    self.edges[e as usize].mark = is_internal;
                    if is_internal && !self.edges[(e ^ 1) as usize].mark {
                        stack.push(e);
                    }
                    e = self.edges[e as usize].lnext;
                    if e == e_start {
                        break;
                    }
                }
            }
            f = self.faces[f as usize].next;
        }

        let max_iter = stack.len() * stack.len() + 1;
        let mut iter = 0;

        while let Some(e) = stack.pop() {
            if iter >= max_iter {
                break;
            }
            iter += 1;
            self.edges[e as usize].mark = false;
            self.edges[(e ^ 1) as usize].mark = false;

            if !self.edge_is_locally_delaunay(e) {
                let neighbors = [
                    self.edges[e as usize].lnext,
                    self.lprev(e),
                    self.edges[(e ^ 1) as usize].lnext,
                    self.lprev(e ^ 1),
                ];
                self.flip_edge(e);
                for &nb in &neighbors {
                    if !self.edges[nb as usize].mark && self.edge_is_internal(nb) {
                        self.edges[nb as usize].mark = true;
                        self.edges[(nb ^ 1) as usize].mark = true;
                        stack.push(nb);
                    }
                }
            }
        }
    }
}
