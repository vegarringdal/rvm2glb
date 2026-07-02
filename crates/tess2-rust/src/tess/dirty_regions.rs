// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
//! Dirty-region repair for the sweep.
//!
//! Split out of `tess/sweep.rs` (libtess2 sweep.c `WalkDirtyRegions`
//! cluster): after each event the affected regions are re-checked for
//! left/right splices and edge intersections, splitting and splicing the
//! mesh until the sweep invariants hold again.

use crate::geom::{edge_intersect, edge_sign, vert_eq, vert_leq};
use crate::mesh::{INVALID};
use crate::priorityq::INVALID_HANDLE;
use super::geometry::{compute_intersect_coords};
use super::{Tessellator, TESS_UNDEF, RegionIdx};

impl Tessellator {
    pub(super) fn check_for_right_splice(&mut self, reg_up: RegionIdx) -> bool {
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

        if self.trace_enabled {
            let vleq = vert_leq(euo_s, euo_t, elo_s, elo_t);
            let es = if vleq {
                edge_sign(eld_s, eld_t, euo_s, euo_t, elo_s, elo_t)
            } else {
                edge_sign(eud_s, eud_t, elo_s, elo_t, euo_s, euo_t)
            };
            eprintln!(
                "R   CFRS euo=({euo_s:.17e},{euo_t:.17e}) elo=({elo_s:.17e},{elo_t:.17e}) eld=({eld_s:.17e},{eld_t:.17e}) eud=({eud_s:.17e},{eud_t:.17e}) vleq={} es={es:.17e}",
                vleq as i32,
            );
        }

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

    pub(super) fn check_for_left_splice(&mut self, reg_up: RegionIdx) -> bool {
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

    pub(super) fn check_for_intersect(&mut self, reg_up: RegionIdx) -> bool {
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
                // Capture the boundary EDGE now — `finish_left_regions` below
                // frees `rb` (it's the first region it finishes), so re-reading
                // `region(rb)` afterward dereferences a freed (`None`) slot.
                // libtess2 saves `eUp = RegionBelow(regUp)->eUp` before
                // `FinishLeftRegions` for exactly this reason (sweep.c).
                let e_up_new = self.region(rb).e_up;
                self.finish_left_regions(self.region_below(reg_up2), reg_lo);
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
                // Retarget reg_lo2's upper edge onto eLo->Oprev.  libtess2's C
                // original leaves the *old* eUp's `activeRegion` dangling here
                // because it recycles region memory by pointer — a stale
                // pointer is simply never dereferenced.  This port keys regions
                // by index into a `Vec` whose slots stay alive, so the old edge
                // would keep `active_region == reg_lo2`.  Once `finish_left_regions`
                // below frees reg_lo2, that orphaned edge survives in the event
                // vertex's onext ring and the next `sweep_event` walks back into
                // the freed region, panicking in `region()` on a `None` slot.
                // Sever the back-pointer before moving the region (same invariant
                // `fix_upper_edge` maintains).
                let old_e_up = self.region(reg_lo2).e_up;
                if old_e_up != INVALID
                    && self.mesh.as_ref().unwrap().edges[old_e_up as usize].active_region == reg_lo2
                {
                    self.mesh.as_mut().unwrap().edges[old_e_up as usize].active_region = INVALID;
                }
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

    pub(super) fn walk_dirty_regions(&mut self, reg_up: RegionIdx) {
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
}
