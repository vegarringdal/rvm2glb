// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
//! Vertex-connection helpers for the sweep.
//!
//! Split out of `tess/sweep.rs` (libtess2 sweep.c `Connect*` /
//! `*RightVertex` / `*LeftVertex` cluster): the cases that wire a new
//! event vertex into the active region structure, plus the dict search
//! helpers they rely on.

use crate::geom::{vert_eq, vert_leq};
use crate::mesh::{EdgeIdx, INVALID, VertIdx};
use crate::dict::DICT_HEAD;
use crate::priorityq::INVALID_HANDLE;
use super::{Tessellator, RegionIdx};

impl Tessellator {
    pub(super) fn connect_right_vertex(&mut self, reg_up: RegionIdx, e_bottom_left: EdgeIdx) {
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
    pub(super) fn connect_left_degenerate(&mut self, reg_up: RegionIdx, v_event: VertIdx) {
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
    pub(super) fn splice_merge_vertices(&mut self, e1: EdgeIdx, e2: EdgeIdx) {
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
    pub(super) fn dict_search_forward(&mut self, tmp_e_up: EdgeIdx) -> RegionIdx {
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

    pub(super) fn connect_left_vertex(&mut self, v_event: VertIdx) {
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
    pub(super) fn dict_search_by_edge(&mut self, tmp_e_up: EdgeIdx) -> RegionIdx {
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
}
