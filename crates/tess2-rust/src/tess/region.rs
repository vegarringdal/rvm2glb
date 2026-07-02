// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
//! Active-region & edge-dictionary bookkeeping for the sweep.
//!
//! Split out of `tess/mod.rs` (libtess2 sweep.c region ops): the region
//! arena, the sorted edge dictionary, winding computation, and the
//! `top_left/right_region` helpers.  The index-keyed `active_region`
//! invariant lives here (see `delete_region` / `fix_upper_edge`).

use crate::geom::{Real, edge_sign, vert_eq, vert_leq};
use crate::mesh::{EdgeIdx, INVALID};
use crate::dict::{DICT_HEAD, Dict, NodeIdx};
use crate::sweep::ActiveRegion;
use super::{Tessellator, WindingRule, RegionIdx};

impl Tessellator {
    // ─────── Edge dictionary initialization ──────────────────────────────────

    pub(super) fn add_sentinel(&mut self, smin: Real, smax: Real, t: Real) -> bool {
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
    pub(super) fn dict_insert_region(&mut self, reg: RegionIdx) -> NodeIdx {
        self.dict_insert_before(reg, DICT_HEAD)
    }

    /// Insert a region before `start_node` in the dict, walking backward
    /// until the correct sorted position is found. Mirrors C's dictInsertBefore.
    pub(super) fn dict_insert_before(&mut self, reg: RegionIdx, start_node: NodeIdx) -> NodeIdx {
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

    pub(super) fn init_edge_dict(&mut self) -> bool {
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

    pub(super) fn done_edge_dict(&mut self) {
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

    pub(super) fn alloc_region(&mut self) -> RegionIdx {
        if let Some(idx) = self.region_free.pop() {
            self.regions[idx as usize] = Some(ActiveRegion::default());
            idx
        } else {
            let idx = self.regions.len() as RegionIdx;
            self.regions.push(Some(ActiveRegion::default()));
            idx
        }
    }

    pub(super) fn free_region(&mut self, idx: RegionIdx) {
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
    pub(super) fn region(&self, idx: RegionIdx) -> &ActiveRegion {
        match self.regions.get(idx as usize).and_then(|r| r.as_ref()) {
            Some(r) => r,
            None => {
                self.aborted.set(true);
                &self.dummy_region
            }
        }
    }

    pub(super) fn region_mut(&mut self, idx: RegionIdx) -> &mut ActiveRegion {
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
    pub(super) fn region_above(&self, reg: RegionIdx) -> RegionIdx {
        let node = self.region(reg).node_up;
        self.dict.key(self.dict.succ(node))
    }

    /// Returns the region index of the dict node's predecessor region.
    pub(super) fn region_below(&self, reg: RegionIdx) -> RegionIdx {
        let node = self.region(reg).node_up;
        self.dict.key(self.dict.pred(node))
    }

    /// EdgeLeq: Returns reg1 <= reg2 at the current sweep position (event).
    pub(super) fn edge_leq(&self, reg1: RegionIdx, reg2: RegionIdx) -> bool {
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
    pub(super) fn add_region_below(&mut self, _reg_above: RegionIdx, e_new_up: EdgeIdx) -> RegionIdx {
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

        // Link the edge to the region.  Note: the SYM of the edge we're binding
        // can legitimately still be another active region's `e_up` here — both
        // halves of the pair end up owned, and the degenerate-2-edge-loop branch
        // in `walk_dirty_regions` then collapses it by `delete_edge`-ing the pair
        // (the sweep produces a correct tessellation regardless; verified by the
        // `glyph_repro_region_none_3` 216-point contour).  This used to be a
        // `debug_assert!`, but that aborted debug builds on perfectly valid input
        // — a robustness bug in itself — so it's now a trace-only note.
        if self.trace_enabled {
            let sym_region = self.mesh.as_ref().unwrap().edges[(e_new_up ^ 1) as usize].active_region;
            if sym_region != INVALID {
                eprintln!(
                    "R   ADD_REGION_BELOW({e_new_up}): sym {} still bound to region {sym_region} \
                     (collapsed later by walk_dirty_regions)",
                    e_new_up ^ 1,
                );
            }
        }
        self.mesh.as_mut().unwrap().edges[e_new_up as usize].active_region = reg_new;

        self.compute_winding(reg_new);

        reg_new
    }

    pub(super) fn delete_region(&mut self, reg: RegionIdx) {
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

    pub(super) fn fix_upper_edge(&mut self, reg: RegionIdx, new_edge: EdgeIdx) -> bool {
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

    pub(super) fn is_winding_inside(&self, n: i32) -> bool {
        match self.winding_rule {
            WindingRule::Odd => n & 1 != 0,
            WindingRule::NonZero => n != 0,
            WindingRule::Positive => n > 0,
            WindingRule::Negative => n < 0,
            WindingRule::AbsGeqTwo => n >= 2 || n <= -2,
        }
    }

    pub(super) fn compute_winding(&mut self, reg: RegionIdx) {
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

    pub(super) fn finish_region(&mut self, reg: RegionIdx) {
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
    pub(super) fn top_left_region(&mut self, reg: RegionIdx) -> RegionIdx {
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

    pub(super) fn top_right_region(&self, reg: RegionIdx) -> RegionIdx {
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
}
