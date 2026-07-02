// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
//! The sweep-line event driver.
//!
//! Core of `tess/sweep.rs` (libtess2 sweep.c): `sweep_event` dispatches
//! each event to the connect / dirty-region machinery, while
//! `add_right_edges` and `finish_left_regions` splice the new and
//! finished edges into the active region structure around it.

use crate::geom::{vert_leq};
use crate::mesh::{EdgeIdx, INVALID, VertIdx};
use super::{Tessellator, RegionIdx};

impl Tessellator {
    pub(super) fn finish_left_regions(&mut self, reg_first: RegionIdx, reg_last: RegionIdx) -> EdgeIdx {
        let mut reg_prev = reg_first;
        let mut e_prev = self.region(reg_first).e_up;

        while reg_prev != reg_last {
            self.region_mut(reg_prev).fix_upper_edge = false;
            let reg = self.region_below(reg_prev);
            if reg == INVALID {
                break;
            }
            let mut e = self.region(reg).e_up;

            let e_org = if e != INVALID {
                self.mesh.as_ref().unwrap().edges[e as usize].org
            } else {
                INVALID
            };
            let ep_org = if e_prev != INVALID {
                self.mesh.as_ref().unwrap().edges[e_prev as usize].org
            } else {
                INVALID
            };

            if e_org != ep_org {
                if !self.region(reg).fix_upper_edge {
                    self.finish_region(reg_prev);
                    break;
                }
                let ep_lprev = if e_prev != INVALID {
                    self.mesh.as_ref().unwrap().lprev(e_prev)
                } else {
                    INVALID
                };
                let e_sym = if e != INVALID { e ^ 1 } else { INVALID };
                let new_e = if ep_lprev != INVALID && e_sym != INVALID {
                    self.mesh.as_mut().unwrap().connect(ep_lprev, e_sym)
                } else {
                    None
                };
                if let Some(ne) = new_e {
                    if !self.fix_upper_edge(reg, ne) {
                        return INVALID;
                    }
                    // C: `e = tessMeshConnect(...)` — the relink splice below must
                    // operate on the newly connected edge, not the old `e`.  Not
                    // reassigning here spliced the wrong edge, corrupting the mesh
                    // topology (found by differential test vs. the C reference).
                    e = ne;
                }
            }

            if e_prev != INVALID && e != INVALID {
                let ep_onext = self.mesh.as_ref().unwrap().edges[e_prev as usize].onext;
                if ep_onext != e {
                    let e_oprev = self.mesh.as_ref().unwrap().oprev(e);
                    self.mesh.as_mut().unwrap().splice(e_oprev, e);
                    self.mesh.as_mut().unwrap().splice(e_prev, e);
                }
            }

            self.finish_region(reg_prev);
            e_prev = self.region(reg).e_up;
            reg_prev = reg;
        }
        e_prev
    }

    pub(super) fn add_right_edges(
        &mut self,
        reg_up: RegionIdx,
        e_first: EdgeIdx,
        e_last: EdgeIdx,
        e_top_left: EdgeIdx,
        clean_up: bool,
    ) {
        // Insert right-going edges into the dictionary.  Guard: the
        // onext ring must contain e_last; if it doesn't (degenerate
        // mesh), break early rather than looping forever.  libtess2's
        // C original asserts `VertLeq(e->Org, e->Dst)` — i.e., `e` is
        // right-going from the event vertex — and our previous Rust
        // port silently accepted any orientation, so a degenerate
        // input could push an edge whose SYM was already an active
        // region's `e_up` into the ring.  `add_region_below` then
        // bound both halves of the same edge pair to two different
        // regions; `walk_dirty_regions`'s degenerate-2-edge-loop
        // branch later `delete_edge`d the pair from under one of
        // them, leaving its e_up dangling and producing the wasm-only
        // `mesh.verts[INVALID]` panic in `check_for_right_splice` /
        // `walk_dirty_regions`.  See `tests/wasm_glyph_repro.rs`.
        let max_edge_iters = self.mesh.as_ref().unwrap().edges.len() + 2;
        let mut e = e_first;
        let mut edge_iter = 0usize;
        loop {
            // STEP2GLB PATCH: `e` (or a previous edge's `onext`) can be
            // INVALID/dangling after a degenerate edge was deleted — indexing
            // `edges[e]` with the INVALID sentinel (u32::MAX) would panic.
            // Bail out of the ring walk cleanly.
            if e == INVALID || (e as usize) >= self.mesh.as_ref().unwrap().edges.len() {
                break;
            }
            // Right-going invariant + duplicate-pair guard.  Either
            // condition means the edge isn't a fresh right-going
            // edge of the event vertex and must be skipped.
            let skip = {
                let mesh = self.mesh.as_ref().unwrap();
                let org = mesh.edges[e as usize].org;
                let dst = mesh.dst(e);
                let not_right_going = org != INVALID
                    && dst != INVALID
                    && !vert_leq(
                        mesh.verts[org as usize].s,
                        mesh.verts[org as usize].t,
                        mesh.verts[dst as usize].s,
                        mesh.verts[dst as usize].t,
                    );
                let sym_already_bound =
                    mesh.edges[(e ^ 1) as usize].active_region != INVALID;
                not_right_going || sym_already_bound
            };
            if !skip {
                self.add_region_below(reg_up, e ^ 1);
            }
            e = self.mesh.as_ref().unwrap().edges[e as usize].onext;
            if e == e_last {
                break;
            }
            edge_iter += 1;
            if edge_iter > max_edge_iters {
                break; // degenerate onext ring — skip remaining edges
            }
        }

        // Determine e_top_left
        let e_top_left = if e_top_left == INVALID {
            let reg_below = self.region_below(reg_up);
            if reg_below == INVALID {
                return;
            }
            let rb_e = self.region(reg_below).e_up;
            if rb_e == INVALID {
                return;
            }
            self.mesh.as_ref().unwrap().rprev(rb_e)
        } else {
            e_top_left
        };

        let mut reg_prev = reg_up;
        let mut e_prev = e_top_left;
        let mut first_time = true;
        let max_reg_iters = self.regions.len() + 2;
        let mut reg_iter2 = 0usize;

        loop {
            let reg = self.region_below(reg_prev);
            if reg == INVALID {
                break;
            }
            let e = {
                let re = self.region(reg).e_up;
                if re == INVALID {
                    break;
                }
                re ^ 1 // e = reg->eUp->Sym
            };
            let e_org = self.mesh.as_ref().unwrap().edges[e as usize].org;
            let ep_org = if e_prev != INVALID {
                self.mesh.as_ref().unwrap().edges[e_prev as usize].org
            } else {
                INVALID
            };
            if e_org != ep_org {
                break;
            }
            reg_iter2 += 1;
            if reg_iter2 > max_reg_iters {
                break; // degenerate region chain
            }

            if e_prev != INVALID {
                // C: if( e->Onext != ePrev ) { splice(e->Oprev, e); splice(ePrev->Oprev, e); }
                let e_onext = self.mesh.as_ref().unwrap().edges[e as usize].onext;
                if e_onext != e_prev {
                    let e_oprev = self.mesh.as_ref().unwrap().oprev(e);
                    self.mesh.as_mut().unwrap().splice(e_oprev, e);
                    let ep_oprev = self.mesh.as_ref().unwrap().oprev(e_prev);
                    self.mesh.as_mut().unwrap().splice(ep_oprev, e);
                }
            }

            let above_winding = self.region(reg_prev).winding_number;
            let e_winding = self.mesh.as_ref().unwrap().edges[e as usize].winding;
            let new_winding = above_winding - e_winding;
            let inside = self.is_winding_inside(new_winding);
            self.region_mut(reg).winding_number = new_winding;
            self.region_mut(reg).inside = inside;
            if self.trace_enabled {
                eprintln!("R   ARE winding={new_winding} inside={}", inside as i32);
            }

            self.region_mut(reg_prev).dirty = true;
            if !first_time {
                let cfrs = self.check_for_right_splice(reg_prev);
                if self.trace_enabled {
                    eprintln!("R   ARE_CFRS={}", cfrs as i32);
                }
                if cfrs {
                    // AddWinding
                    let re = self.region(reg).e_up;
                    let rep = self.region(reg_prev).e_up;
                    if re != INVALID && rep != INVALID {
                        let w1 = self.mesh.as_ref().unwrap().edges[re as usize].winding;
                        let w2 = self.mesh.as_ref().unwrap().edges[(re ^ 1) as usize].winding;
                        let wp1 = self.mesh.as_ref().unwrap().edges[rep as usize].winding;
                        let wp2 = self.mesh.as_ref().unwrap().edges[(rep ^ 1) as usize].winding;
                        self.mesh.as_mut().unwrap().edges[re as usize].winding += wp1;
                        self.mesh.as_mut().unwrap().edges[(re ^ 1) as usize].winding += wp2;
                    }
                    self.delete_region(reg_prev);
                    if e_prev != INVALID {
                        self.mesh.as_mut().unwrap().delete_edge(e_prev);
                    }
                }
            }
            first_time = false;
            reg_prev = reg;
            e_prev = e;
        }

        self.region_mut(reg_prev).dirty = true;

        if clean_up {
            self.walk_dirty_regions(reg_prev);
        }
    }


    pub(super) fn sweep_event(&mut self, v_event: VertIdx) -> bool {
        let an_edge = self.mesh.as_ref().unwrap().verts[v_event as usize].an_edge;
        if an_edge == INVALID {
            return true;
        }

        if self.trace_enabled {
            let (vs, vt) = (
                self.mesh.as_ref().unwrap().verts[v_event as usize].s,
                self.mesh.as_ref().unwrap().verts[v_event as usize].t,
            );
            eprintln!(
                "R SWEEP #{} s={:.6} t={:.6}",
                self.sweep_event_num, vs, vt
            );
            self.sweep_event_num += 1;
        }

        // Walk through all edges at v_event (the onext ring).
        // If ANY has active_region != INVALID, it's already in the dict -> "right vertex" case.
        // If NONE has active_region set -> call connect_left_vertex (C: ConnectLeftVertex).
        let e_start = an_edge;
        let mut e = e_start;
        let found_e = loop {
            let ar = self.mesh.as_ref().unwrap().edges[e as usize].active_region;
            if ar != INVALID {
                break Some(e);
            }
            let next = self.mesh.as_ref().unwrap().edges[e as usize].onext;
            e = next;
            if e == e_start {
                break None;
            }
        };

        if found_e.is_none() {
            if self.trace_enabled {
                eprintln!("R   PATH left");
            }
            self.connect_left_vertex(v_event);
            return true;
        }

        // At least one edge is already in the dict.
        let e = found_e.unwrap();
        if self.trace_enabled {
            eprintln!("R   PATH right");
        }
        let reg_up = {
            let ar = self.mesh.as_ref().unwrap().edges[e as usize].active_region;
            self.top_left_region(ar)
        };
        if reg_up == INVALID {
            return false;
        }

        let reg_lo = self.region_below(reg_up);
        if reg_lo == INVALID {
            return true;
        }
        let e_top_left = self.region(reg_lo).e_up;
        let e_bottom_left = self.finish_left_regions(reg_lo, INVALID);

        if e_bottom_left == INVALID {
            return true;
        }
        let e_bottom_left_onext = self.mesh.as_ref().unwrap().edges[e_bottom_left as usize].onext;
        if e_bottom_left_onext == e_top_left {
            if self.trace_enabled {
                eprintln!("R   CONNECT_RIGHT");
            }
            self.connect_right_vertex(reg_up, e_bottom_left);
        } else {
            if self.trace_enabled {
                eprintln!("R   ADD_RIGHT_EDGES");
            }
            self.add_right_edges(reg_up, e_bottom_left_onext, e_top_left, e_top_left, true);
        }
        true
    }
}
