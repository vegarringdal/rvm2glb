// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
//! Priority queue / event ordering for the sweep.
//!
//! Split out of `tess/mod.rs` (libtess2 priorityq.c usage + the sorted
//! initial event list).  Operates on the same `Tessellator` state.

use crate::geom::{Real, vert_leq};
use crate::mesh::{INVALID, V_HEAD, VertIdx};
use super::{Tessellator};

impl Tessellator {
    pub(super) fn init_priority_queue(&mut self) -> bool {
        let mesh = match self.mesh.as_ref() {
            Some(m) => m,
            None => return true,
        };
        let mut count = 0usize;
        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            count += 1;
            v = mesh.verts[v as usize].next;
        }

        // Collect (s,t,vert_idx) and sort ascending by vert_leq.
        let mut vert_coords: Vec<(Real, Real, VertIdx)> = Vec::with_capacity(count);
        let mut v = mesh.verts[V_HEAD as usize].next;
        while v != V_HEAD {
            vert_coords.push((mesh.verts[v as usize].s, mesh.verts[v as usize].t, v));
            v = mesh.verts[v as usize].next;
        }
        drop(mesh);

        vert_coords.sort_unstable_by(|a, b| {
            if vert_leq(a.0, a.1, b.0, b.1) {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        });

        // Build the sorted event queue. Store each vertex's position as a negative
        // handle (convention: -(index+1)) so that pq_delete can invalidate it.
        self.sorted_events = vert_coords.iter().map(|&(_, _, v)| v).collect();
        self.sorted_event_pos = 0;
        self.intersection_verts.clear();
        self.next_isect_handle = 0;

        // Assign each initial vertex a handle encoding its sorted_events index.
        for (idx, &(_, _, v)) in vert_coords.iter().enumerate() {
            let handle = -(idx as i32 + 1); // negative → sorted_events slot
            self.mesh.as_mut().unwrap().verts[v as usize].pq_handle = handle;
        }

        true
    }

    pub(super) fn pq_is_empty(&self) -> bool {
        self.sorted_events_min() == INVALID && self.intersection_verts.is_empty()
    }

    pub(super) fn sorted_events_min(&self) -> VertIdx {
        let mut pos = self.sorted_event_pos;
        while pos < self.sorted_events.len() {
            let v = self.sorted_events[pos];
            if v != INVALID {
                return v;
            }
            pos += 1;
        }
        INVALID
    }

    /// Find the minimum intersection vertex by scanning with coordinate comparison.
    pub(super) fn isect_minimum(&self) -> VertIdx {
        if self.intersection_verts.is_empty() {
            return INVALID;
        }
        let mesh = self.mesh.as_ref().unwrap();
        let mut best = INVALID;
        for &v in &self.intersection_verts {
            if best == INVALID {
                best = v;
            } else {
                let (bs, bt) = (mesh.verts[best as usize].s, mesh.verts[best as usize].t);
                let (vs, vt) = (mesh.verts[v as usize].s, mesh.verts[v as usize].t);
                if vert_leq(vs, vt, bs, bt) {
                    best = v;
                }
            }
        }
        best
    }

    pub(super) fn pq_minimum(&self) -> VertIdx {
        let sort_min = self.sorted_events_min();
        let isect_min = self.isect_minimum();

        match (sort_min, isect_min) {
            (INVALID, INVALID) => INVALID,
            (INVALID, h) => h,
            (s, INVALID) => s,
            (s, h) => {
                let mesh = self.mesh.as_ref().unwrap();
                let (ss, st) = (mesh.verts[s as usize].s, mesh.verts[s as usize].t);
                let (hs, ht) = (mesh.verts[h as usize].s, mesh.verts[h as usize].t);
                if vert_leq(ss, st, hs, ht) {
                    s
                } else {
                    h
                }
            }
        }
    }

    pub(super) fn pq_extract_min(&mut self) -> VertIdx {
        let v = self.pq_minimum();
        if v == INVALID {
            return INVALID;
        }

        if self.sorted_events_min() == v {
            while self.sorted_event_pos < self.sorted_events.len() {
                let s = self.sorted_events[self.sorted_event_pos];
                self.sorted_event_pos += 1;
                if s != INVALID {
                    break;
                }
            }
        } else {
            // Remove from intersection_verts
            if let Some(pos) = self.intersection_verts.iter().position(|&x| x == v) {
                self.intersection_verts.swap_remove(pos);
            }
        }
        v
    }

    pub(super) fn pq_delete(&mut self, handle: i32) {
        if handle >= 0 {
            // Intersection vertex handle: scan and remove by handle index
            let vert_idx = handle as u32;
            if let Some(pos) = self.intersection_verts.iter().position(|&x| x == vert_idx) {
                self.intersection_verts.swap_remove(pos);
            }
        } else {
            // Sorted-events handle: mark the slot as INVALID
            let idx = (-(handle + 1)) as usize;
            if idx < self.sorted_events.len() {
                self.sorted_events[idx] = INVALID;
            }
        }
    }

    pub(super) fn pq_insert(&mut self, v: VertIdx) -> i32 {
        self.intersection_verts.push(v);
        // Return the VertIdx itself as the handle (positive, so pq_delete knows it's an intersection vertex)
        v as i32
    }
}
