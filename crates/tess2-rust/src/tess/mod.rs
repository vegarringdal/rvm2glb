// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 tess.c/h + sweep.c/h + tesselator.h
//
// This module is the complete tessellator: public API + full sweep line algorithm.
// The C code is split across tess.c and sweep.c; they're merged here since both
// share the same internal state (TESStesselator).

mod api;
mod connect;
mod dirty_regions;
mod geometry;
mod output;
mod priority_queue;
mod region;
mod sweep;
#[cfg(test)]
mod tests;

pub use api::TessellatorApi;

use geometry::{check_orientation, compute_normal, dot, is_valid_coord, long_axis};

use crate::dict::Dict;
use crate::geom::{vert_eq, Real};
use crate::mesh::{Mesh, VertIdx, E_HEAD, INVALID, V_HEAD};
use crate::sweep::ActiveRegion;

// ─────────────────────────────── Public types ──────────────────────────────────

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum WindingRule {
    Odd,
    NonZero,
    Positive,
    Negative,
    AbsGeqTwo,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ElementType {
    Polygons,
    ConnectedPolygons,
    BoundaryContours,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TessOption {
    ConstrainedDelaunayTriangulation,
    ReverseContours,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TessStatus {
    Ok,
    OutOfMemory,
    InvalidInput,
}

pub const TESS_UNDEF: u32 = u32::MAX;
// Max magnitude that input coordinates can safely take without losing
// precision in the sweep.  f64's 52-bit mantissa keeps integer coords
// exact up to 2^53; we keep a conservative margin.
const MAX_VALID_COORD: Real = (1u64 << 50) as Real;
const MIN_VALID_COORD: Real = -MAX_VALID_COORD;

type RegionIdx = u32;

// ─────────────────────────── Tessellator ──────────────────────────────────────

pub struct Tessellator {
    mesh: Option<Mesh>,
    pub status: TessStatus,
    normal: [Real; 3],
    s_unit: [Real; 3],
    t_unit: [Real; 3],
    bmin: [Real; 2],
    bmax: [Real; 2],
    process_cdt: bool,
    reverse_contours: bool,
    winding_rule: WindingRule,

    // Sweep state
    dict: Dict,
    /// Intersection vertices inserted during sweep (heap replacement).
    /// Each entry is a VertIdx; ordering is done by coordinate lookup.
    intersection_verts: Vec<VertIdx>,
    next_isect_handle: i32,
    event: VertIdx,
    event_s: Real,
    event_t: Real,

    // Region arena
    regions: Vec<Option<ActiveRegion>>,
    region_free: Vec<RegionIdx>,

    // STEP2GLB PATCH: set when the sweep references a freed/invalid region;
    // makes the run fail soft (return false) instead of panicking. `Cell` so the
    // `&self` `region()` accessor can flag it. `dummy_region` is handed back on
    // the failing access so no `unwrap()` panics.
    pub(super) aborted: std::cell::Cell<bool>,
    pub(super) dummy_region: ActiveRegion,

    // Output
    pub out_vertices: Vec<Real>,
    pub out_vertex_indices: Vec<u32>,
    pub out_elements: Vec<u32>,
    /// Per triangle-vertex edge-flag (parallel to `out_elements`).
    ///
    /// `1` when the polygon edge starting at this vertex (going to the next
    /// vertex in CCW order within the same triangle) is an **original
    /// boundary edge** of the input polygon; `0` when the edge is a new
    /// interior edge added by the tessellation sweep.
    ///
    /// Parallels the C libtess2 / agg-sharp `EdgeFlagCallback` mechanism —
    /// consumers that want analytic per-edge anti-aliasing (halo strips,
    /// conservative outlines) look at each triangle's three flags and only
    /// expand the sides that are actual polygon boundaries.
    ///
    /// Populated for `ElementType::Polygons` and `ElementType::ConnectedPolygons`;
    /// empty for `ElementType::BoundaryContours` (no triangles are emitted in
    /// that mode).  Length equals `poly_size × element_count`.
    pub out_edge_flags: Vec<u8>,
    pub out_vertex_count: usize,
    pub out_element_count: usize,
    vertex_index_counter: u32,

    // Primary event queue: pre-sorted vertices for the initial sweep phase
    sorted_events: Vec<VertIdx>,
    sorted_event_pos: usize,
    sweep_event_num: u32,
    trace_enabled: bool,
}

impl Tessellator {
    pub fn new() -> Self {
        Tessellator {
            mesh: None,
            status: TessStatus::Ok,
            normal: [0.0; 3],
            s_unit: [0.0; 3],
            t_unit: [0.0; 3],
            bmin: [0.0; 2],
            bmax: [0.0; 2],
            process_cdt: false,
            reverse_contours: false,
            winding_rule: WindingRule::Odd,
            dict: Dict::new(),
            intersection_verts: Vec::new(),
            next_isect_handle: 0,
            event: INVALID,
            event_s: 0.0,
            event_t: 0.0,
            regions: Vec::new(),
            region_free: Vec::new(),
            aborted: std::cell::Cell::new(false),
            dummy_region: ActiveRegion::default(),
            out_vertices: Vec::new(),
            out_vertex_indices: Vec::new(),
            out_elements: Vec::new(),
            out_edge_flags: Vec::new(),
            out_vertex_count: 0,
            out_element_count: 0,
            vertex_index_counter: 0,
            sorted_events: Vec::new(),
            sorted_event_pos: 0,
            sweep_event_num: 0,
            trace_enabled: std::env::var("TESS_TRACE").is_ok(),
        }
    }

    pub fn set_option(&mut self, option: TessOption, value: bool) {
        match option {
            TessOption::ConstrainedDelaunayTriangulation => self.process_cdt = value,
            TessOption::ReverseContours => self.reverse_contours = value,
        }
    }

    /// Add a contour. `size` = 2 or 3 (coords per vertex). `vertices` is flat.
    ///
    /// Input type is `Real` — currently `f64` — to avoid losing precision on
    /// coordinate input.  Callers holding `f32` data should cast element-wise
    /// at the call site.
    pub fn add_contour(&mut self, size: usize, vertices: &[Real]) {
        if self.status != TessStatus::Ok {
            return;
        }
        let size = size.min(3).max(2);
        let count = vertices.len() / size;
        if self.mesh.is_none() {
            self.mesh = Some(Mesh::new());
        }

        let mut e = INVALID;
        for i in 0..count {
            let cx = vertices[i * size];
            let cy = vertices[i * size + 1];
            let cz = if size > 2 {
                vertices[i * size + 2]
            } else {
                0.0
            };

            if !is_valid_coord(cx) || !is_valid_coord(cy) || (size > 2 && !is_valid_coord(cz)) {
                self.status = TessStatus::InvalidInput;
                return;
            }

            let mesh = self.mesh.as_mut().unwrap();
            if e == INVALID {
                let new_e = match mesh.make_edge() {
                    Some(v) => v,
                    None => {
                        self.status = TessStatus::OutOfMemory;
                        return;
                    }
                };
                e = new_e;
                if !mesh.splice(e, e ^ 1) {
                    self.status = TessStatus::OutOfMemory;
                    return;
                }
            } else {
                if mesh.split_edge(e).is_none() {
                    self.status = TessStatus::OutOfMemory;
                    return;
                }
                e = mesh.edges[e as usize].lnext;
            }

            let org = mesh.edges[e as usize].org;
            mesh.verts[org as usize].coords[0] = cx;
            mesh.verts[org as usize].coords[1] = cy;
            mesh.verts[org as usize].coords[2] = cz;
            mesh.verts[org as usize].idx = self.vertex_index_counter;
            self.vertex_index_counter += 1;

            let w = if self.reverse_contours { -1 } else { 1 };
            mesh.edges[e as usize].winding = w;
            mesh.edges[(e ^ 1) as usize].winding = -w;
        }
    }

    pub fn tessellate(
        &mut self,
        winding_rule: WindingRule,
        element_type: ElementType,
        poly_size: usize,
        vertex_size: usize,
        normal: Option<[Real; 3]>,
    ) -> bool {
        if self.status != TessStatus::Ok {
            return false;
        }
        self.winding_rule = winding_rule;
        self.out_vertices.clear();
        self.out_vertex_indices.clear();
        self.out_elements.clear();
        self.out_edge_flags.clear();
        self.out_vertex_count = 0;
        self.out_element_count = 0;
        self.normal = normal.unwrap_or([0.0, 0.0, 0.0]);

        if self.mesh.is_none() {
            self.mesh = Some(Mesh::new());
        }

        if !self.project_polygon() {
            self.status = TessStatus::OutOfMemory;
            return false;
        }

        if !self.compute_interior() {
            if self.status == TessStatus::Ok {
                self.status = TessStatus::OutOfMemory;
            }
            return false;
        }

        let vertex_size = vertex_size.min(3).max(2);
        if element_type == ElementType::BoundaryContours {
            self.output_contours(vertex_size);
        } else {
            self.output_polymesh(element_type, poly_size, vertex_size);
        }

        self.mesh = None;
        self.status == TessStatus::Ok
    }

    // ─────── Accessors ────────────────────────────────────────────────────────

    pub fn vertex_count(&self) -> usize {
        self.out_vertex_count
    }
    pub fn element_count(&self) -> usize {
        self.out_element_count
    }
    pub fn vertices(&self) -> &[Real] {
        &self.out_vertices
    }
    pub fn vertex_indices(&self) -> &[u32] {
        &self.out_vertex_indices
    }
    pub fn elements(&self) -> &[u32] {
        &self.out_elements
    }
    /// Per triangle-vertex edge flags (see [`Tessellator::out_edge_flags`]).
    ///
    /// Returns an empty slice for `ElementType::BoundaryContours`.
    pub fn edge_flags(&self) -> &[u8] {
        &self.out_edge_flags
    }
    pub fn get_status(&self) -> TessStatus {
        self.status
    }

    // ─────── Projection ───────────────────────────────────────────────────────

    fn project_polygon(&mut self) -> bool {
        let mut norm = self.normal;
        let mut computed_normal = false;
        if norm[0] == 0.0 && norm[1] == 0.0 && norm[2] == 0.0 {
            if let Some(ref m) = self.mesh {
                compute_normal(m, &mut norm);
            }
            computed_normal = true;
        }

        let i = long_axis(&norm);
        self.s_unit = [0.0; 3];
        self.t_unit = [0.0; 3];
        self.s_unit[(i + 1) % 3] = 1.0;
        self.t_unit[(i + 2) % 3] = if norm[i] > 0.0 { 1.0 } else { -1.0 };
        let su = self.s_unit;
        let tu = self.t_unit;

        if let Some(ref mut mesh) = self.mesh {
            let mut v = mesh.verts[V_HEAD as usize].next;
            while v != V_HEAD {
                let c = mesh.verts[v as usize].coords;
                mesh.verts[v as usize].s = dot(&c, &su);
                mesh.verts[v as usize].t = dot(&c, &tu);
                v = mesh.verts[v as usize].next;
            }
            if computed_normal {
                check_orientation(mesh);
            }

            let mut first = true;
            let mut v = mesh.verts[V_HEAD as usize].next;
            while v != V_HEAD {
                let vs = mesh.verts[v as usize].s;
                let vt = mesh.verts[v as usize].t;
                if first {
                    self.bmin = [vs, vt];
                    self.bmax = [vs, vt];
                    first = false;
                } else {
                    if vs < self.bmin[0] {
                        self.bmin[0] = vs;
                    }
                    if vs > self.bmax[0] {
                        self.bmax[0] = vs;
                    }
                    if vt < self.bmin[1] {
                        self.bmin[1] = vt;
                    }
                    if vt > self.bmax[1] {
                        self.bmax[1] = vt;
                    }
                }
                v = mesh.verts[v as usize].next;
            }
        }
        true
    }

    // ─────── Main interior computation ───────────────────────────────────────

    fn compute_interior(&mut self) -> bool {
        self.sweep_event_num = 0;

        if !self.remove_degenerate_edges() {
            return false;
        }
        if !self.init_priority_queue() {
            return false;
        }
        if !self.init_edge_dict() {
            return false;
        }

        loop {
            // STEP2GLB PATCH: a region accessor hit a freed/invalid slot; bail
            // out of the sweep with a clean failure instead of continuing on
            // corrupt state (which would eventually panic on wasm).
            if self.aborted.get() {
                return false;
            }
            if self.pq_is_empty() {
                break;
            }

            let v = self.pq_extract_min();
            if v == INVALID {
                break;
            }

            // Coalesce coincident vertices
            loop {
                if self.pq_is_empty() {
                    break;
                }
                let next_v = self.pq_minimum();
                if next_v == INVALID {
                    break;
                }
                let (v_s, v_t) = {
                    let mesh = self.mesh.as_ref().unwrap();
                    (mesh.verts[v as usize].s, mesh.verts[v as usize].t)
                };
                let (nv_s, nv_t) = {
                    let mesh = self.mesh.as_ref().unwrap();
                    (mesh.verts[next_v as usize].s, mesh.verts[next_v as usize].t)
                };
                if !vert_eq(v_s, v_t, nv_s, nv_t) {
                    break;
                }
                let next_v = self.pq_extract_min();
                // Merge next_v into v
                let an1 = self.mesh.as_ref().unwrap().verts[v as usize].an_edge;
                let an2 = self.mesh.as_ref().unwrap().verts[next_v as usize].an_edge;
                if an1 != INVALID && an2 != INVALID {
                    if !self.mesh.as_mut().unwrap().splice(an1, an2) {
                        return false;
                    }
                }
            }

            self.event = v;
            let (v_s, v_t) = {
                let m = self.mesh.as_ref().unwrap();
                (m.verts[v as usize].s, m.verts[v as usize].t)
            };
            self.event_s = v_s;
            self.event_t = v_t;

            if !self.sweep_event(v) {
                return false;
            }
        }

        // STEP2GLB PATCH: also catch a poison set during the final event.
        if self.aborted.get() {
            return false;
        }

        self.done_edge_dict();

        let trace = self.trace_enabled;
        if let Some(ref mut mesh) = self.mesh {
            if trace {
                let mut inside = 0u32;
                let mut outside = 0u32;
                let mut f = mesh.faces[crate::mesh::F_HEAD as usize].next;
                while f != crate::mesh::F_HEAD {
                    let an = mesh.faces[f as usize].an_edge;
                    let mut edge_count = 0u32;
                    if an != INVALID {
                        let mut e = an;
                        loop {
                            edge_count += 1;
                            e = mesh.edges[e as usize].lnext;
                            if e == an { break; }
                            if edge_count > 10000 { break; }
                        }
                    }
                    if mesh.faces[f as usize].inside {
                        inside += 1;
                        eprintln!("R FACE inside edges={}", edge_count);
                    } else {
                        outside += 1;
                    }
                    f = mesh.faces[f as usize].next;
                }
                eprintln!("R FACES inside={} outside={}", inside, outside);
            }
            if !mesh.tessellate_interior() {
                return false;
            }
            if self.process_cdt {
                mesh.refine_delaunay();
            }
        }
        true
    }

    fn remove_degenerate_edges(&mut self) -> bool {
        // Mirrors C RemoveDegenerateEdges exactly
        let mesh = match self.mesh.as_mut() {
            Some(m) => m,
            None => return true,
        };
        let mut e = mesh.edges[E_HEAD as usize].next;
        while e != E_HEAD {
            let mut e_next = mesh.edges[e as usize].next;
            let mut e_lnext = mesh.edges[e as usize].lnext;

            let org = mesh.edges[e as usize].org;
            let dst = mesh.dst(e);
            let valid = org != INVALID
                && dst != INVALID
                && (org as usize) < mesh.verts.len()
                && (dst as usize) < mesh.verts.len();

            if valid {
                let (os, ot) = (mesh.verts[org as usize].s, mesh.verts[org as usize].t);
                let (ds, dt) = (mesh.verts[dst as usize].s, mesh.verts[dst as usize].t);

                if vert_eq(os, ot, ds, dt) && mesh.edges[e_lnext as usize].lnext != e {
                    // Zero-length edge, contour has at least 3 edges
                    mesh.splice(e_lnext, e);
                    if !mesh.delete_edge(e) {
                        return false;
                    }
                    e = e_lnext;
                    e_lnext = mesh.edges[e as usize].lnext;
                }
            }

            // Degenerate contour (one or two edges): e_lnext->lnext == e
            let e_lnext_lnext = mesh.edges[e_lnext as usize].lnext;
            if e_lnext_lnext == e {
                if e_lnext != e {
                    // Advance e_next past e_lnext or its sym
                    if e_lnext == e_next || e_lnext == (e_next ^ 1) {
                        e_next = mesh.edges[e_next as usize].next;
                    }
                    let w1 = mesh.edges[e_lnext as usize].winding;
                    let w2 = mesh.edges[(e_lnext ^ 1) as usize].winding;
                    mesh.edges[e as usize].winding += w1;
                    mesh.edges[(e ^ 1) as usize].winding += w2;
                    if !mesh.delete_edge(e_lnext) {
                        return false;
                    }
                }
                // Advance e_next past e or its sym
                if e == e_next || e == (e_next ^ 1) {
                    e_next = mesh.edges[e_next as usize].next;
                }
                if !mesh.delete_edge(e) {
                    return false;
                }
            }

            e = e_next;
        }
        true
    }
}
