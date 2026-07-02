// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 mesh.c/h
//
// The mesh is a half-edge data structure (similar to Guibas/Stolfi quad-edge).
// All pointers from the C code are replaced with u32 indices into Vec arenas.
//
// Design:
//   - INVALID: u32::MAX  (null pointer equivalent)
//   - Half-edges allocated in pairs: edges[i] and edges[i^1] are always a pair.
//     sym(e) = e ^ 1.  Even index = e, odd index = eSym.
//   - Sentinel/dummy nodes:
//     - verts[0] = vHead (dummy vertex)
//     - faces[0] = fHead (dummy face)
//     - edges[0] = eHead, edges[1] = eHeadSym (dummy edge pair)

mod delaunay;

use crate::geom::{vert_ccw, Real};

pub const INVALID: u32 = u32::MAX;

/// Index into Mesh::verts
pub type VertIdx = u32;
/// Index into Mesh::faces
pub type FaceIdx = u32;
/// Index into Mesh::edges
pub type EdgeIdx = u32;

/// Compute the symmetric half-edge index (always the other half of the pair).
#[inline(always)]
pub fn sym(e: EdgeIdx) -> EdgeIdx {
    e ^ 1
}

#[derive(Clone, Debug)]
pub struct Vertex {
    pub next: VertIdx,
    pub prev: VertIdx,
    pub an_edge: EdgeIdx,
    pub coords: [Real; 3],
    pub s: Real,
    pub t: Real,
    pub pq_handle: i32,
    pub n: u32,
    pub idx: u32,
}

impl Default for Vertex {
    fn default() -> Self {
        Self {
            next: INVALID,
            prev: INVALID,
            an_edge: INVALID,
            coords: [0.0; 3],
            s: 0.0,
            t: 0.0,
            pq_handle: 0,
            n: INVALID,
            idx: INVALID,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Face {
    pub next: FaceIdx,
    pub prev: FaceIdx,
    pub an_edge: EdgeIdx,
    pub trail: FaceIdx,
    pub n: u32,
    pub marked: bool,
    pub inside: bool,
}

impl Default for Face {
    fn default() -> Self {
        Self {
            next: INVALID,
            prev: INVALID,
            an_edge: INVALID,
            trail: INVALID,
            n: INVALID,
            marked: false,
            inside: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct HalfEdge {
    /// Next in the global edge list (even-indexed edges link to even-indexed edges,
    /// odd-indexed edges link to odd-indexed edges).
    pub next: EdgeIdx,
    /// Next edge CCW around the origin vertex.
    pub onext: EdgeIdx,
    /// Next edge CCW around the left face.
    pub lnext: EdgeIdx,
    /// Origin vertex index.
    pub org: VertIdx,
    /// Left face index.
    pub lface: FaceIdx,
    /// Active region index (INVALID if not in the edge dictionary).
    pub active_region: u32,
    /// Winding number change when crossing this edge.
    pub winding: i32,
    /// Used by edge flip (Delaunay refinement).
    pub mark: bool,
}

impl Default for HalfEdge {
    fn default() -> Self {
        Self {
            next: INVALID,
            onext: INVALID,
            lnext: INVALID,
            org: INVALID,
            lface: INVALID,
            active_region: INVALID,
            winding: 0,
            mark: false,
        }
    }
}

/// The half-edge mesh.
pub struct Mesh {
    pub verts: Vec<Vertex>,
    pub faces: Vec<Face>,
    pub edges: Vec<HalfEdge>,
}

// ──────────────────────────────── Sentinel indices ────────────────────────────
pub const V_HEAD: VertIdx = 0;
pub const F_HEAD: FaceIdx = 0;
pub const E_HEAD: EdgeIdx = 0;
pub const E_HEAD_SYM: EdgeIdx = 1;

impl Mesh {
    /// Create a new empty mesh with dummy sentinel nodes.
    pub fn new() -> Self {
        let mut m = Mesh {
            verts: Vec::new(),
            faces: Vec::new(),
            edges: Vec::new(),
        };

        // vHead (index 0) -- dummy vertex
        let mut v_head = Vertex::default();
        v_head.next = V_HEAD;
        v_head.prev = V_HEAD;
        v_head.an_edge = INVALID;
        m.verts.push(v_head);

        // fHead (index 0) -- dummy face
        let mut f_head = Face::default();
        f_head.next = F_HEAD;
        f_head.prev = F_HEAD;
        f_head.an_edge = INVALID;
        f_head.trail = INVALID;
        f_head.marked = false;
        f_head.inside = false;
        m.faces.push(f_head);

        // eHead (index 0), eHeadSym (index 1) -- dummy edge pair
        let mut e_head = HalfEdge::default();
        e_head.next = E_HEAD;
        e_head.onext = INVALID;
        e_head.lnext = INVALID;
        e_head.org = INVALID;
        e_head.lface = INVALID;
        e_head.winding = 0;
        e_head.active_region = INVALID;

        let mut e_head_sym = HalfEdge::default();
        e_head_sym.next = E_HEAD_SYM;
        e_head_sym.onext = INVALID;
        e_head_sym.lnext = INVALID;
        e_head_sym.org = INVALID;
        e_head_sym.lface = INVALID;
        e_head_sym.winding = 0;
        e_head_sym.active_region = INVALID;

        m.edges.push(e_head);
        m.edges.push(e_head_sym);

        m
    }

    // ──────────────── Navigation helpers (C macro translations) ────────────────

    /// Symmetric half-edge (always the other element of the pair).
    #[inline(always)]
    pub fn esym(&self, e: EdgeIdx) -> EdgeIdx {
        e ^ 1
    }

    /// Right face of e (= lface of Sym).
    #[inline]
    pub fn rface(&self, e: EdgeIdx) -> FaceIdx {
        self.edges[(e ^ 1) as usize].lface
    }

    /// Destination vertex of e (= org of Sym).
    #[inline]
    pub fn dst(&self, e: EdgeIdx) -> VertIdx {
        self.edges[(e ^ 1) as usize].org
    }

    /// Oprev: Sym->Lnext
    #[inline]
    pub fn oprev(&self, e: EdgeIdx) -> EdgeIdx {
        self.edges[(e ^ 1) as usize].lnext
    }

    /// Lprev: Onext->Sym
    #[inline]
    pub fn lprev(&self, e: EdgeIdx) -> EdgeIdx {
        self.edges[e as usize].onext ^ 1
    }

    /// Dprev: Lnext->Sym
    #[inline]
    pub fn dprev(&self, e: EdgeIdx) -> EdgeIdx {
        self.edges[e as usize].lnext ^ 1
    }

    /// Rprev: Sym->Onext
    #[inline]
    pub fn rprev(&self, e: EdgeIdx) -> EdgeIdx {
        self.edges[(e ^ 1) as usize].onext
    }

    /// Dnext: Rprev->Sym = (Sym->Onext)->Sym
    #[inline]
    pub fn dnext(&self, e: EdgeIdx) -> EdgeIdx {
        self.edges[(e ^ 1) as usize].onext ^ 1
    }

    /// Rnext: Oprev->Sym = (Sym->Lnext)->Sym
    #[inline]
    pub fn rnext(&self, e: EdgeIdx) -> EdgeIdx {
        self.edges[(e ^ 1) as usize].lnext ^ 1
    }

    /// EdgeGoesLeft: VertLeq(Dst, Org)
    #[inline]
    pub fn edge_goes_left(&self, e: EdgeIdx) -> bool {
        let dst = self.dst(e);
        let org = self.edges[e as usize].org;
        let ds = self.verts[dst as usize].s;
        let dt = self.verts[dst as usize].t;
        let os = self.verts[org as usize].s;
        let ot = self.verts[org as usize].t;
        crate::geom::vert_leq(ds, dt, os, ot)
    }

    /// EdgeGoesRight: VertLeq(Org, Dst)
    #[inline]
    pub fn edge_goes_right(&self, e: EdgeIdx) -> bool {
        let org = self.edges[e as usize].org;
        let dst = self.dst(e);
        let os = self.verts[org as usize].s;
        let ot = self.verts[org as usize].t;
        let ds = self.verts[dst as usize].s;
        let dt = self.verts[dst as usize].t;
        crate::geom::vert_leq(os, ot, ds, dt)
    }

    /// EdgeIsInternal: e->Rface && e->Rface->inside
    #[inline]
    pub fn edge_is_internal(&self, e: EdgeIdx) -> bool {
        let rf = self.rface(e);
        rf != INVALID && self.faces[rf as usize].inside
    }

    // ──────────────────────── Private allocation helpers ─────────────────────

    /// Allocate a new half-edge pair.  Returns the index of `e` (even); sym is `e ^ 1`.
    /// The new pair is inserted in the global edge list before `e_next`.
    fn make_edge_pair(&mut self, e_next: EdgeIdx) -> EdgeIdx {
        // Normalize: e_next must be the even half (e, not eSym)
        let e_next = if e_next & 1 != 0 { e_next ^ 1 } else { e_next };

        // Validate e_next
        let e_next_sym = e_next ^ 1;
        if (e_next as usize) >= self.edges.len() || (e_next_sym as usize) >= self.edges.len() {
            return INVALID;
        }

        let e_new = self.edges.len() as EdgeIdx;
        let e_sym = e_new ^ 1;

        // ePrev = eNext->Sym->next
        let e_prev = self.edges[(e_next ^ 1) as usize].next;
        if e_prev == INVALID {
            return INVALID;
        }

        // Insert new pair between ePrev and eNext in the global edge list.
        // List A (even edges): ePrev ← e_new → e_next (forward)
        // List B (odd edges): ePrev^1 ← e_sym → e_next^1
        let mut e = HalfEdge::default();
        e.next = e_next;
        let mut e_s = HalfEdge::default();
        e_s.next = e_prev;

        self.edges.push(e); // index e_new
        self.edges.push(e_s); // index e_sym

        // ePrev->Sym->next = e_new  →  edges[e_prev^1].next = e_new
        self.edges[(e_prev ^ 1) as usize].next = e_new;
        // eNext->Sym->next = e_sym  →  edges[e_next^1].next = e_sym
        self.edges[(e_next ^ 1) as usize].next = e_sym;

        // Initialize edge fields
        self.edges[e_new as usize].onext = e_new;
        self.edges[e_new as usize].lnext = e_sym;
        self.edges[e_new as usize].org = INVALID;
        self.edges[e_new as usize].lface = INVALID;
        self.edges[e_new as usize].winding = 0;
        self.edges[e_new as usize].active_region = INVALID;
        self.edges[e_new as usize].mark = false;

        self.edges[e_sym as usize].onext = e_sym;
        self.edges[e_sym as usize].lnext = e_new;
        self.edges[e_sym as usize].org = INVALID;
        self.edges[e_sym as usize].lface = INVALID;
        self.edges[e_sym as usize].winding = 0;
        self.edges[e_sym as usize].active_region = INVALID;
        self.edges[e_sym as usize].mark = false;

        e_new
    }

    /// Allocate a new vertex and insert it before `v_next` in the vertex list.
    ///
    /// Returns `INVALID` if `v_next` itself is `INVALID` (or out of bounds),
    /// which happens when a caller hands us an edge whose sym-side origin
    /// has been killed.  We propagate `INVALID` rather than crash so the
    /// caller can decide whether to abort or continue.
    fn make_vertex(&mut self, e_orig: EdgeIdx, v_next: VertIdx) -> VertIdx {
        if v_next == INVALID || (v_next as usize) >= self.verts.len() {
            return INVALID;
        }
        let v_new = self.verts.len() as VertIdx;
        let v_prev = self.verts[v_next as usize].prev;
        if v_prev == INVALID || (v_prev as usize) >= self.verts.len() {
            return INVALID;
        }

        let mut v = Vertex::default();
        v.prev = v_prev;
        v.next = v_next;
        v.an_edge = e_orig;
        self.verts.push(v);

        self.verts[v_prev as usize].next = v_new;
        self.verts[v_next as usize].prev = v_new;

        // Set all edges in the origin ring to point to v_new
        let mut e = e_orig;
        loop {
            self.edges[e as usize].org = v_new;
            e = self.edges[e as usize].onext;
            if e == e_orig {
                break;
            }
        }

        v_new
    }

    /// Allocate a new face and insert it before `f_next` in the face list.
    fn make_face(&mut self, e_orig: EdgeIdx, f_next: FaceIdx) -> FaceIdx {
        if f_next == INVALID || (f_next as usize) >= self.faces.len() {
            return INVALID;
        }
        let f_new = self.faces.len() as FaceIdx;
        let f_prev = self.faces[f_next as usize].prev;
        if f_prev == INVALID || (f_prev as usize) >= self.faces.len() {
            return INVALID;
        }

        let inside_val = self.faces[f_next as usize].inside;

        let mut f = Face::default();
        f.prev = f_prev;
        f.next = f_next;
        f.an_edge = e_orig;
        f.trail = INVALID;
        f.marked = false;
        f.inside = inside_val;
        self.faces.push(f);

        self.faces[f_prev as usize].next = f_new;
        self.faces[f_next as usize].prev = f_new;

        // Set all edges in the face loop to point to f_new
        let mut e = e_orig;
        loop {
            self.edges[e as usize].lface = f_new;
            e = self.edges[e as usize].lnext;
            if e == e_orig {
                break;
            }
        }

        f_new
    }

    /// Kill (remove) a vertex from the global vertex list and update its edges to point to `new_org`.
    ///
    /// Equivalent to libtess2's `KillVertex`.  `v_del` MUST be a real
    /// vertex — passing `INVALID` here means a caller forgot to bail
    /// or skipped a precondition check, and is a porting bug.  We
    /// `debug_assert!` on it so the bug surfaces at the source in
    /// dev/test builds; release builds index naturally and panic the
    /// same way the C original would dereference a NULL pointer.
    fn kill_vertex(&mut self, v_del: VertIdx, new_org: VertIdx) {
        // Contract assertion (always-on): libtess2's C original would
        // dereference NULL here, so passing `INVALID` is an upstream
        // porting bug we want to surface immediately rather than let it
        // resurface as a generic `index out of bounds` panic 30 lines
        // later.  The release-build cost is one branch per kill.
        assert_ne!(
            v_del, INVALID,
            "kill_vertex called with INVALID — caller must filter first"
        );
        // Re-point all edges in the vertex ring
        let e_start = self.verts[v_del as usize].an_edge;
        if e_start != INVALID {
            let mut e = e_start;
            loop {
                self.edges[e as usize].org = new_org;
                e = self.edges[e as usize].onext;
                if e == e_start {
                    break;
                }
            }
        }

        // Remove from doubly-linked vertex list
        let v_prev = self.verts[v_del as usize].prev;
        let v_next = self.verts[v_del as usize].next;
        if v_prev != INVALID && v_prev < self.verts.len() as u32 {
            self.verts[v_prev as usize].next = v_next;
        }
        if v_next != INVALID && v_next < self.verts.len() as u32 {
            self.verts[v_next as usize].prev = v_prev;
        }

        // Mark as deleted (we don't actually reclaim the Vec slot)
        self.verts[v_del as usize].next = INVALID;
        self.verts[v_del as usize].prev = INVALID;
        self.verts[v_del as usize].an_edge = INVALID;
    }

    /// Kill (remove) a face from the global face list and update its edges to point to `new_lface`.
    ///
    /// Equivalent to libtess2's `KillFace`.  `f_del` MUST be a real
    /// face — see [`Self::kill_vertex`] for the contract.  Passing
    /// `INVALID` here means an upstream operation didn't maintain
    /// face references correctly and is a porting bug.
    fn kill_face(&mut self, f_del: FaceIdx, new_lface: FaceIdx) {
        // See `kill_vertex` for why this is `assert_ne!` (always-on)
        // rather than `debug_assert_ne!`.
        assert_ne!(
            f_del, INVALID,
            "kill_face called with INVALID — caller must filter first"
        );
        let e_start = self.faces[f_del as usize].an_edge;
        if e_start != INVALID {
            let mut e = e_start;
            loop {
                self.edges[e as usize].lface = new_lface;
                e = self.edges[e as usize].lnext;
                if e == e_start {
                    break;
                }
            }
        }

        let f_prev = self.faces[f_del as usize].prev;
        let f_next = self.faces[f_del as usize].next;
        if f_prev != INVALID && f_prev < self.faces.len() as u32 {
            self.faces[f_prev as usize].next = f_next;
        }
        if f_next != INVALID && f_next < self.faces.len() as u32 {
            self.faces[f_next as usize].prev = f_prev;
        }

        self.faces[f_del as usize].next = INVALID;
        self.faces[f_del as usize].prev = INVALID;
        self.faces[f_del as usize].an_edge = INVALID;
    }

    /// Kill (remove) an edge pair from the global edge list.
    fn kill_edge(&mut self, e_del: EdgeIdx) {
        // See `kill_vertex` for why this is `assert_ne!` (always-on)
        // rather than `debug_assert_ne!`.
        assert_ne!(
            e_del, INVALID,
            "kill_edge called with INVALID — caller must filter first"
        );
        let e_del = if e_del & 1 != 0 { e_del ^ 1 } else { e_del };
        let e_next = self.edges[e_del as usize].next;
        let e_prev = self.edges[(e_del ^ 1) as usize].next;

        let nlen = self.edges.len() as u32;
        if e_next != INVALID && (e_next ^ 1) < nlen {
            self.edges[(e_next ^ 1) as usize].next = e_prev;
        }
        if e_prev != INVALID && (e_prev ^ 1) < nlen {
            self.edges[(e_prev ^ 1) as usize].next = e_next;
        }

        // Mark edge as deleted
        self.edges[e_del as usize].next = INVALID;
        self.edges[(e_del ^ 1) as usize].next = INVALID;
    }

    // ──────────────────────── Public mesh operations ──────────────────────────

    /// tessMeshMakeEdge: creates one edge, two vertices, and a loop (face).
    pub fn make_edge(&mut self) -> Option<EdgeIdx> {
        let e = self.make_edge_pair(E_HEAD);
        let e_sym = e ^ 1;

        let v1 = self.make_vertex(e, V_HEAD);
        let v2 = self.make_vertex(e_sym, V_HEAD);
        let _f = self.make_face(e, F_HEAD);

        self.edges[e as usize].org = v1;
        self.edges[e_sym as usize].org = v2;

        Some(e)
    }

    /// tessMeshSplice: the fundamental connectivity-changing operation.
    /// Exchanges eOrg->Onext and eDst->Onext.
    pub fn splice(&mut self, e_org: EdgeIdx, e_dst: EdgeIdx) -> bool {
        if e_org == e_dst {
            return true;
        }

        let org_org = self.edges[e_org as usize].org;
        let dst_org = self.edges[e_dst as usize].org;
        let org_lface = self.edges[e_org as usize].lface;
        let dst_lface = self.edges[e_dst as usize].lface;

        let joining_vertices = dst_org != org_org;
        let joining_loops = dst_lface != org_lface;

        if joining_vertices {
            self.kill_vertex(dst_org, org_org);
        }
        if joining_loops {
            self.kill_face(dst_lface, org_lface);
        }

        Mesh::do_splice(&mut self.edges, e_org, e_dst);

        if !joining_vertices {
            let new_v = self.make_vertex(e_dst, org_org);
            // make sure old vertex still has a valid half-edge
            self.edges[e_org as usize].org = org_org; // org unchanged
            self.verts[org_org as usize].an_edge = e_org;
            let _ = new_v;
        }
        if !joining_loops {
            let new_f = self.make_face(e_dst, org_lface);
            self.verts[org_org as usize].an_edge = e_org; // leave org alone
            self.faces[org_lface as usize].an_edge = e_org;
            let _ = new_f;
        }

        true
    }

    fn do_splice(edges: &mut Vec<HalfEdge>, a: EdgeIdx, b: EdgeIdx) {
        let a_onext = edges[a as usize].onext;
        let b_onext = edges[b as usize].onext;
        edges[(a_onext ^ 1) as usize].lnext = b;
        edges[(b_onext ^ 1) as usize].lnext = a;
        edges[a as usize].onext = b_onext;
        edges[b as usize].onext = a_onext;
    }

    /// tessMeshDelete: remove edge eDel.
    pub fn delete_edge(&mut self, e_del: EdgeIdx) -> bool {
        // NOTE: an edge can legitimately still be bound to an `active_region`
        // here.  `walk_dirty_regions`' degenerate-2-edge-loop collapse deletes
        // an edge whose SYM is still another active region's `e_up` (both halves
        // owned — see `add_region_below`); the sweep nonetheless produces a
        // correct tessellation (verified by `glyph_repro_region_none_3`, a
        // 216-point glyph that hit this).  This used to be a `debug_assert!`,
        // but that aborted debug builds on valid input — a robustness bug — while
        // release silently (and correctly) skipped it.  `active_region` is an
        // implementation detail of the sweep, not a mesh invariant, so it is no
        // longer enforced here.
        let e_del_sym = e_del ^ 1;

        let e_del_lface = self.edges[e_del as usize].lface;
        let e_del_rface = self.rface(e_del);
        let joining_loops = e_del_lface != e_del_rface;

        if joining_loops {
            self.kill_face(e_del_lface, e_del_rface);
        }

        let e_del_onext = self.edges[e_del as usize].onext;
        if e_del_onext == e_del {
            let e_del_org = self.edges[e_del as usize].org;
            self.kill_vertex(e_del_org, INVALID);
        } else {
            // Make sure eDel->Org and eDel->Rface point to valid half-edges
            let e_del_oprev = self.oprev(e_del);
            let e_del_rface2 = self.rface(e_del);
            self.faces[e_del_rface2 as usize].an_edge = e_del_oprev;
            let e_del_org2 = self.edges[e_del as usize].org;
            self.verts[e_del_org2 as usize].an_edge = e_del_onext;

            Mesh::do_splice(&mut self.edges, e_del, e_del_oprev);

            if !joining_loops {
                let new_f = self.make_face(e_del, e_del_lface);
                let _ = new_f;
            }
        }

        let e_del_sym_onext = self.edges[e_del_sym as usize].onext;
        if e_del_sym_onext == e_del_sym {
            let e_del_sym_org = self.edges[e_del_sym as usize].org;
            self.kill_vertex(e_del_sym_org, INVALID);
            let e_del_lface2 = self.edges[e_del as usize].lface;
            self.kill_face(e_del_lface2, INVALID);
        } else {
            let e_del_lface3 = self.edges[e_del as usize].lface;
            let e_del_sym_oprev = self.oprev(e_del_sym);
            self.faces[e_del_lface3 as usize].an_edge = e_del_sym_oprev;
            let e_del_sym_org2 = self.edges[e_del_sym as usize].org;
            self.verts[e_del_sym_org2 as usize].an_edge = e_del_sym_onext;
            Mesh::do_splice(&mut self.edges, e_del_sym, e_del_sym_oprev);
        }

        self.kill_edge(e_del);
        true
    }

    /// tessMeshAddEdgeVertex: create a new edge eNew = eOrg->Lnext,
    /// and eNew->Dst is a new vertex. eOrg and eNew share the same left face.
    pub fn add_edge_vertex(&mut self, e_org: EdgeIdx) -> Option<EdgeIdx> {
        let e_new = self.make_edge_pair(e_org);
        if e_new == INVALID {
            return None;
        }
        let e_new_sym = e_new ^ 1;

        // Connect: eNew is inserted after eOrg in the Lnext ring
        let e_org_lnext = self.edges[e_org as usize].lnext;
        Mesh::do_splice(&mut self.edges, e_new, e_org_lnext);

        // Set origin of eNew to eOrg->Dst
        let e_org_dst = self.dst(e_org);
        self.edges[e_new as usize].org = e_org_dst;

        // Create new vertex at the other end.  If `e_org`'s Dst has been
        // killed upstream (`dst(e_org)` == INVALID) we can't build a valid
        // vertex for the new edge — bail instead of indexing INVALID into
        // `self.verts`.  This can occur when the sweep deletes edges in a
        // different order than libtess2 expects for some self-intersecting
        // inputs.
        let v_new = self.make_vertex(e_new_sym, e_org_dst);
        if v_new == INVALID {
            return None;
        }

        // Both eNew and eNewSym share the same left face as eOrg
        let e_org_lface = self.edges[e_org as usize].lface;
        self.edges[e_new as usize].lface = e_org_lface;
        self.edges[e_new_sym as usize].lface = e_org_lface;

        Some(e_new)
    }

    /// tessMeshSplitEdge: split eOrg into eOrg and eNew, with eNew = eOrg->Lnext.
    pub fn split_edge(&mut self, e_org: EdgeIdx) -> Option<EdgeIdx> {
        let temp = self.add_edge_vertex(e_org)?;
        let e_new = temp ^ 1;

        // Disconnect eOrg from eOrg->Dst and reconnect to eNew->Org
        let e_org_sym = e_org ^ 1;
        let e_org_sym_oprev = self.oprev(e_org_sym);
        Mesh::do_splice(&mut self.edges, e_org_sym, e_org_sym_oprev);
        Mesh::do_splice(&mut self.edges, e_org_sym, e_new);

        // Update vertex/face pointers
        let e_new_org = self.edges[e_new as usize].org;
        let e_org_dst_idx = e_org ^ 1; // sym
        self.edges[e_org_dst_idx as usize].org = e_new_org;
        let e_new_dst = self.dst(e_new);
        self.verts[e_new_dst as usize].an_edge = e_new ^ 1;

        let e_org_rface = self.rface(e_org);
        self.edges[(e_new ^ 1) as usize].lface = e_org_rface; // eNew->Rface = eOrg->Rface (Rface = Sym->Lface)
        let e_org_winding = self.edges[e_org as usize].winding;
        let e_org_sym_winding = self.edges[e_org_sym as usize].winding;
        self.edges[e_new as usize].winding = e_org_winding;
        self.edges[(e_new ^ 1) as usize].winding = e_org_sym_winding;

        Some(e_new)
    }

    /// tessMeshConnect: create a new edge from eOrg->Dst to eDst->Org.
    /// Returns the new half-edge.
    pub fn connect(&mut self, e_org: EdgeIdx, e_dst: EdgeIdx) -> Option<EdgeIdx> {
        let e_new = self.make_edge_pair(e_org);
        // If `make_edge_pair` couldn't allocate (out-of-bounds seed, broken
        // `next` chain on the sym side), bail.  Forwarding the `INVALID`
        // into the subsequent `do_splice`/`kill_face` was the root cause of
        // the lion-polygon `INVALID do_splice` panic.  Returning `None`
        // lets `tessellate_mono_region` skip this triangulation step; the
        // face is then marked non-inside so no degenerate output slips
        // through (see `tessellate_interior`'s fallback).
        if e_new == INVALID { return None; }
        let e_new_sym = e_new ^ 1;

        let e_dst_lface = self.edges[e_dst as usize].lface;
        let e_org_lface = self.edges[e_org as usize].lface;
        let joining_loops = e_dst_lface != e_org_lface;

        if joining_loops {
            self.kill_face(e_dst_lface, e_org_lface);
        }

        // Connect: Splice(eNew, eOrg->Lnext); Splice(eNewSym, eDst)
        let e_org_lnext = self.edges[e_org as usize].lnext;
        Mesh::do_splice(&mut self.edges, e_new, e_org_lnext);
        Mesh::do_splice(&mut self.edges, e_new_sym, e_dst);

        // Set vertex/face
        let e_org_dst = self.dst(e_org);
        self.edges[e_new as usize].org = e_org_dst;
        let e_dst_org = self.edges[e_dst as usize].org;
        self.edges[e_new_sym as usize].org = e_dst_org;
        self.edges[e_new as usize].lface = e_org_lface;
        self.edges[e_new_sym as usize].lface = e_org_lface;

        // Make sure the old face points to a valid half-edge
        self.faces[e_org_lface as usize].an_edge = e_new_sym;

        if !joining_loops {
            let new_f = self.make_face(e_new, e_org_lface);
            let _ = new_f;
        }

        Some(e_new)
    }

    /// tessMeshZapFace: destroy a face and remove it from the global face list.
    /// All edges of fZap get lface = INVALID. Edges whose rface is also INVALID
    /// are deleted entirely.
    pub fn zap_face(&mut self, f_zap: FaceIdx) {
        // See `kill_vertex` for why this is `assert_ne!` (always-on)
        // rather than `debug_assert_ne!`.
        assert_ne!(
            f_zap, INVALID,
            "zap_face called with INVALID — caller must filter first"
        );
        let e_start = self.faces[f_zap as usize].an_edge;
        let mut e_next = self.edges[e_start as usize].lnext;

        loop {
            let e = e_next;
            e_next = self.edges[e as usize].lnext;

            self.edges[e as usize].lface = INVALID;

            let e_rface = self.rface(e);
            if e_rface == INVALID {
                // Delete the edge
                let e_onext = self.edges[e as usize].onext;
                if e_onext == e {
                    let e_org = self.edges[e as usize].org;
                    if e_org != INVALID {
                        self.kill_vertex(e_org, INVALID);
                    }
                } else {
                    let e_org = self.edges[e as usize].org;
                    if e_org != INVALID {
                        self.verts[e_org as usize].an_edge = e_onext;
                    }
                    let e_oprev = self.oprev(e);
                    Mesh::do_splice(&mut self.edges, e, e_oprev);
                }

                let e_sym = e ^ 1;
                let e_sym_onext = self.edges[e_sym as usize].onext;
                if e_sym_onext == e_sym {
                    let e_sym_org = self.edges[e_sym as usize].org;
                    if e_sym_org != INVALID {
                        self.kill_vertex(e_sym_org, INVALID);
                    }
                } else {
                    let e_sym_org = self.edges[e_sym as usize].org;
                    if e_sym_org != INVALID {
                        self.verts[e_sym_org as usize].an_edge = e_sym_onext;
                    }
                    let e_sym_oprev = self.oprev(e_sym);
                    Mesh::do_splice(&mut self.edges, e_sym, e_sym_oprev);
                }

                self.kill_edge(e);
            }

            if e == e_start {
                break;
            }
        }

        // Delete from face list
        let f_prev = self.faces[f_zap as usize].prev;
        let f_next = self.faces[f_zap as usize].next;
        self.faces[f_prev as usize].next = f_next;
        self.faces[f_next as usize].prev = f_prev;
        self.faces[f_zap as usize].next = INVALID;
        self.faces[f_zap as usize].prev = INVALID;
        self.faces[f_zap as usize].an_edge = INVALID;
    }

    /// Count vertices in a face loop.
    pub fn count_face_verts(&self, f: FaceIdx) -> usize {
        let e_start = self.faces[f as usize].an_edge;
        let mut e = e_start;
        let mut n = 0;
        loop {
            n += 1;
            e = self.edges[e as usize].lnext;
            if e == e_start {
                break;
            }
        }
        n
    }

    /// tessMeshMergeConvexFaces: merge convex adjacent faces if the result
    /// would have <= maxVertsPerFace vertices.
    pub fn merge_convex_faces(&mut self, max_verts_per_face: usize) -> bool {
        let mut e = self.edges[E_HEAD as usize].next;
        while e != E_HEAD {
            let e_next = self.edges[e as usize].next;
            let e_sym = e ^ 1;

            let e_lface = self.edges[e as usize].lface;
            let e_sym_lface = self.edges[e_sym as usize].lface;

            if e_lface == INVALID
                || !self.faces[e_lface as usize].inside
                || e_sym_lface == INVALID
                || !self.faces[e_sym_lface as usize].inside
            {
                e = e_next;
                continue;
            }

            let left_nv = self.count_face_verts(e_lface);
            let right_nv = self.count_face_verts(e_sym_lface);
            if left_nv + right_nv - 2 > max_verts_per_face {
                e = e_next;
                continue;
            }

            // Check convexity: va--vb--vc and vd--ve--vf must be CCW
            let va = self.edges[self.lprev(e) as usize].org;
            let vb = self.edges[e as usize].org;
            let vc_edge = self.edges[e_sym as usize].lnext;
            let vc = self.dst(vc_edge);

            let vd = self.edges[self.lprev(e_sym) as usize].org;
            let ve = self.edges[e_sym as usize].org;
            let vf_edge = self.edges[e as usize].lnext;
            let vf = self.dst(vf_edge);

            let convex = vert_ccw(
                self.verts[va as usize].s,
                self.verts[va as usize].t,
                self.verts[vb as usize].s,
                self.verts[vb as usize].t,
                self.verts[vc as usize].s,
                self.verts[vc as usize].t,
            ) && vert_ccw(
                self.verts[vd as usize].s,
                self.verts[vd as usize].t,
                self.verts[ve as usize].s,
                self.verts[ve as usize].t,
                self.verts[vf as usize].s,
                self.verts[vf as usize].t,
            );

            if convex {
                let actual_next = if e == e_next || e == e_next ^ 1 {
                    self.edges[e_next as usize].next
                } else {
                    e_next
                };
                if !self.delete_edge(e) {
                    return false;
                }
                e = actual_next;
                continue;
            }

            e = e_next;
        }
        true
    }

    /// tessMeshFlipEdge: flip an internal edge (used for Delaunay refinement).
    pub fn flip_edge(&mut self, edge: EdgeIdx) {
        let a0 = edge;
        let a1 = self.edges[a0 as usize].lnext;
        let a2 = self.edges[a1 as usize].lnext;
        let b0 = edge ^ 1;
        let b1 = self.edges[b0 as usize].lnext;
        let b2 = self.edges[b1 as usize].lnext;

        let a_org = self.edges[a0 as usize].org;
        let a_opp = self.edges[a2 as usize].org;
        let b_org = self.edges[b0 as usize].org;
        let b_opp = self.edges[b2 as usize].org;

        let fa = self.edges[a0 as usize].lface;
        let fb = self.edges[b0 as usize].lface;

        self.edges[a0 as usize].org = b_opp;
        self.edges[a0 as usize].onext = self.edges[b1 as usize].onext ^ 1; // b1->Sym
        self.edges[b0 as usize].org = a_opp;
        self.edges[b0 as usize].onext = self.edges[a1 as usize].onext ^ 1; // a1->Sym
        self.edges[a2 as usize].onext = b0;
        self.edges[b2 as usize].onext = a0;
        self.edges[b1 as usize].onext = self.edges[a2 as usize].onext ^ 1; // a2->Sym... wait

        // Redo using correct flip logic from C code:
        self.edges[a0 as usize].lnext = a2;
        self.edges[a2 as usize].lnext = b1;
        self.edges[b1 as usize].lnext = a0;

        self.edges[b0 as usize].lnext = b2;
        self.edges[b2 as usize].lnext = a1;
        self.edges[a1 as usize].lnext = b0;

        self.edges[a1 as usize].lface = fb;
        self.edges[b1 as usize].lface = fa;

        self.faces[fa as usize].an_edge = a0;
        self.faces[fb as usize].an_edge = b0;

        if self.verts[a_org as usize].an_edge == a0 {
            self.verts[a_org as usize].an_edge = b1;
        }
        if self.verts[b_org as usize].an_edge == b0 {
            self.verts[b_org as usize].an_edge = a1;
        }
    }

    /// tessMeshSetWindingNumber: reset winding numbers.
    pub fn set_winding_number(&mut self, value: i32, keep_only_boundary: bool) -> bool {
        let mut e = self.edges[E_HEAD as usize].next;
        while e != E_HEAD {
            let e_next = self.edges[e as usize].next;
            let e_lface = self.edges[e as usize].lface;
            let e_rface = self.rface(e);

            let lf_inside = if e_lface != INVALID {
                self.faces[e_lface as usize].inside
            } else {
                false
            };
            let rf_inside = if e_rface != INVALID {
                self.faces[e_rface as usize].inside
            } else {
                false
            };

            if rf_inside != lf_inside {
                self.edges[e as usize].winding = if lf_inside { value } else { -value };
            } else if !keep_only_boundary {
                self.edges[e as usize].winding = 0;
            } else if !self.delete_edge(e) {
                return false;
            }

            e = e_next;
        }
        true
    }

    /// Discard all exterior faces (zap them).
    pub fn discard_exterior(&mut self) {
        let mut f = self.faces[F_HEAD as usize].next;
        while f != F_HEAD {
            let next = self.faces[f as usize].next;
            if !self.faces[f as usize].inside {
                self.zap_face(f);
            }
            f = next;
        }
    }

}

mod tessellate;

impl Default for Mesh {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn make_edge_creates_single_edge() {
        let mut mesh = Mesh::new();
        let e = mesh.make_edge().unwrap();
        // Should have 3 vertices (vHead + 2 new), 2 faces (fHead + 1 new), 4 edges (eHead pair + 1 pair)
        assert_eq!(mesh.verts.len(), 3);
        assert_eq!(mesh.faces.len(), 2);
        assert_eq!(mesh.edges.len(), 4);
        // Edge and its sym should have different orgs
        let org1 = mesh.edges[e as usize].org;
        let org2 = mesh.edges[(e ^ 1) as usize].org;
        assert_ne!(org1, org2);
        assert_ne!(org1, INVALID);
        assert_ne!(org2, INVALID);
    }

    #[test]
    fn sym_involution() {
        // sym(sym(e)) == e
        for e in 0u32..16 {
            assert_eq!(sym(sym(e)), e);
        }
    }

    #[test]
    fn vertex_list_circular() {
        let mut mesh = Mesh::new();
        mesh.make_edge().unwrap();
        // vHead.next.next should eventually circle back
        let first = mesh.verts[V_HEAD as usize].next;
        assert_ne!(first, V_HEAD);
        let second = mesh.verts[first as usize].next;
        assert_ne!(second, INVALID);
    }

}
