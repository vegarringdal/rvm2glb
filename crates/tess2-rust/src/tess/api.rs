// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// High-level public wrapper around `Tessellator`.  Kept as a thin
// pass-through so the algorithm core in `mod.rs` stays focused on
// sweep / mesh state and so the file-size compliance test
// (`tests/file_compliance.rs`) keeps `tess/mod.rs` under its limit.

use crate::geom::Real;

use super::{ElementType, TessOption, TessStatus, Tessellator, WindingRule};

/// High-level tessellator (public interface).
pub struct TessellatorApi {
    inner: Tessellator,
}

impl TessellatorApi {
    pub fn new() -> Self {
        TessellatorApi {
            inner: Tessellator::new(),
        }
    }
    pub fn set_option(&mut self, option: TessOption, value: bool) {
        self.inner.set_option(option, value);
    }
    pub fn add_contour(&mut self, size: usize, vertices: &[Real]) {
        self.inner.add_contour(size, vertices);
    }
    pub fn tessellate(
        &mut self,
        winding_rule: WindingRule,
        element_type: ElementType,
        poly_size: usize,
        vertex_size: usize,
        normal: Option<[Real; 3]>,
    ) -> bool {
        self.inner
            .tessellate(winding_rule, element_type, poly_size, vertex_size, normal)
    }
    pub fn vertex_count(&self) -> usize {
        self.inner.vertex_count()
    }
    pub fn element_count(&self) -> usize {
        self.inner.element_count()
    }
    pub fn vertices(&self) -> &[Real] {
        self.inner.vertices()
    }
    pub fn vertex_indices(&self) -> &[u32] {
        self.inner.vertex_indices()
    }
    pub fn elements(&self) -> &[u32] {
        self.inner.elements()
    }
    /// Per triangle-vertex edge flags — see [`Tessellator::out_edge_flags`].
    pub fn edge_flags(&self) -> &[u8] {
        self.inner.edge_flags()
    }
    pub fn status(&self) -> TessStatus {
        self.inner.get_status()
    }
}

impl Default for TessellatorApi {
    fn default() -> Self {
        Self::new()
    }
}
