//! Thin per-primitive tessellation entry point.
//!
//! [`Tessellator`] bundles the tessellation knobs (chord-height `tolerance`, Line
//! `line_width`, `align_segments`) and hands one [`Geometry`] to
//! [`triangulation_factory::tessellate`](crate::triangulation_factory::tessellate),
//! which does the actual work. The per-primitive scale is taken from the geometry's
//! accumulated `m_3x4` so circle segment counts adapt to size.

use crate::geometry::{Connection, Geometry, Triangulation};
use crate::triangulation_factory::tessellate;

/// Tessellation parameters, applied to each primitive via [`Tessellator::geometry`].
pub struct Tessellator {
    pub tolerance: f32,
    pub line_width: f32,
    pub align_segments: bool,
}

impl Tessellator {
    pub fn new(tolerance: f32, line_width: f32, align_segments: bool) -> Self {
        Self {
            tolerance,
            line_width,
            align_segments,
        }
    }

    /// Tessellate one primitive (`geo`); `geo_all`/`connections` give neighbour context
    /// for cap/adjacency handling. Returns `None` when the primitive yields no geometry.
    pub fn geometry(
        &self,
        geo: &Geometry,
        geo_all: &[Geometry],
        connections: &[Connection],
    ) -> Option<Triangulation> {
        tessellate(
            geo,
            geo_all,
            connections,
            self.tolerance,
            self.line_width,
            geo.m_3x4.get_scale(),
            self.align_segments,
        )
    }
}
