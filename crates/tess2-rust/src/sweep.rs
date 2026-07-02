// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Port of libtess2 sweep.c/h
//
// The Bentley-Ottmann sweep line algorithm for polygon tessellation.
// This module contains the ActiveRegion struct and the sweep computation.
// All logic is driven through the Tessellator in tess.rs.

use crate::dict::NodeIdx;
use crate::mesh::{EdgeIdx, INVALID};

/// An active region: the area between two adjacent edges crossing the sweep line.
#[derive(Clone, Debug, Default)]
pub struct ActiveRegion {
    /// Upper edge (directed right to left).
    pub e_up: EdgeIdx,
    /// Node in the edge dictionary for this region.
    pub node_up: NodeIdx,
    /// Running winding number for the region.
    pub winding_number: i32,
    /// Is this region inside the polygon?
    pub inside: bool,
    /// Sentinel: marks fake edges at t = ±infinity.
    pub sentinel: bool,
    /// Dirty: upper or lower edge changed, need to check for intersection.
    pub dirty: bool,
    /// Temporary edge introduced for a right vertex (will be fixed later).
    pub fix_upper_edge: bool,
}

pub const INVALID_REGION: u32 = INVALID;
