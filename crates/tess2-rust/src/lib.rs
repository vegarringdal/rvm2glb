// tess2-rust: Pure Rust port of libtess2 (SGI tessellation library)
// Copyright 2025 Lars Brubaker
// License: SGI Free Software License B (MIT-compatible)
//
// Vendored fork for step2glb — the sweep's region accessors fail soft instead
// of panicking (see src/tess/region.rs "STEP2GLB PATCH"). Upstream lints are
// silenced so this vendored copy doesn't add warnings to the workspace build.
#![allow(warnings, clippy::all)]

pub mod bucketalloc;
pub mod dict;
pub mod geom;
pub mod mesh;
pub mod priorityq;
pub mod sweep;
pub mod tess;

pub use tess::{ElementType, TessOption, TessStatus, Tessellator, TessellatorApi, WindingRule};
