//! rvm2glb core: RVM (PDMS/E3D plant model) → GLB conversion.
//!
//! Platform-agnostic conversion kernel driven by the CLI / wasm / capi shells via
//! [`convert`](fn@convert) + [`ConvertOptions`]/[`ConvertReport`] and the I/O traits in [`io`].

pub mod color_store;
pub mod convert;
pub mod geometry;
pub mod glb_writer;
pub mod instancing;
pub mod io;
pub mod linalg;
pub mod rvm_parser;
pub mod status_writer;
pub mod tessellator;
pub mod triangulation_factory;

pub use convert::{ConvertOptions, ConvertReport, Progress, convert};
pub use io::{InputHandle, MemSink, OutputHandle, OutputSink};
pub use rvm_parser::{OutputMode, RvmParser};
