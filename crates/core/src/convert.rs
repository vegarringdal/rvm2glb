//! Public conversion entry point: `convert(input, sink, opts, progress)`.
//!
//! The kernel reads from an [`InputHandle`] in small chunks and emits one GLB per
//! exported site/level (plus a final `status_file.json`) through an [`OutputSink`],
//! reporting per-site [`Progress`].

use crate::io::{InputHandle, OutputSink};
use crate::rvm_parser::{FileMeta, HeaderBlock, OutputMode, RvmParser};

/// Conversion knobs (the CLI flags, minus the I/O paths).
#[derive(Debug, Clone)]
pub struct ConvertOptions {
    /// Hierarchy depth at which to split into separate GLB files (0 = site).
    pub level: u8,
    /// Drop nodes/branches with no geometry.
    pub remove_empty: bool,
    /// Weld coincident vertices per item.
    pub cleanup_position: bool,
    /// Decimal places for the vertex-weld grid.
    pub cleanup_precision: u8,
    /// meshopt simplification ratio (1.0 disables; merged path).
    pub meshopt_threshold: f32,
    /// meshopt simplification target error.
    pub meshopt_target_error: f32,
    /// Tessellation chord-height tolerance.
    pub tolerance: f32,
    /// merged / instanced / standard.
    pub mode: OutputMode,
    /// Width of the "+" cross drawn for RVM Line primitives.
    pub line_width: f32,
    /// Round circle tessellation up to a multiple of 4 segments.
    pub align_segments: bool,
    /// Instanced debug colouring (shared shapes yellow, one-offs grey).
    pub highlight_instance: bool,
    /// Parse only; do not open any output.
    pub dry_run: bool,
    /// Original input name recorded in `status_file.json` (the source RVM basename).
    pub source_name: String,
}

impl Default for ConvertOptions {
    fn default() -> Self {
        Self {
            level: 0,
            remove_empty: true,
            cleanup_position: true,
            cleanup_precision: 3,
            meshopt_threshold: 0.75,
            meshopt_target_error: 0.0,
            tolerance: 0.01,
            mode: OutputMode::Merged,
            line_width: 0.05,
            align_segments: false,
            highlight_instance: false,
            dry_run: false,
            source_name: String::new(),
        }
    }
}

/// Per-site progress, fired once as each output GLB is written. The per-input-file
/// granularity (file *i* of *N*) lives in the batch/wasm driver, since `convert`
/// handles one input at a time.
#[derive(Debug, Clone, Copy)]
pub struct Progress<'a> {
    /// 0-based index of this output among the GLBs written by this call.
    pub output_index: u32,
    /// The file name written (e.g. `"HA-PIPE.glb"`).
    pub output_name: &'a str,
    /// Number of nodes in this site.
    pub nodes: u32,
}

/// Outcome of a conversion: the per-output metadata (also serialised to
/// `status_file.json`), parser warnings, the RVM header, and any COLR overrides found.
#[derive(Debug, Default, Clone)]
pub struct ConvertReport {
    pub filemeta: Vec<FileMeta>,
    pub warnings: Vec<String>,
    pub header: HeaderBlock,
    /// COLR `(index, rgb)` overrides loaded by the pre-scan (empty for most files).
    pub color_overrides: Vec<(u32, u32)>,
}

/// Convert one RVM input, emitting GLB(s) + `status_file.json` through `sink`.
pub fn convert(
    input: Box<dyn InputHandle>,
    sink: &mut dyn OutputSink,
    opts: &ConvertOptions,
    progress: &mut dyn FnMut(&Progress),
) -> Result<ConvertReport, String> {
    let mut parser = RvmParser::new(opts);
    parser
        .run(input, sink, progress)
        .map_err(|e| e.to_string())?;
    Ok(parser.into_report())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::MemSink;

    // End-to-end through the in-RAM path (Vec<u8> input → MemSink) — the shape wasm and
    // capi use. A trivially-short buffer parses to nothing, so the only output is the
    // status JSON, proving the open→write→close + convert wiring without a real RVM.
    #[test]
    fn convert_in_ram_emits_status_json() {
        let input: Vec<u8> = vec![0u8; 8];
        let mut sink = MemSink::new();
        let mut fired = 0;
        let report = convert(
            Box::new(input),
            &mut sink,
            &ConvertOptions::default(),
            &mut |_p| fired += 1,
        )
        .unwrap();
        assert!(report.filemeta.is_empty());
        assert_eq!(fired, 0); // no GLB sites, so no per-site progress
        let files = sink.into_files();
        assert_eq!(files.len(), 1);
        assert_eq!(files[0].0, "status_file.json");
        assert!(files[0].1.starts_with(b"{"));
    }

    #[test]
    fn convert_dry_run_writes_nothing() {
        let input: Vec<u8> = vec![0u8; 8];
        let mut sink = MemSink::new();
        convert(
            Box::new(input),
            &mut sink,
            &ConvertOptions {
                dry_run: true,
                ..ConvertOptions::default()
            },
            &mut |_p| {},
        )
        .unwrap();
        assert!(sink.into_files().is_empty());
    }
}
