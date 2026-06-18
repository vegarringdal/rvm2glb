mod io;

use clap::{Parser, ValueEnum};
use io::{DirSink, FileInput};
use rvm2glb::{ConvertOptions, OutputMode, convert};

/// Output mode.
#[derive(ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    /// One merged mesh per colour (geometry baked to world space).
    Merged,
    /// Node-per-instance: one mesh per unique shape, one node per occurrence.
    Instanced,
    /// One mesh + node per component, no merge and no instancing (plain glTF).
    Standard,
}

impl From<Mode> for OutputMode {
    fn from(m: Mode) -> Self {
        match m {
            Mode::Merged => OutputMode::Merged,
            Mode::Instanced => OutputMode::Instanced,
            Mode::Standard => OutputMode::Standard,
        }
    }
}

/// RVM to GLB converter (merged-per-colour, instanced, or standard)
#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// RVM file input path
    #[arg(short = 'i', long = "input", required = true)]
    input: String,

    /// Output folder (created if missing)
    #[arg(short = 'o', long = "output", default_value = "./exports/")]
    output: String,

    /// Dry run: parse only, do not write files
    #[arg(short = 'x', long = "dry-run", default_value_t = false)]
    dry_run: bool,

    /// Hierarchy level at which to split output files (0 = site)
    #[arg(short = 'l', long = "level", default_value_t = 0)]
    level: u8,

    /// Remove elements that have no geometry (default on; disable with `-r 0`/`false`)
    #[arg(short = 'r', long = "remove-empty", default_value_t = true,
          action = clap::ArgAction::Set, value_parser = clap::builder::BoolishValueParser::new())]
    remove_empty: bool,

    /// Remove duplicate vertex positions per item (default on; disable with `-d 0`)
    #[arg(short = 'd', long = "cleanup-position", default_value_t = true,
          action = clap::ArgAction::Set, value_parser = clap::builder::BoolishValueParser::new())]
    cleanup_position: bool,

    /// Rounding precision used when deduplicating positions
    #[arg(short = 'p', long = "cleanup-precision", default_value_t = 3)]
    cleanup_precision: u8,

    /// meshopt simplification threshold (fraction of indices to keep)
    #[arg(short = 'm', long = "meshopt-threshold", default_value_t = 0.75)]
    meshopt_threshold: f32,

    /// meshopt target error
    #[arg(short = 'e', long = "meshopt-target-error", default_value_t = 0.0)]
    meshopt_target_error: f32,

    /// Tessellation tolerance
    #[arg(short = 't', long = "tolerance", default_value_t = 0.01)]
    tolerance: f32,

    /// Output mode: merged (one mesh per colour) or instanced (node per occurrence)
    #[arg(long = "mode", value_enum, default_value_t = Mode::Merged)]
    mode: Mode,

    /// Width of the "+" cross drawn for RVM Line primitives (model units)
    #[arg(long = "line-width", default_value_t = 0.05)]
    line_width: f32,

    /// Round circle tessellation up to a multiple of 4 segments so adjacent
    /// primitives share boundary vertices (better flat-shading alignment, ~25%
    /// larger output). Off by default (raw chord-height count, matches the C++ tool)
    #[arg(long = "align-segments", default_value_t = false)]
    align_segments: bool,

    /// Debug colouring for instanced mode: shapes shared by ≥2 occurrences render
    /// yellow, one-offs grey — so you can see what actually got instanced.
    #[arg(long = "highlight-instance", default_value_t = false)]
    highlight_instance: bool,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let source_name = std::path::Path::new(&args.input)
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| args.input.clone());

    let opts = ConvertOptions {
        level: args.level,
        remove_empty: args.remove_empty,
        cleanup_position: args.cleanup_position,
        cleanup_precision: args.cleanup_precision,
        meshopt_threshold: args.meshopt_threshold,
        meshopt_target_error: args.meshopt_target_error,
        tolerance: args.tolerance,
        mode: args.mode.into(),
        line_width: args.line_width,
        align_segments: args.align_segments,
        highlight_instance: args.highlight_instance,
        dry_run: args.dry_run,
        source_name,
    };

    println!("Reading: {}", args.input);
    let input = FileInput::open(&args.input)?;
    let mut sink = DirSink::new(&args.output);
    let start = std::time::Instant::now();

    let dir = args.output.clone();
    let report = convert(Box::new(input), &mut sink, &opts, &mut |p| {
        println!("File created: {}{}", dir, p.output_name);
    })
    .map_err(anyhow::Error::msg)?;

    for &(index, rgb) in &report.color_overrides {
        println!(
            "Found color index: {} \tR: {} \tG: {} \tB: {}",
            index,
            (rgb >> 16) & 0xff,
            (rgb >> 8) & 0xff,
            rgb & 0xff
        );
    }
    for w in &report.warnings {
        eprintln!("warning: {w}");
    }
    if args.dry_run {
        println!("[dry-run] parsed only; no files written");
    }
    println!("Done in {:.2}s", start.elapsed().as_secs_f64());
    Ok(())
}
