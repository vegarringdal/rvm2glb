//! WebAssembly shell for rvm2glb.
//!
//! Two entry points, mirroring step2glb but with our 3-way `mode` and N-outputs model:
//!  - [`convert_in_ram`] — input bytes in, every output (GLBs + `status_file.json`)
//!    collected in RAM and handed back via [`ConvertResult`]. For files that fit in RAM.
//!  - [`convert_streaming`] — input + output stream through a JS [`Io`] object backed by
//!    OPFS sync access handles in a Worker (`size`/`read` for input; `open`/`write`/
//!    `close` per output file; `progress` per GLB). No whole-file buffering.
//!
//! meshopt is compiled out here (core built `--no-default-features`).

use rvm2glb::{ConvertOptions, InputHandle, OutputHandle, OutputSink, Progress, convert};
use std::io;
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(start)]
pub fn start() {
    console_error_panic_hook::set_once();
}

/// Crate version string.
#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

// Freestanding `wasm32-unknown-unknown` has no C++ runtime, so meshoptimizer's C++
// `operator new`/`delete` (`_Znwm`/`_ZdlPv`/…) are undefined at link time. Provide
// minimal versions backed by Rust's global allocator. Only on wasm + `optimize` — on
// native, libstdc++/libc++ already defines these (defining ours would clash).
#[cfg(all(feature = "optimize", target_arch = "wasm32"))]
mod cxx_alloc {
    use std::alloc::{Layout, alloc, dealloc};

    // Stash the allocation size in a 16-byte header so sizeless `operator delete` can
    // reconstruct the Layout; 16 also keeps the returned pointer suitably aligned.
    const HDR: usize = 16;

    unsafe fn cxx_new(size: usize) -> *mut u8 {
        let layout = Layout::from_size_align(size + HDR, HDR).unwrap();
        // SAFETY: layout has non-zero size (size + HDR >= HDR).
        let base = unsafe { alloc(layout) };
        if base.is_null() {
            return base;
        }
        unsafe {
            (base as *mut usize).write(size);
            base.add(HDR)
        }
    }

    unsafe fn cxx_delete(ptr: *mut u8) {
        if ptr.is_null() {
            return;
        }
        unsafe {
            let base = ptr.sub(HDR);
            let size = (base as *const usize).read();
            dealloc(base, Layout::from_size_align(size + HDR, HDR).unwrap());
        }
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn _Znwm(size: usize) -> *mut u8 {
        unsafe { cxx_new(size) }
    }
    #[unsafe(no_mangle)]
    pub extern "C" fn _Znam(size: usize) -> *mut u8 {
        unsafe { cxx_new(size) }
    }
    #[unsafe(no_mangle)]
    pub extern "C" fn _ZdlPv(ptr: *mut u8) {
        unsafe { cxx_delete(ptr) }
    }
    #[unsafe(no_mangle)]
    pub extern "C" fn _ZdaPv(ptr: *mut u8) {
        unsafe { cxx_delete(ptr) }
    }
    #[unsafe(no_mangle)]
    pub extern "C" fn _ZdlPvm(ptr: *mut u8, _size: usize) {
        unsafe { cxx_delete(ptr) }
    }
    #[unsafe(no_mangle)]
    pub extern "C" fn _ZdaPvm(ptr: *mut u8, _size: usize) {
        unsafe { cxx_delete(ptr) }
    }
}

// ── Options (a JS class; mode: 0 merged, 1 instanced, 2 standard) ────────────

#[wasm_bindgen]
#[derive(Clone, Copy)]
pub struct Options {
    pub level: u8,
    pub mode: u8,
    pub remove_empty: bool,
    pub cleanup_position: bool,
    pub cleanup_precision: u8,
    pub meshopt_threshold: f32,
    pub meshopt_target_error: f32,
    pub tolerance: f32,
    pub line_width: f32,
    pub align_segments: bool,
    pub highlight_instance: bool,
    pub dry_run: bool,
}

#[wasm_bindgen]
impl Options {
    /// Defaults match the CLI (merged, remove-empty on, weld on, tolerance 0.01, …).
    #[wasm_bindgen(constructor)]
    pub fn new() -> Options {
        Options {
            level: 0,
            mode: 0,
            remove_empty: true,
            cleanup_position: true,
            cleanup_precision: 3,
            meshopt_threshold: 0.75,
            meshopt_target_error: 0.0,
            tolerance: 0.01,
            line_width: 0.05,
            align_segments: false,
            highlight_instance: false,
            dry_run: false,
        }
    }
}

impl Default for Options {
    fn default() -> Self {
        Options::new()
    }
}

fn to_core(o: &Options, source_name: String) -> ConvertOptions {
    ConvertOptions {
        level: o.level,
        remove_empty: o.remove_empty,
        cleanup_position: o.cleanup_position,
        cleanup_precision: o.cleanup_precision,
        meshopt_threshold: o.meshopt_threshold,
        meshopt_target_error: o.meshopt_target_error,
        tolerance: o.tolerance,
        mode: match o.mode {
            1 => rvm2glb::OutputMode::Instanced,
            2 => rvm2glb::OutputMode::Standard,
            _ => rvm2glb::OutputMode::Merged,
        },
        line_width: o.line_width,
        align_segments: o.align_segments,
        highlight_instance: o.highlight_instance,
        dry_run: o.dry_run,
        source_name,
    }
}

// ── JS-provided objects we call into ─────────────────────────────────────────

#[wasm_bindgen]
extern "C" {
    /// Per-GLB progress callback (in-RAM path).
    pub type ProgressSink;
    #[wasm_bindgen(method)]
    fn report(this: &ProgressSink, output_index: u32, name: String, nodes: u32);

    /// Streaming I/O backed by OPFS sync access handles (in a Worker).
    pub type Io;
    #[wasm_bindgen(method)]
    fn size(this: &Io) -> f64;
    /// Read up to `len` bytes at `offset`; returns the bytes (may be shorter at EOF).
    #[wasm_bindgen(method)]
    fn read(this: &Io, offset: f64, len: f64) -> Vec<u8>;
    /// Open output `name`; returns a nonzero handle id, or 0 on failure.
    #[wasm_bindgen(method)]
    fn open(this: &Io, name: String) -> f64;
    #[wasm_bindgen(method)]
    fn write(this: &Io, handle: f64, bytes: &[u8]);
    #[wasm_bindgen(method)]
    fn close(this: &Io, handle: f64);
    #[wasm_bindgen(method)]
    fn progress(this: &Io, output_index: u32, name: String, nodes: u32);
}

// ── In-RAM path ──────────────────────────────────────────────────────────────

/// Every output of one conversion (GLBs + `status_file.json`), held in RAM.
#[wasm_bindgen]
pub struct ConvertResult {
    files: Vec<(String, Vec<u8>)>,
    info: String,
}

#[wasm_bindgen]
impl ConvertResult {
    /// Number of files produced.
    #[wasm_bindgen(getter)]
    pub fn len(&self) -> usize {
        self.files.len()
    }
    /// Name of file `i` (e.g. `"HA-PIPE.glb"`, `"status_file.json"`).
    pub fn name(&self, i: usize) -> Option<String> {
        self.files.get(i).map(|f| f.0.clone())
    }
    /// Bytes of file `i`.
    pub fn bytes(&self, i: usize) -> Option<Vec<u8>> {
        self.files.get(i).map(|f| f.1.clone())
    }
    /// Small JSON summary (`{"files":N,"warnings":M}`).
    #[wasm_bindgen(getter)]
    pub fn info(&self) -> String {
        self.info.clone()
    }
}

/// Convert RVM bytes entirely in memory; returns all outputs in a [`ConvertResult`].
#[wasm_bindgen]
pub fn convert_in_ram(
    input: Vec<u8>,
    opts: &Options,
    source_name: String,
    progress: &ProgressSink,
) -> Result<ConvertResult, JsValue> {
    let mut sink = rvm2glb::MemSink::new();
    let report = convert(
        Box::new(input),
        &mut sink,
        &to_core(opts, source_name),
        &mut |p: &Progress| progress.report(p.output_index, p.output_name.to_string(), p.nodes),
    )
    .map_err(|e| JsValue::from_str(&e))?;
    let info = format!(
        "{{\"files\":{},\"warnings\":{}}}",
        report.filemeta.len(),
        report.warnings.len()
    );
    Ok(ConvertResult {
        files: sink.into_files(),
        info,
    })
}

// ── Streaming (OPFS) path ────────────────────────────────────────────────────

struct IoInput(Io);
// wasm32 is single-threaded; the JS handle never crosses threads.
unsafe impl Send for IoInput {}
unsafe impl Sync for IoInput {}

impl InputHandle for IoInput {
    fn size(&self) -> u64 {
        self.0.size() as u64
    }
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize> {
        let data = self.0.read(offset as f64, buf.len() as f64);
        let n = data.len().min(buf.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }
}

struct IoSink(Io);

impl OutputSink for IoSink {
    fn open(&mut self, name: &str) -> io::Result<Box<dyn OutputHandle>> {
        let handle = self.0.open(name.to_string());
        if handle == 0.0 {
            return Err(io::Error::other("open callback failed"));
        }
        Ok(Box::new(IoHandle {
            io: self.0.clone().unchecked_into(),
            handle,
        }))
    }
}

struct IoHandle {
    io: Io,
    handle: f64,
}

impl OutputHandle for IoHandle {
    fn write(&mut self, buf: &[u8]) -> io::Result<()> {
        self.io.write(self.handle, buf);
        Ok(())
    }
}

impl Drop for IoHandle {
    fn drop(&mut self) {
        self.io.close(self.handle);
    }
}

/// Stream a conversion through a JS `Io` (OPFS-backed). Returns a small JSON summary.
#[wasm_bindgen]
pub fn convert_streaming(io: Io, opts: &Options, source_name: String) -> Result<String, JsValue> {
    let input = Box::new(IoInput(io.clone().unchecked_into()));
    let mut sink = IoSink(io.clone().unchecked_into());
    let prog: Io = io.clone().unchecked_into();
    let report = convert(
        input,
        &mut sink,
        &to_core(opts, source_name),
        &mut |p: &Progress| prog.progress(p.output_index, p.output_name.to_string(), p.nodes),
    )
    .map_err(|e| JsValue::from_str(&e))?;
    Ok(format!(
        "{{\"files\":{},\"warnings\":{}}}",
        report.filemeta.len(),
        report.warnings.len()
    ))
}
