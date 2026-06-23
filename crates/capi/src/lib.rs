//! C ABI for rvm2glb.
//!
//! Streaming + progress, driven by C function pointers (richer than step2glb's in-RAM
//! capi). The host supplies a positional `read` for input and an `open`/`write`/`close`
//! trio backing the per-level output sink (one GLB per site + the final
//! `status_file.json`); `progress` fires once per GLB. All callbacks receive the opaque
//! `user` pointer verbatim. See `include/rvm2glb.h`.

use core::ffi::{c_char, c_int, c_void};
use rvm2glb::{ConvertOptions, InputHandle, OutputHandle, OutputSink, Progress, convert};
use std::ffi::{CStr, CString};

/// Read up to `len` bytes at `offset` into `buf`; return bytes read (0 at EOF, <0 error).
pub type ReadFn = extern "C" fn(user: *mut c_void, offset: u64, buf: *mut u8, len: usize) -> i64;
/// Open output `name`; return a nonzero handle, or 0 on failure.
pub type OpenFn = extern "C" fn(user: *mut c_void, name: *const c_char) -> u64;
/// Append `len` bytes to `handle`; return 0 on success.
pub type WriteFn =
    extern "C" fn(user: *mut c_void, handle: u64, buf: *const u8, len: usize) -> c_int;
/// Close `handle` (no more writes follow).
pub type CloseFn = extern "C" fn(user: *mut c_void, handle: u64);
/// Per-GLB progress: `name` is the file just written, `nodes` its node count.
pub type ProgressFn =
    extern "C" fn(user: *mut c_void, output_index: u32, name: *const c_char, nodes: u32);

/// Conversion options (mirrors core's `ConvertOptions`). `mode`: 0 merged, 1 instanced,
/// 2 standard. `source_name` may be null.
#[repr(C)]
pub struct Rvm2GlbOptions {
    pub level: u8,
    pub mode: c_int,
    pub remove_empty: bool,
    pub cleanup_position: bool,
    pub cleanup_precision: u8,
    pub meshopt_threshold: f32,
    pub meshopt_target_error: f32,
    pub tolerance: f32,
    pub line_width: f32,
    /// Include RVM Line primitives. `false` (the default) skips them entirely —
    /// they are numerous and add visual noise.
    pub include_line: bool,
    pub align_segments: bool,
    pub highlight_instance: bool,
    pub dry_run: bool,
    /// Extract the RVM structure as JSON (`<site>.json` + `base.json`) instead of GLB.
    /// Overrides `mode`; honours `level`.
    pub extract_json: bool,
    pub source_name: *const c_char,
}

// ── trait adapters over the C callbacks ──────────────────────────────────────

struct CInput {
    user: *mut c_void,
    size: u64,
    read: ReadFn,
}
// The host guarantees single-threaded use (one worker / one thread).
unsafe impl Send for CInput {}
unsafe impl Sync for CInput {}

impl InputHandle for CInput {
    fn size(&self) -> u64 {
        self.size
    }
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = (self.read)(self.user, offset, buf.as_mut_ptr(), buf.len());
        if n < 0 {
            Err(std::io::Error::other("input read callback failed"))
        } else {
            Ok((n as usize).min(buf.len()))
        }
    }
}

struct CSink {
    user: *mut c_void,
    open: OpenFn,
    write: WriteFn,
    close: CloseFn,
}

impl OutputSink for CSink {
    fn open(&mut self, name: &str) -> std::io::Result<Box<dyn OutputHandle>> {
        let cname = CString::new(name).map_err(|_| std::io::Error::other("output name has NUL"))?;
        let handle = (self.open)(self.user, cname.as_ptr());
        if handle == 0 {
            return Err(std::io::Error::other("open callback failed"));
        }
        Ok(Box::new(CHandle {
            user: self.user,
            handle,
            write: self.write,
            close: self.close,
        }))
    }
}

struct CHandle {
    user: *mut c_void,
    handle: u64,
    write: WriteFn,
    close: CloseFn,
}

impl OutputHandle for CHandle {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<()> {
        let rc = (self.write)(self.user, self.handle, buf.as_ptr(), buf.len());
        if rc != 0 {
            Err(std::io::Error::other("write callback failed"))
        } else {
            Ok(())
        }
    }
}

impl Drop for CHandle {
    fn drop(&mut self) {
        (self.close)(self.user, self.handle);
    }
}

/// Convert one RVM input. Returns 0 on success, 1 for bad arguments (null callback /
/// options / unknown mode), 2 if the conversion itself failed. `progress` may be null.
///
/// # Safety
/// `opts` must point to a valid `Rvm2GlbOptions` (with `source_name` null or a valid
/// C string); the callbacks must be valid for the duration of the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rvm2glb_convert(
    user: *mut c_void,
    input_size: u64,
    read: Option<ReadFn>,
    open: Option<OpenFn>,
    write: Option<WriteFn>,
    close: Option<CloseFn>,
    progress: Option<ProgressFn>,
    opts: *const Rvm2GlbOptions,
) -> c_int {
    let (Some(read), Some(open), Some(write), Some(close)) = (read, open, write, close) else {
        return 1;
    };
    if opts.is_null() {
        return 1;
    }
    let o = unsafe { &*opts };
    let mode = match o.mode {
        0 => rvm2glb::OutputMode::Merged,
        1 => rvm2glb::OutputMode::Instanced,
        2 => rvm2glb::OutputMode::Standard,
        _ => return 1,
    };
    let source_name = if o.source_name.is_null() {
        String::new()
    } else {
        unsafe { CStr::from_ptr(o.source_name) }
            .to_string_lossy()
            .into_owned()
    };

    let options = ConvertOptions {
        level: o.level,
        remove_empty: o.remove_empty,
        cleanup_position: o.cleanup_position,
        cleanup_precision: o.cleanup_precision,
        meshopt_threshold: o.meshopt_threshold,
        meshopt_target_error: o.meshopt_target_error,
        tolerance: o.tolerance,
        mode,
        line_width: o.line_width,
        include_line: o.include_line,
        align_segments: o.align_segments,
        highlight_instance: o.highlight_instance,
        dry_run: o.dry_run,
        extract_json: o.extract_json,
        source_name,
    };

    let input = Box::new(CInput {
        user,
        size: input_size,
        read,
    });
    let mut sink = CSink {
        user,
        open,
        write,
        close,
    };
    let mut prog = |p: &Progress| {
        if let Some(f) = progress
            && let Ok(cname) = CString::new(p.output_name)
        {
            f(user, p.output_index, cname.as_ptr(), p.nodes);
        }
    };

    match convert(input, &mut sink, &options, &mut prog) {
        Ok(_) => 0,
        Err(_) => 2,
    }
}

/// ABI version of this library (bump on any breaking change to the C surface).
#[unsafe(no_mangle)]
pub extern "C" fn rvm2glb_capi_abi_version() -> u32 {
    // 2: added `extract_json` to rvm2glb_options (struct layout change).
    // 3: added `include_line` to rvm2glb_options (struct layout change).
    3
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::collections::HashMap;

    // A Rust stand-in for a C host: an input buffer + the files it "wrote".
    struct Host {
        input: Vec<u8>,
        files: Vec<(String, Vec<u8>)>,
        open: HashMap<u64, (String, Vec<u8>)>,
        next: u64,
    }

    extern "C" fn read_cb(user: *mut c_void, offset: u64, buf: *mut u8, len: usize) -> i64 {
        let h = unsafe { &*(user as *const RefCell<Host>) }.borrow();
        let off = offset as usize;
        if off >= h.input.len() {
            return 0;
        }
        let n = len.min(h.input.len() - off);
        unsafe { std::ptr::copy_nonoverlapping(h.input[off..].as_ptr(), buf, n) };
        n as i64
    }
    extern "C" fn open_cb(user: *mut c_void, name: *const c_char) -> u64 {
        let mut h = unsafe { &*(user as *const RefCell<Host>) }.borrow_mut();
        h.next += 1;
        let id = h.next;
        let name = unsafe { CStr::from_ptr(name) }
            .to_string_lossy()
            .into_owned();
        h.open.insert(id, (name, Vec::new()));
        id
    }
    extern "C" fn write_cb(user: *mut c_void, handle: u64, buf: *const u8, len: usize) -> c_int {
        let mut h = unsafe { &*(user as *const RefCell<Host>) }.borrow_mut();
        let slice = unsafe { std::slice::from_raw_parts(buf, len) };
        match h.open.get_mut(&handle) {
            Some(f) => {
                f.1.extend_from_slice(slice);
                0
            }
            None => 1,
        }
    }
    extern "C" fn close_cb(user: *mut c_void, handle: u64) {
        let mut h = unsafe { &*(user as *const RefCell<Host>) }.borrow_mut();
        if let Some(f) = h.open.remove(&handle) {
            h.files.push(f);
        }
    }

    fn default_opts() -> Rvm2GlbOptions {
        Rvm2GlbOptions {
            level: 0,
            mode: 0,
            remove_empty: true,
            cleanup_position: true,
            cleanup_precision: 3,
            meshopt_threshold: 0.75,
            meshopt_target_error: 0.0,
            tolerance: 0.01,
            line_width: 0.005,
            include_line: false,
            align_segments: false,
            highlight_instance: false,
            dry_run: false,
            extract_json: false,
            source_name: std::ptr::null(),
        }
    }

    // Full C-ABI round trip over the streaming callbacks: a trivially-short input parses
    // to nothing, so the only output is status_file.json through open→write→close.
    #[test]
    fn capi_round_trip_emits_status_json() {
        let host = RefCell::new(Host {
            input: vec![0u8; 8],
            files: Vec::new(),
            open: HashMap::new(),
            next: 0,
        });
        let user = &host as *const RefCell<Host> as *mut c_void;
        let opts = default_opts();
        let rc = unsafe {
            rvm2glb_convert(
                user,
                8,
                Some(read_cb),
                Some(open_cb),
                Some(write_cb),
                Some(close_cb),
                None,
                &opts,
            )
        };
        assert_eq!(rc, 0);
        let files = &host.borrow().files;
        assert_eq!(files.len(), 1);
        assert_eq!(files[0].0, "status_file.json");
        assert!(files[0].1.starts_with(b"{"));
    }

    #[test]
    fn capi_rejects_null_callback_and_bad_mode() {
        let host = RefCell::new(Host {
            input: vec![0u8; 8],
            files: Vec::new(),
            open: HashMap::new(),
            next: 0,
        });
        let user = &host as *const RefCell<Host> as *mut c_void;
        let opts = default_opts();
        // missing read callback → 1
        let rc = unsafe {
            rvm2glb_convert(
                user,
                8,
                None,
                Some(open_cb),
                Some(write_cb),
                Some(close_cb),
                None,
                &opts,
            )
        };
        assert_eq!(rc, 1);
        // unknown mode → 1
        let bad = Rvm2GlbOptions {
            mode: 9,
            ..default_opts()
        };
        let rc = unsafe {
            rvm2glb_convert(
                user,
                8,
                Some(read_cb),
                Some(open_cb),
                Some(write_cb),
                Some(close_cb),
                None,
                &bad,
            )
        };
        assert_eq!(rc, 1);
    }
}
