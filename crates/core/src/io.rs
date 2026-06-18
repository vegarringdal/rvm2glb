//! I/O trait injection for the conversion kernel.
//!
//! All three are **synchronous** on purpose: the CPU-bound core must not become
//! `async`. In the browser the host runs the core in a Web Worker and backs these
//! with OPFS `FileSystemSyncAccessHandle`s, which are synchronous there. The CLI
//! backs them with files; tests/capi/wasm-in-RAM use the `Mem*` impls below.
//!
//! Unlike step2glb there is no `TempHandle`: we split per site (level 0 by default),
//! one site's GLB fits in RAM, so the core builds a site, writes it through an
//! `OutputHandle`, drops it (= close), and moves to the next.

use std::cell::RefCell;
use std::io;
use std::rc::Rc;

/// Random-access, read-only source. Read in small chunks (≤10 MB); never the whole
/// file. `read_at` also serves the COLR last-10MB pre-scan via a tail offset.
pub trait InputHandle: Send + Sync {
    /// Total size of the source in bytes.
    fn size(&self) -> u64;
    /// Read up to `buf.len()` bytes starting at `offset`; returns bytes read (0 at or
    /// past the end). May be short — callers loop when they need a full fill.
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize>;
}

/// A single open output (one GLB, or the status JSON). Closing is `Drop`.
pub trait OutputHandle {
    fn write(&mut self, buf: &[u8]) -> io::Result<()>;
}

/// Factory for outputs: the core requests one handle per exported site/level (plus one
/// for the final `status_file.json`), writes it, and drops it before opening the next.
pub trait OutputSink {
    fn open(&mut self, name: &str) -> io::Result<Box<dyn OutputHandle>>;
}

// ── In-memory input impls ────────────────────────────────────────────────────

fn read_slice(data: &[u8], offset: u64, buf: &mut [u8]) -> usize {
    let off = offset as usize;
    if off >= data.len() {
        return 0;
    }
    let n = buf.len().min(data.len() - off);
    buf[..n].copy_from_slice(&data[off..off + n]);
    n
}

impl InputHandle for Vec<u8> {
    fn size(&self) -> u64 {
        self.len() as u64
    }
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize> {
        Ok(read_slice(self, offset, buf))
    }
}

impl InputHandle for &'static [u8] {
    fn size(&self) -> u64 {
        self.len() as u64
    }
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize> {
        Ok(read_slice(self, offset, buf))
    }
}

// ── In-memory output sink (capi / wasm-in-RAM / tests) ───────────────────────

/// Collects every opened output into `(name, bytes)` pairs in completion order.
#[derive(Default, Clone)]
pub struct MemSink {
    files: Rc<RefCell<Vec<(String, Vec<u8>)>>>,
}

impl MemSink {
    pub fn new() -> Self {
        Self::default()
    }
    /// The collected outputs. Call after the borrowing `convert()` has returned.
    pub fn into_files(self) -> Vec<(String, Vec<u8>)> {
        Rc::try_unwrap(self.files)
            .map(RefCell::into_inner)
            .unwrap_or_else(|rc| rc.borrow().clone())
    }
}

struct MemSinkHandle {
    name: String,
    buf: Vec<u8>,
    files: Rc<RefCell<Vec<(String, Vec<u8>)>>>,
}

impl OutputHandle for MemSinkHandle {
    fn write(&mut self, buf: &[u8]) -> io::Result<()> {
        self.buf.extend_from_slice(buf);
        Ok(())
    }
}

impl Drop for MemSinkHandle {
    fn drop(&mut self) {
        self.files.borrow_mut().push((
            std::mem::take(&mut self.name),
            std::mem::take(&mut self.buf),
        ));
    }
}

impl OutputSink for MemSink {
    fn open(&mut self, name: &str) -> io::Result<Box<dyn OutputHandle>> {
        Ok(Box::new(MemSinkHandle {
            name: name.to_string(),
            buf: Vec::new(),
            files: self.files.clone(),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mem_sink_collects_named_outputs_in_completion_order() {
        let mut sink = MemSink::new();
        {
            let mut a = sink.open("a.glb").unwrap();
            a.write(b"AA").unwrap();
            a.write(b"BB").unwrap(); // dropped (closed) here
        }
        {
            let mut b = sink.open("status_file.json").unwrap();
            b.write(b"{}").unwrap();
        }
        let files = sink.into_files();
        assert_eq!(files.len(), 2);
        assert_eq!(files[0], ("a.glb".to_string(), b"AABB".to_vec()));
        assert_eq!(files[1], ("status_file.json".to_string(), b"{}".to_vec()));
    }

    #[test]
    fn vec_input_read_at_is_short_at_eof() {
        let data: Vec<u8> = (0..10).collect();
        assert_eq!(data.size(), 10);
        let mut buf = [0u8; 4];
        assert_eq!(data.read_at(8, &mut buf).unwrap(), 2); // only 2 bytes left
        assert_eq!(&buf[..2], &[8, 9]);
        assert_eq!(data.read_at(10, &mut buf).unwrap(), 0); // at EOF
    }
}
