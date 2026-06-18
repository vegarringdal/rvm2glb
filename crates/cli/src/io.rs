//! File-backed I/O handles for the CLI (native).

use rvm2glb::{InputHandle, OutputHandle, OutputSink};
use std::fs;
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;

/// Random-access file input. `read_at` uses positional reads (`pread`/`seek_read`) so
/// it needs only `&self` and never holds the whole file in RAM.
pub struct FileInput {
    file: fs::File,
    size: u64,
}

impl FileInput {
    pub fn open(path: &str) -> io::Result<FileInput> {
        let file = fs::File::open(path)?;
        let size = file.metadata()?.len();
        Ok(FileInput { file, size })
    }
}

impl InputHandle for FileInput {
    fn size(&self) -> u64 {
        self.size
    }

    #[cfg(unix)]
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize> {
        use std::os::unix::fs::FileExt;
        self.file.read_at(buf, offset)
    }

    #[cfg(windows)]
    fn read_at(&self, offset: u64, buf: &mut [u8]) -> io::Result<usize> {
        use std::os::windows::fs::FileExt;
        self.file.seek_read(buf, offset)
    }
}

/// Output sink that creates `<dir>/<name>` files (the dir is created on demand).
pub struct DirSink {
    dir: PathBuf,
}

impl DirSink {
    pub fn new(dir: &str) -> DirSink {
        DirSink {
            dir: PathBuf::from(dir),
        }
    }
}

impl OutputSink for DirSink {
    fn open(&mut self, name: &str) -> io::Result<Box<dyn OutputHandle>> {
        fs::create_dir_all(&self.dir)?;
        let f = fs::File::create(self.dir.join(name))?;
        Ok(Box::new(FileSink(BufWriter::new(f))))
    }
}

/// One open file. The buffered writer flushes on drop (= close).
struct FileSink(BufWriter<fs::File>);

impl OutputHandle for FileSink {
    fn write(&mut self, buf: &[u8]) -> io::Result<()> {
        self.0.write_all(buf)
    }
}
