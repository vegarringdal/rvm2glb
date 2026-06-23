//! RVM binary parser + per-site export driver.
//!
//! Walks the RVM chunk stream (HEAD/MODL/CNTB/CNTE/PRIM/OBST/INSU/COLR/END:) building a
//! `MetaNode` hierarchy, tessellating each primitive, and flushing one export "root" per
//! container at the split level. [`RvmParser::run`] reads through an `InputHandle` and
//! emits each GLB + the final `status_file.json` through an `OutputSink` (see
//! the `crate::convert` module for the public `convert()` wrapper). Data types `MetaNode`,
//! `NodePrim`, `FileMeta`, `HeaderBlock`, and `OutputMode` live here.

use md5::Context as Md5Context;
use std::collections::{HashMap, HashSet};

use crate::color_store::ColorStore;
use crate::convert::{ConvertOptions, ConvertReport, Progress};
use crate::geometry::*;
use crate::glb_writer::{Cleanup, GlbWriter};
use crate::instancing::shape_key;
use crate::io::{InputHandle, OutputSink};
use crate::json_writer::{base_json, site_json};
use crate::linalg::{BBox3f, Mat3x4f, Vec3f, transform_bbox};
use crate::status_writer::status_json;
use crate::tessellator::Tessellator;

// ─── Public data types ────────────────────────────────────────────────────

/// Output layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    /// One merged mesh per colour (geometry baked); web3d extras.
    Merged,
    /// Node-per-instance with shared meshes (shape dedup); plain glTF.
    Instanced,
    /// One mesh + node per component, no merge, no dedup; plain glTF.
    Standard,
}

#[derive(Debug, Default, Clone)]
pub struct BBox3 {
    pub max_x: f32,
    pub max_y: f32,
    pub max_z: f32,
    pub min_x: f32,
    pub min_y: f32,
    pub min_z: f32,
}

impl BBox3 {
    pub fn new() -> Self {
        Self {
            min_x: f32::MAX,
            min_y: f32::MAX,
            min_z: f32::MAX,
            max_x: f32::MIN,
            max_y: f32::MIN,
            max_z: f32::MIN,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct FileMeta {
    pub root_name: String,
    pub file_name: String,
    /// Source RVM file name (basename).
    pub source_file_name: String,
    /// MD5 of the RVM input stream consumed up to this root.
    pub md5: String,
    /// MD5 of the GLB bytes we wrote for this root.
    pub glb_md5: String,
    /// The `--level` split depth this export used.
    pub export_lvl: u8,
    /// Names of the containers above this root (top-down). Empty when `--level 0`.
    pub parent: Vec<String>,
    /// Stable hash of the parent path; folded into `file_name` when split deeper so
    /// same-named roots under different parents stay distinct and stable over time.
    pub parent_hash: String,
    pub bbox: BBox3,
}

#[derive(Debug, Default, Clone)]
pub struct HeaderBlock {
    pub version: u32,
    pub info: String,
    pub note: String,
    pub date: String,
    pub user: String,
    pub encoding: String,
}

/// A tessellated primitive stored on its MetaNode.
///
/// `vertices`/`normals`/`indices` are world-space baked geometry (used by the
/// merged path). `shape` + `world_transform` retain the placement-independent
/// definition and its placement so the instanced path can triangulate each unique
/// `shape_key` once in local space and reference it from one node per occurrence.
#[derive(Debug, Clone)]
pub struct NodePrim {
    pub opacity: u8,
    pub geo_type: GeometryType,
    pub vertices: Vec<f32>,
    pub normals: Vec<f32>,
    pub indices: Vec<u32>,
    pub vertices_n: u32,
    pub triangles_n: u32,
    pub world_transform: Mat3x4f,
    pub shape: GeometryShape,
    pub shape_key: u64,
}

#[derive(Debug, Default, Clone)]
pub struct MetaNode {
    pub id: u32,
    pub parent_id: u32,
    pub name: String,
    pub material_id: u32,
    pub start: u32,
    pub count: u32,
    pub opacity: u8,
    pub version: u8,
    pub color_with_alpha: u32,
    pub primitives: Vec<NodePrim>,
}

// ─── Chunk name → u32 (last byte of each 4-byte word) ────────────────────

const fn chunk_id(s: &[u8; 4]) -> u32 {
    ((s[0] as u32) << 24) | ((s[1] as u32) << 16) | ((s[2] as u32) << 8) | (s[3] as u32)
}

const HEAD: u32 = chunk_id(b"HEAD");
const MODL: u32 = chunk_id(b"MODL");
const CNTB: u32 = chunk_id(b"CNTB");
const CNTE: u32 = chunk_id(b"CNTE");
const PRIM: u32 = chunk_id(b"PRIM");
const OBST: u32 = chunk_id(b"OBST");
const INSU: u32 = chunk_id(b"INSU");
const COLR: u32 = chunk_id(b"COLR");
const END_: u32 = chunk_id(b"END:");

// ─── Parser ────────────────────────────────────────────────────────────────

pub struct RvmParser {
    // config
    export_level: u8,
    remove_empty: bool,
    remove_dups: bool,
    dup_precision: u8,
    tolerance: f32,
    meshopt_threshold: f32,
    meshopt_target_error: f32,
    dry_run: bool,
    mode: OutputMode,
    line_width: f32,
    include_line: bool,
    align_segments: bool,
    highlight_instance: bool,
    extract_json: bool,

    // input stream (read in ≤buf-sized chunks via InputHandle::read_at)
    input: Option<Box<dyn InputHandle>>,
    pos: usize, // current byte position in file
    file_len: usize,
    buf: [u8; 1024],  // 1 KB read buffer (matches C++ p_buffer_size)
    buf_start: usize, // file offset of buf[0]
    buf_len: usize,   // valid bytes in buf
    md5: Md5Context,

    // hierarchy state
    node_count_id: u32,
    level: u32,
    current_root_name: String,
    source_file_name: String,
    name_stack: Vec<String>, // container names, parallel to parent_stack
    current_root_parents: Vec<String>, // ancestor names captured at the current root
    current_root_bbox: BBox3f, // world bounds of the current root (JSON extract only)

    header: HeaderBlock,
    color_store: ColorStore,

    current_node: MetaNode,
    parent_stack: Vec<u32>, // stack of parent node IDs

    nodes: HashMap<u32, MetaNode>,
    site_colors: HashSet<u32>,

    filemeta: Vec<FileMeta>,
    errors: Vec<String>,
    color_overrides: Vec<(u32, u32)>,
}

impl RvmParser {
    pub fn new(opts: &ConvertOptions) -> Self {
        Self {
            export_level: opts.level,
            remove_empty: opts.remove_empty,
            remove_dups: opts.cleanup_position,
            dup_precision: opts.cleanup_precision,
            tolerance: opts.tolerance,
            meshopt_threshold: opts.meshopt_threshold,
            meshopt_target_error: opts.meshopt_target_error,
            dry_run: opts.dry_run,
            mode: opts.mode,
            line_width: opts.line_width,
            include_line: opts.include_line,
            align_segments: opts.align_segments,
            highlight_instance: opts.highlight_instance,
            extract_json: opts.extract_json,
            input: None,
            pos: 0,
            file_len: 0,
            buf: [0u8; 1024],
            buf_start: 0,
            buf_len: 0,
            md5: Md5Context::new(),
            node_count_id: 0,
            level: 0,
            current_root_name: String::new(),
            source_file_name: opts.source_name.clone(),
            name_stack: Vec::new(),
            current_root_parents: Vec::new(),
            current_root_bbox: BBox3f::empty(),
            header: HeaderBlock::default(),
            color_store: ColorStore::new(),
            current_node: MetaNode::default(),
            parent_stack: vec![0],
            nodes: HashMap::new(),
            site_colors: HashSet::new(),
            filemeta: Vec::new(),
            errors: Vec::new(),
            color_overrides: Vec::new(),
        }
    }

    /// Move the accumulated results into a [`ConvertReport`].
    pub fn into_report(self) -> ConvertReport {
        ConvertReport {
            filemeta: self.filemeta,
            warnings: self.errors,
            header: self.header,
            color_overrides: self.color_overrides,
        }
    }

    // ── Public entry point ────────────────────────────────────────────────

    /// Drive a full conversion: read from `input`, emit one GLB per site/level plus the
    /// final `status_file.json` through `sink`, reporting per-site `progress`.
    pub fn run(
        &mut self,
        input: Box<dyn InputHandle>,
        sink: &mut dyn OutputSink,
        progress: &mut dyn FnMut(&Progress),
    ) -> anyhow::Result<()> {
        // Load the file's own COLR colour overrides into the store BEFORE the main
        // parse, so material indices referenced anywhere resolve to the right RGB.
        self.prescan_colors(input.as_ref())?;

        self.file_len = input.size() as usize;
        self.input = Some(input);
        self.pos = 0;
        self.buf_start = 0;
        self.buf_len = 0;
        self.md5 = Md5Context::new();

        self.parse_rvm(sink, progress)?;

        // Final index JSON through the same sink. Skipped in dry-run, which writes
        // nothing. `--extract-json` emits `base.json` (header + site list + per-site
        // file metadata); otherwise the usual `status_file.json`.
        if !self.dry_run {
            let (name, json) = if self.extract_json {
                let json = base_json(
                    &self.filemeta,
                    &self.errors,
                    &self.header,
                    &self.source_file_name,
                    self.export_level,
                );
                ("base.json", json)
            } else {
                (
                    "status_file.json",
                    status_json(&self.filemeta, &self.errors, &self.header),
                )
            };
            match sink.open(name) {
                Ok(mut h) => {
                    if let Err(e) = h.write(json.as_bytes()) {
                        self.errors.push(format!("failed writing {name}: {e}"));
                    }
                }
                Err(e) => self.errors.push(format!("failed opening {name}: {e}")),
            }
        }
        Ok(())
    }

    /// Pre-scan the input's tail for COLR blocks and load their index→RGB overrides
    /// into the colour store (a port of the C++ "quickfix"). RVM can redefine the
    /// colour table near the end of the file, after the geometry that references it,
    /// so a single forward pass would miss them. Scans the last 10 MB (or the whole
    /// file if smaller), matching the C++ window.
    fn prescan_colors(&mut self, input: &dyn InputHandle) -> anyhow::Result<()> {
        let total = input.size() as usize;
        let window = total.min(10 * 1024 * 1024);
        let base = (total - window) as u64;
        let mut buf = vec![0u8; window];
        // read_at may be short — loop until the window is filled or the source ends.
        let mut got = 0usize;
        while got < window {
            let n = input.read_at(base + got as u64, &mut buf[got..])?;
            if n == 0 {
                break;
            }
            got += n;
        }
        let buf = &buf[..got];

        let blocks = scan_colr_blocks(buf);
        for &(index, rgb) in &blocks {
            self.color_store.insert(index, rgb);
        }
        self.color_overrides = blocks;
        Ok(())
    }

    // ── Byte-level readers ────────────────────────────────────────────────

    // Ensure buf covers self.pos, refilling from the input if needed. read_at takes an
    // absolute offset, so PRIM/CNTE position jumps need no seek bookkeeping.
    fn fill_buf(&mut self) {
        if self.pos >= self.buf_start + self.buf_len {
            let input = self.input.as_ref().expect("input not open");
            self.buf_start = self.pos;
            self.buf_len = input
                .read_at(self.pos as u64, &mut self.buf)
                .expect("read failed");
        }
    }

    fn read_u8(&mut self) -> u8 {
        self.fill_buf();
        let b = self.buf[self.pos - self.buf_start];
        self.pos += 1;
        self.md5.consume(&[b]);
        b
    }

    fn read_u32_be(&mut self) -> u32 {
        let b3 = self.read_u8() as u32;
        let b2 = self.read_u8() as u32;
        let b1 = self.read_u8() as u32;
        let b0 = self.read_u8() as u32;
        (b3 << 24) | (b2 << 16) | (b1 << 8) | b0
    }

    fn read_f32_be(&mut self) -> f32 {
        f32::from_bits(self.read_u32_be())
    }

    fn read_string(&mut self) -> String {
        let s_len = self.read_u32_be() as usize;
        let byte_len = 4 * s_len;
        let mut result = String::new();
        let mut read = 0usize;
        while read < byte_len {
            let b = self.read_u8();
            read += 1;
            if b == 0 {
                while read < byte_len {
                    self.read_u8();
                    read += 1;
                }
                break;
            }
            result.push(b as char);
        }
        result
    }

    // Read chunk header: 4 char-words (16 bytes) for name + next_abs(4) + unk(4) = 24 bytes total.
    // Returns (chunk_id, next_abs_byte_offset).
    // next_abs is the absolute byte offset in the file where the NEXT SIBLING chunk begins.
    fn read_chunk_header(&mut self) -> (u32, usize) {
        // 4 words encoding the chunk name; we only care about byte[3] of each word
        let mut name_bytes = [0u8; 4];
        for i in 0..4 {
            self.read_u8();
            self.read_u8();
            self.read_u8();
            name_bytes[i] = self.read_u8();
        }
        let cid = ((name_bytes[0] as u32) << 24)
            | ((name_bytes[1] as u32) << 16)
            | ((name_bytes[2] as u32) << 8)
            | (name_bytes[3] as u32);
        let next_abs = self.read_u32_be() as usize; // absolute byte offset to next sibling
        let _unk = self.read_u32_be(); // version/unknown word
        (cid, next_abs)
    }

    // ── Block parsers ─────────────────────────────────────────────────────

    fn parse_head(&mut self) -> HeaderBlock {
        let version = self.read_u32_be();
        let info = self.read_string();
        let note = self.read_string();
        let date = self.read_string();
        let user = self.read_string();
        let encoding = if version >= 2 {
            self.read_string()
        } else {
            String::new()
        };
        HeaderBlock {
            version,
            info,
            note,
            date,
            user,
            encoding,
        }
    }

    fn skip_modl(&mut self) {
        let _version = self.read_u32_be();
        let _project = self.read_string();
        let _name = self.read_string();
    }

    fn skip_colr(&mut self) {
        let _ver = self.read_u32_be();
        let _index = self.read_u32_be();
        self.read_u8();
        self.read_u8();
        self.read_u8();
        self.read_u8();
    }

    /// Reads CNTB body (after header). Returns (name, resolved_color_hex, opacity, body_end_pos).
    /// Children begin at body_end_pos.
    fn read_cntb_body(&mut self) -> (String, u32, u8) {
        let version = self.read_u32_be();
        let name = self.read_string();
        let _tx = self.read_f32_be();
        let _ty = self.read_f32_be();
        let _tz = self.read_f32_be();
        let raw_material = self.read_u32_be();
        let material = self.color_store.get(raw_material);
        let opacity = if version > 2 {
            let op = self.read_u8();
            self.read_u8();
            self.read_u8();
            self.read_u8();
            op
        } else {
            100
        };
        (name, material, opacity)
    }

    fn parse_prim(&mut self, chunk_type: u32, next_abs: usize) {
        let _version = self.read_u32_be();
        let kind = self.read_u32_be();

        let mut geo_type = GeometryType::Primitive;
        let mut prim_opacity = 100u8;

        // OBST/INSU have an extra 4-byte opacity field before the matrix
        match chunk_type {
            OBST | INSU => {
                prim_opacity = self.read_u8();
                self.read_u8();
                self.read_u8();
                self.read_u8();
                geo_type = if chunk_type == INSU {
                    GeometryType::Insulation
                } else {
                    GeometryType::Obstruction
                };
            }
            _ => {}
        }

        let mut m_3x4 = Mat3x4f::default();
        for i in 0..12 {
            m_3x4.data[i] = self.read_f32_be();
        }

        let bbox_raw: [f32; 6] = std::array::from_fn(|_| self.read_f32_be());
        let bbox_local = BBox3f {
            min: Vec3f::new(bbox_raw[0], bbox_raw[1], bbox_raw[2]),
            max: Vec3f::new(bbox_raw[3], bbox_raw[4], bbox_raw[5]),
        };
        let bbox_world = transform_bbox(&m_3x4, &bbox_local);

        // Validate the primitive kind before reading its parameters: an unknown tag
        // means we don't know the body layout, so skip it (and tell the user) rather
        // than misinterpret the bytes.
        let geo_kind = match kind_from_u32(kind) {
            Some(k) => k,
            None => {
                let msg = format!(
                    "Unknown primitive kind {} — skipping (RVM may have a new primitive type)",
                    kind
                );
                eprintln!("WARNING: {}", msg);
                self.errors.push(msg);
                self.pos = next_abs;
                return;
            }
        };

        // RVM Line primitives are skipped unless explicitly included — they are
        // numerous and add visual noise. Skipping here (before reading the body)
        // excludes them from every path: streaming, merged, instanced, and JSON.
        if geo_kind == GeometryKind::Line && !self.include_line {
            self.pos = next_abs;
            return;
        }

        let shape = match self.parse_shape(kind, next_abs) {
            Some(s) => s,
            None => return,
        };

        let mut geo = Geometry::new(geo_kind, shape);
        geo.geo_type = geo_type;
        geo.m_3x4 = m_3x4;
        geo.bbox_local = bbox_local;
        geo.bbox_world = bbox_world;

        // JSON-extract path: record the parametric shape + placement only — no
        // tessellation. The world bbox feeds the per-root bounds in `base.json`.
        if self.extract_json {
            self.current_root_bbox.engulf_box(&bbox_world);
            self.current_node.primitives.push(NodePrim {
                opacity: prim_opacity,
                geo_type,
                vertices: Vec::new(),
                normals: Vec::new(),
                indices: Vec::new(),
                vertices_n: 0,
                triangles_n: 0,
                world_transform: m_3x4,
                shape_key: 0,
                shape: geo.shape.clone(),
            });
            self.pos = next_abs;
            return;
        }

        let tessellator = Tessellator::new(self.tolerance, self.line_width, self.align_segments);
        // Pass empty slices – connection-based cap removal is skipped (no connection data in stream)
        if let Some(tri) = tessellator.geometry(&geo, &[], &[]) {
            if tri.vertices_n > 0 && !self.dry_run {
                let prim = NodePrim {
                    opacity: prim_opacity,
                    geo_type,
                    vertices: tri.vertices,
                    normals: tri.normals,
                    indices: tri.indices,
                    vertices_n: tri.vertices_n,
                    triangles_n: tri.triangles_n,
                    world_transform: m_3x4,
                    shape_key: shape_key(&geo.shape, self.tolerance),
                    shape: geo.shape.clone(),
                };
                self.current_node.primitives.push(prim);
                // Colour (with opacity-derived alpha) is computed at store time, after
                // primitives are split by geo_type — see store_current_node.
            }
        }

        // Jump to start of next sibling (past any shape data we didn't read)
        self.pos = next_abs;
    }

    fn parse_shape(&mut self, kind: u32, next_abs: usize) -> Option<GeometryShape> {
        let shape = match kind {
            1 => GeometryShape::Pyramid {
                bottom: [self.read_f32_be(), self.read_f32_be()],
                top: [self.read_f32_be(), self.read_f32_be()],
                offset: [self.read_f32_be(), self.read_f32_be()],
                height: self.read_f32_be(),
            },
            2 => GeometryShape::Box {
                lengths: [self.read_f32_be(), self.read_f32_be(), self.read_f32_be()],
            },
            3 => GeometryShape::RectangularTorus {
                inner_radius: self.read_f32_be(),
                outer_radius: self.read_f32_be(),
                height: self.read_f32_be(),
                angle: self.read_f32_be(),
            },
            4 => GeometryShape::CircularTorus {
                offset: self.read_f32_be(),
                radius: self.read_f32_be(),
                angle: self.read_f32_be(),
            },
            5 => GeometryShape::EllipticalDish {
                base_radius: self.read_f32_be(),
                height: self.read_f32_be(),
            },
            6 => GeometryShape::SphericalDish {
                base_radius: self.read_f32_be(),
                height: self.read_f32_be(),
            },
            7 => GeometryShape::Snout {
                radius_b: self.read_f32_be(),
                radius_t: self.read_f32_be(),
                height: self.read_f32_be(),
                offset: [self.read_f32_be(), self.read_f32_be()],
                bshear: [self.read_f32_be(), self.read_f32_be()],
                tshear: [self.read_f32_be(), self.read_f32_be()],
            },
            8 => GeometryShape::Cylinder {
                radius: self.read_f32_be(),
                height: self.read_f32_be(),
            },
            9 => GeometryShape::Sphere {
                diameter: self.read_f32_be(),
            },
            10 => {
                // Line endpoints along the local X-axis: P0=(a,0,0), P1=(b,0,0).
                // Drawn as a thin cross of quads (see triangulation_factory::line_cross).
                let a = self.read_f32_be();
                let b = self.read_f32_be();
                return Some(GeometryShape::Line { a, b });
            }
            11 => {
                let polys_n = self.read_u32_be() as usize;
                let mut polygons = Vec::with_capacity(polys_n);
                for _ in 0..polys_n {
                    let contours_n = self.read_u32_be() as usize;
                    let mut contours = Vec::with_capacity(contours_n);
                    for _ in 0..contours_n {
                        let verts_n = self.read_u32_be() as usize;
                        let mut vertices = Vec::with_capacity(verts_n * 3);
                        for _ in 0..verts_n {
                            vertices.push(self.read_f32_be()); // x
                            vertices.push(self.read_f32_be()); // y
                            vertices.push(self.read_f32_be()); // z
                            // normals – read and discard
                            self.read_f32_be();
                            self.read_f32_be();
                            self.read_f32_be();
                        }
                        contours.push(Contour { vertices });
                    }
                    polygons.push(Polygon { contours });
                }
                GeometryShape::FacetGroup { polygons }
            }
            // Defensive: parse_prim already validated the kind via kind_from_u32, so an
            // unknown tag never reaches here. Skip safely if the invariant ever breaks.
            _ => {
                self.pos = next_abs;
                return None;
            }
        };
        Some(shape)
    }

    // ── Main parse loop ───────────────────────────────────────────────────
    //
    // RVM chunk layout:
    //   [4 words = chunk name] [next_abs: u32] [unk: u32] [body...]
    //   next_abs = absolute byte offset of the next SIBLING chunk.
    //   CNTB children immediately follow the CNTB body.
    //   CNTE has no body; on CNTE we pop next_abs from our stack and jump there.

    fn parse_rvm(
        &mut self,
        sink: &mut dyn OutputSink,
        progress: &mut dyn FnMut(&Progress),
    ) -> anyhow::Result<()> {
        while self.pos + 24 <= self.file_len {
            let (cid, next_abs) = self.read_chunk_header();

            match cid {
                HEAD => {
                    self.header = self.parse_head();
                    // jump to next sibling (skips any unread HEAD data)
                    self.pos = next_abs;
                }
                MODL => {
                    self.skip_modl();
                    self.pos = next_abs;
                }
                COLR => {
                    self.skip_colr();
                    self.pos = next_abs;
                }
                CNTB => {
                    let (name, material, opacity) = self.read_cntb_body();

                    // Realign to the chunk header's offset (the end of the CNTB body /
                    // first child). For known versions read_cntb_body lands here exactly
                    // (a no-op); for the occasional CNTB version 4 the body carries extra
                    // bytes after the fields we read, and this skips them — the clean
                    // equivalent of the C++ "quickfix for cntb version 4 offset".
                    self.pos = next_abs;

                    // Store previously accumulated node
                    self.store_current_node();

                    self.level += 1;

                    // At the export split level, start a new root export
                    if self.level == (self.export_level + 1) as u32 {
                        if !self.current_root_name.is_empty() {
                            self.flush_root(sink, progress);
                        }
                        self.current_root_name = name.clone();
                        // Capture ancestor names BEFORE this container is pushed onto
                        // the name stack — these are the containers above the root.
                        self.current_root_parents = self.name_stack.clone();
                        self.nodes.clear();
                        self.site_colors.clear();
                        self.node_count_id = 0;
                        self.current_root_bbox = BBox3f::empty();
                    }

                    self.node_count_id += 1;
                    let parent_id = *self.parent_stack.last().unwrap_or(&0);

                    self.current_node = MetaNode {
                        id: self.node_count_id,
                        parent_id,
                        name: name.clone(),
                        material_id: material, // resolved RGB; alpha + split applied at store time
                        opacity,
                        ..MetaNode::default()
                    };

                    self.parent_stack.push(self.node_count_id);
                    self.name_stack.push(name);
                    // Children follow immediately; CNTB body is already fully consumed.
                }
                CNTE => {
                    // End of a CNTB block. We do NOT store/clear current_node here:
                    // PDMS stores connecting tubes as branch-level PRIMs that appear
                    // AFTER a component's CNTE, and they must attach to that just-closed
                    // component (matching the C++ reference). current_node therefore
                    // persists until the next CNTB stores it.
                    self.parent_stack.pop();
                    self.name_stack.pop();
                    self.level = self.level.saturating_sub(1);
                    self.pos = next_abs;
                }
                PRIM | OBST | INSU => {
                    self.parse_prim(cid, next_abs);
                    // parse_prim sets self.pos = next_abs already
                }
                END_ => {
                    break;
                }
                _ => {
                    // Unknown chunk — skip to next sibling
                    if next_abs > self.pos {
                        self.pos = next_abs;
                    } else {
                        break; // corrupt / end of file
                    }
                }
            }
        }

        // Flush the final root
        if !self.current_root_name.is_empty() {
            self.store_current_node();
            self.flush_root(sink, progress);
        }

        Ok(())
    }

    /// Store the just-finished node. Ported from the C++ `store_last_node`: primitives
    /// are split by geometry type into separate MetaNodes — the PRIM group keeps
    /// the node's id/name, INSU/OBST split into sibling nodes with fresh ids and a
    /// "(INSU)"/"(OBST)" name suffix — and each gets `color_with_alpha = (alpha<<24)|rgb`
    /// with `alpha = opacity*255/100` (PRIM uses the container opacity, INSU/OBST
    /// the primitive's own opacity).
    fn store_current_node(&mut self) {
        if self.current_node.id == 0 {
            return;
        }

        // JSON-extract path: keep the node verbatim — every primitive with its own
        // kind/type, no geo-type split, empty containers retained — so the dumped tree
        // mirrors the RVM hierarchy exactly.
        if self.extract_json {
            let node = std::mem::take(&mut self.current_node);
            self.nodes.insert(node.id, node);
            return;
        }

        let node = std::mem::take(&mut self.current_node);
        let rgb = node.material_id & 0x00FF_FFFF;

        let mut prim: Vec<NodePrim> = Vec::new();
        let mut insu: Vec<NodePrim> = Vec::new();
        let mut obst: Vec<NodePrim> = Vec::new();
        for p in node.primitives {
            match p.geo_type {
                GeometryType::Primitive => prim.push(p),
                GeometryType::Insulation => insu.push(p),
                GeometryType::Obstruction => obst.push(p),
            }
        }

        // Empty node (container / empty leaf): keep it for the hierarchy, no colour.
        if prim.is_empty() && insu.is_empty() && obst.is_empty() {
            self.nodes.insert(
                node.id,
                MetaNode {
                    id: node.id,
                    parent_id: node.parent_id,
                    name: node.name,
                    opacity: node.opacity,
                    ..MetaNode::default()
                },
            );
            return;
        }

        // PRIM keeps the node's id; INSU/OBST become new sibling nodes (fresh ids).
        let insu_op = insu.first().map(|p| p.opacity).unwrap_or(node.opacity);
        let obst_op = obst.first().map(|p| p.opacity).unwrap_or(node.opacity);
        let mut used_id = false;
        for (prims, opacity, suffix) in [
            (prim, node.opacity, ""),
            (insu, insu_op, "(INSU)"),
            (obst, obst_op, "(OBST)"),
        ] {
            if prims.is_empty() {
                continue;
            }
            let alpha = ((opacity as u32 * 255) / 100).min(255);
            let color = (alpha << 24) | rgb;
            self.site_colors.insert(color);
            let id = if used_id {
                self.node_count_id += 1;
                self.node_count_id
            } else {
                used_id = true;
                node.id
            };
            let name = if suffix.is_empty() {
                node.name.clone()
            } else {
                format!("{}{}", node.name, suffix)
            };
            self.nodes.insert(
                id,
                MetaNode {
                    id,
                    parent_id: node.parent_id,
                    name,
                    material_id: node.material_id,
                    color_with_alpha: color,
                    opacity: node.opacity,
                    primitives: prims,
                    ..MetaNode::default()
                },
            );
        }
    }

    fn flush_root(&mut self, sink: &mut dyn OutputSink, progress: &mut dyn FnMut(&Progress)) {
        if self.dry_run {
            self.nodes.clear();
            self.site_colors.clear();
            self.node_count_id = 0;
            self.current_root_name.clear();
            self.current_root_bbox = BBox3f::empty();
            return;
        }

        // JSON-extract path: write `<site>.json` (the structure tree) instead of a GLB.
        if self.extract_json {
            self.flush_root_json(sink, progress);
            self.nodes.clear();
            self.site_colors.clear();
            self.node_count_id = 0;
            self.current_root_name.clear();
            self.current_root_bbox = BBox3f::empty();
            return;
        }

        let mut colors: Vec<u32> = self.site_colors.iter().copied().collect();
        colors.sort();

        let md5_hash = format!("{:x}", self.md5.clone().compute());

        let cleanup = Cleanup {
            cleanup_positions: self.remove_dups,
            cleanup_precision: self.dup_precision,
            meshopt_threshold: self.meshopt_threshold,
            meshopt_target_error: self.meshopt_target_error,
        };
        let (glb_bytes, bbox) = match self.mode {
            OutputMode::Instanced => GlbWriter::build_instanced(
                &self.nodes,
                self.tolerance,
                self.line_width,
                self.align_segments,
                self.highlight_instance,
                self.remove_empty,
                &cleanup,
            ),
            OutputMode::Standard => {
                GlbWriter::build_standard(&self.nodes, self.remove_empty, &cleanup)
            }
            OutputMode::Merged => GlbWriter::build(
                &mut self.nodes,
                &colors,
                self.remove_empty,
                self.remove_dups,
                self.dup_precision,
                self.meshopt_threshold,
                self.meshopt_target_error,
            ),
        };

        if !glb_bytes.is_empty() {
            let glb_md5 = format!("{:x}", md5::compute(&glb_bytes));
            let parent = self.current_root_parents.clone();
            // Stable, deterministic hash of the parent path. Folded into the filename
            // when split deeper than the site level so same-named roots under
            // different parents stay distinct and stable across re-exports.
            let parent_hash = if parent.is_empty() {
                String::new()
            } else {
                let h = format!("{:x}", md5::compute(parent.join("/").as_bytes()));
                h[..8].to_string()
            };

            let base = sanitize_filename(&self.current_root_name);
            let filename = if parent_hash.is_empty() {
                format!("{}.glb", base)
            } else {
                format!("{}_{}.glb", base, parent_hash)
            };

            match sink.open(&filename) {
                Ok(mut h) => {
                    if let Err(e) = h.write(&glb_bytes) {
                        self.errors.push(format!("failed writing {filename}: {e}"));
                    }
                    // drop(h) closes the output.
                }
                Err(e) => self.errors.push(format!("failed opening {filename}: {e}")),
            }

            let output_index = self.filemeta.len() as u32;
            let nodes = self.nodes.len() as u32;
            self.filemeta.push(FileMeta {
                root_name: self.current_root_name.clone(),
                file_name: filename.clone(),
                source_file_name: self.source_file_name.clone(),
                md5: md5_hash,
                glb_md5,
                export_lvl: self.export_level,
                parent,
                parent_hash,
                bbox,
            });
            progress(&Progress {
                output_index,
                output_name: &filename,
                nodes,
            });
        }

        self.nodes.clear();
        self.site_colors.clear();
        self.node_count_id = 0;
        self.current_root_name.clear();
    }

    /// Write the current root's structure tree as `<site>.json` and record its metadata
    /// (file name, parent path, world bbox) for `base.json`. Mirrors `flush_root`'s file
    /// naming (parent-hash suffix when split below site level) so JSON and GLB exports
    /// of the same model line up. The caller clears per-root state afterwards.
    fn flush_root_json(&mut self, sink: &mut dyn OutputSink, progress: &mut dyn FnMut(&Progress)) {
        let json = site_json(&self.nodes);

        let parent = self.current_root_parents.clone();
        let parent_hash = if parent.is_empty() {
            String::new()
        } else {
            let h = format!("{:x}", md5::compute(parent.join("/").as_bytes()));
            h[..8].to_string()
        };
        let base = sanitize_filename(&self.current_root_name);
        let filename = if parent_hash.is_empty() {
            format!("{}.json", base)
        } else {
            format!("{}_{}.json", base, parent_hash)
        };

        match sink.open(&filename) {
            Ok(mut h) => {
                if let Err(e) = h.write(json.as_bytes()) {
                    self.errors.push(format!("failed writing {filename}: {e}"));
                }
            }
            Err(e) => self.errors.push(format!("failed opening {filename}: {e}")),
        }

        let b = self.current_root_bbox;
        let bbox = if b.is_empty() {
            BBox3::default()
        } else {
            BBox3 {
                min_x: b.min.x,
                min_y: b.min.y,
                min_z: b.min.z,
                max_x: b.max.x,
                max_y: b.max.y,
                max_z: b.max.z,
            }
        };

        let output_index = self.filemeta.len() as u32;
        let nodes = self.nodes.len() as u32;
        self.filemeta.push(FileMeta {
            root_name: self.current_root_name.clone(),
            file_name: filename.clone(),
            source_file_name: self.source_file_name.clone(),
            md5: String::new(),
            glb_md5: String::new(),
            export_lvl: self.export_level,
            parent,
            parent_hash,
            bbox,
        });
        progress(&Progress {
            output_index,
            output_name: &filename,
            nodes,
        });
    }
}

/// Map an RVM primitive-kind tag to a `GeometryKind`. Returns `None` for any tag
/// outside the known 1..=11 range so the caller can skip + warn rather than guess
/// (e.g. if a future RVM revision adds a primitive type).
fn kind_from_u32(k: u32) -> Option<GeometryKind> {
    Some(match k {
        1 => GeometryKind::Pyramid,
        2 => GeometryKind::Box,
        3 => GeometryKind::RectangularTorus,
        4 => GeometryKind::CircularTorus,
        5 => GeometryKind::EllipticalDish,
        6 => GeometryKind::SphericalDish,
        7 => GeometryKind::Snout,
        8 => GeometryKind::Cylinder,
        9 => GeometryKind::Sphere,
        10 => GeometryKind::Line,
        11 => GeometryKind::FacetGroup,
        _ => return None,
    })
}

/// Scan a byte buffer for RVM COLR blocks, returning `(color_index, rgb)` pairs.
/// Matches the C++ signature check: "COLR" (one char per 4-byte word) at i,+4,+8,+12,
/// validation bytes 0,0,0,1 at i+17..=20, index (big-endian) at i+25..=28, and RGB at
/// i+29..=31.
fn scan_colr_blocks(buf: &[u8]) -> Vec<(u32, u32)> {
    let n = buf.len();
    let mut out = Vec::new();
    for i in 0..n.saturating_sub(31) {
        if buf[i] == b'C'
            && buf[i + 4] == b'O'
            && buf[i + 8] == b'L'
            && buf[i + 12] == b'R'
            && buf[i + 17] == 0
            && buf[i + 18] == 0
            && buf[i + 19] == 0
            && buf[i + 20] == 1
        {
            let index = u32::from_be_bytes([buf[i + 25], buf[i + 26], buf[i + 27], buf[i + 28]]);
            let (cr, cg, cb) = (buf[i + 29] as u32, buf[i + 30] as u32, buf[i + 31] as u32);
            out.push((index, (cr << 16) | (cg << 8) | cb));
        }
    }
    out
}

fn sanitize_filename(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn store_splits_insu_obst_with_opacity_alpha() {
        let mut p = RvmParser::new(&ConvertOptions {
            meshopt_threshold: 1.0,
            ..ConvertOptions::default()
        });
        let mk = |gt: GeometryType, op: u8| NodePrim {
            opacity: op,
            geo_type: gt,
            vertices: vec![0.0; 3],
            normals: vec![],
            indices: vec![0],
            vertices_n: 1,
            triangles_n: 0,
            world_transform: Mat3x4f::identity(),
            shape: GeometryShape::Box {
                lengths: [1.0, 1.0, 1.0],
            },
            shape_key: 0,
        };
        p.node_count_id = 7;
        p.current_node = MetaNode {
            id: 7,
            parent_id: 3,
            name: "ELBOW".into(),
            material_id: 0x00cc_0000, // resolved RGB
            opacity: 100,
            primitives: vec![
                mk(GeometryType::Primitive, 100),
                mk(GeometryType::Insulation, 50),
                mk(GeometryType::Obstruction, 25),
            ],
            ..MetaNode::default()
        };
        p.store_current_node();

        // PRIM keeps id 7; INSU/OBST split into new sibling nodes 8/9 with suffixes.
        assert_eq!(p.nodes[&7].name, "ELBOW");
        assert_eq!(p.nodes[&8].name, "ELBOW(INSU)");
        assert_eq!(p.nodes[&9].name, "ELBOW(OBST)");
        for id in [7, 8, 9] {
            assert_eq!(p.nodes[&id].parent_id, 3);
            assert_eq!(p.nodes[&id].color_with_alpha & 0xFF_FFFF, 0xcc_0000);
        }
        // alpha = opacity*255/100: 100→255, 50→127, 25→63.
        assert_eq!(p.nodes[&7].color_with_alpha >> 24, 255);
        assert_eq!(p.nodes[&8].color_with_alpha >> 24, 127);
        assert_eq!(p.nodes[&9].color_with_alpha >> 24, 63);
        assert_eq!(p.site_colors.len(), 3);
    }

    #[test]
    fn colr_prescan_extracts_index_and_rgb() {
        // One COLR block: index 42 (big-endian), rgb #112233.
        let mut buf = vec![0u8; 40];
        buf[0] = b'C';
        buf[4] = b'O';
        buf[8] = b'L';
        buf[12] = b'R';
        buf[20] = 1; // validation 0,0,0,1 at 17..=20
        buf[28] = 42; // index big-endian
        buf[29] = 0x11;
        buf[30] = 0x22;
        buf[31] = 0x33;
        assert_eq!(scan_colr_blocks(&buf), vec![(42, 0x112233)]);
        // No false positive without the signature.
        assert!(scan_colr_blocks(&[0u8; 64]).is_empty());
    }

    #[test]
    fn known_kinds_map_unknown_is_none() {
        assert_eq!(kind_from_u32(2), Some(GeometryKind::Box));
        assert_eq!(kind_from_u32(10), Some(GeometryKind::Line));
        assert_eq!(kind_from_u32(11), Some(GeometryKind::FacetGroup));
        // Out-of-range tags must be None so the parser skips + warns, never guesses.
        assert!(kind_from_u32(0).is_none());
        assert!(kind_from_u32(12).is_none());
        assert!(kind_from_u32(99).is_none());
    }
}
