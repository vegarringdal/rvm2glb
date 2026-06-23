# rvm2glb

**rvm2glb** converts PDMS/E3D **RVM** plant-model files to **GLB** (binary glTF 2.0).
It is a port/evolution of a C++ tool (itself derived from [rvm_parser_glb](https://github.com/vegarringdal/rvm_parser_glb) thats derived from
[cdyk/rvmparser](https://github.com/cdyk/rvmparser)).

> This was written with help heavy usage of misc AI tools to save time and get something Ive been planning done :-)

> Atm only tested on very simple rvm files, so might need some patches before it stable

It is a Cargo **workspace**: a platform-agnostic conversion engine (`rvm2glb-core`)
driven by a CLI, a C ABI, and a WebAssembly shell with an in-browser demo — see
[Crates](#crates) below.

**▶ Live demo: <https://vegarringdal.github.io/rvm2glb/>** — converts RVM → GLB entirely
in your browser (Web Worker + OPFS), nothing uploaded. Auto-deployed from `main` by
[`.github/workflows/pages.yml`](.github/workflows/pages.yml).

Four output modes (`--mode`):

- **`merged`** (default) — one merged mesh per unique colour, geometry baked to
  world space. Smallest output; per-component identity is kept out-of-band in
  scene `extras` (`id_hierarchy` + `draw_ranges`) for treeview + selection.
- **`instanced`** — repeated shapes are detected and each unique shape is
  triangulated once. Emits the **native glTF node tree** (RVM hierarchy) with each
  occurrence as a child node referencing the shared mesh with its own `matrix` — plain
  glTF mesh reuse, no extension, no web3d `extras`. Component nodes carry the RVM name.
- **`gpu-instanced`** — same shape dedup as `instanced`, but every occurrence of a
  (shape, colour) mesh collapses into a **single** node using the
  [`EXT_mesh_gpu_instancing`](https://github.com/KhronosGroup/glTF/blob/main/extensions/2.0/Vendor/EXT_mesh_gpu_instancing/README.md)
  extension, which carries per-instance `TRANSLATION`/`ROTATION`/`SCALE`. Far fewer
  nodes and GPU-friendly draw-instancing, at the cost of flattening the RVM node tree
  (the extension takes TRS, so each world transform is decomposed — RVM transforms are
  rigid + uniform scale, so it's exact). The extension is listed in `extensionsUsed`
  *and* `extensionsRequired`, so the viewer must support it (three.js / `<model-viewer>`
  do). Example: on HA-PSUP this is 811 nodes vs `instanced`'s 13778, ~1.6 MB vs 2.7 MB.
- **`standard`** — neither merged nor instanced: the **native glTF node tree**
  mirroring the RVM hierarchy (one node per container/leaf, wired parent→children;
  meshes on geometry nodes, world-space, no merge/dedup). The most faithful structure
  (and largest output); every node's `name` is the component's actual RVM name, so the
  full model — geometry, colours, hierarchy — is readable back from it.

RVM **Line** primitives (which the C++ tool skipped) are drawn as a thin "+" cross
of two quads swept along the line, so they are selectable in a triangle viewer.

Alternatively, `--extract-json` dumps the **RVM structure as JSON** instead of
converting to GLB: one `<site>.json` per root (the full hierarchy, each primitive with
its kind, type, opacity, transform matrix, and parametric params) plus a `base.json`
index. FacetGroups are summarised to polygon/vertex counts. No tessellation — handy for
inspecting or diffing model structure. See [Output](#output).

## Crates

| Crate | Kind | What it is |
|-------|------|-----------|
| [`crates/core`](crates/core) — `rvm2glb-core` (lib name `rvm2glb`) | library | the whole engine: RVM parser, tessellation, instancing, the three output modes, the GLB writer, and the one-call `convert()` + I/O traits (`InputHandle` / `OutputHandle` / `OutputSink`). **Filesystem-free**, no CLI dependency. Feature `optimize` (default on) pulls in meshoptimizer (simplify + vertex-cache). |
| [`crates/cli`](crates/cli) — `rvm2glb-cli` (bin `rvm2glb`) | binary | clap front end: file input + a directory output sink. |
| [`crates/capi`](crates/capi) — `rvm2glb-capi` | cdylib + staticlib | C ABI (`rvm2glb_convert`) with streaming I/O + progress via C function pointers; header at [`crates/capi/include/rvm2glb.h`](crates/capi/include/rvm2glb.h). |
| [`crates/wasm`](crates/wasm) — `rvm2glb-wasm` | cdylib (wasm) | `wasm-bindgen` shell: in-RAM + OPFS-streaming entry points. meshopt is **off by default** (opt-in `optimize` feature; needs `clang`). |
| [`wasm-demo/`](wasm-demo) | TypeScript app | in-browser demo (Web Worker + OPFS + `<model-viewer>`) — **[live](https://vegarringdal.github.io/rvm2glb/)**. See its [README](wasm-demo/README.md). |

The engine exposes `convert(input, sink, opts, progress) -> ConvertReport`: it reads the
RVM in small chunks through an `InputHandle` and emits one GLB per site/level (plus a
`status_file.json`) through an `OutputSink` — the same kernel backs the CLI (files), the
C ABI (callbacks), and wasm (OPFS). There is no temp file; one site is built in RAM,
written, and dropped before the next.

## Build

```bash
# Install Rust (if needed): https://rustup.rs
cargo build --release            # default members: core + cli + capi
# CLI binary:  target/release/rvm2glb
# C library:   target/release/librvm2glb_capi.{so,a}  (+ crates/capi/include/rvm2glb.h)

# Cross-compile the CLI for Windows from Linux (linker config in .cargo/config.toml)
sudo apt install gcc-mingw-w64-x86-64
rustup target add x86_64-pc-windows-gnu
cargo build --release -p rvm2glb-cli --target x86_64-pc-windows-gnu
# Binary: target/x86_64-pc-windows-gnu/release/rvm2glb.exe

# WebAssembly + browser demo (needs Node + wasm-pack)
cd wasm-demo && npm install && npm run dev     # serves http://localhost:5173
# meshopt in the browser build (optional, needs clang): npm run build:wasm:opt
```

Run the tests:

```bash
cargo test --workspace
```

## Usage

```
rvm2glb --input <FILE> [OPTIONS]
```

| Option | Default | Description |
|--------|---------|-------------|
| `-i, --input <FILE>` | *(required)* | RVM input file. |
| `-o, --output <DIR>` | `./exports/` | Output folder (created if missing). |
| `--mode <MODE>` | `merged` | `merged` (one mesh per colour), `instanced` (node per occurrence, shared meshes), `gpu-instanced` (one node per shared mesh via `EXT_mesh_gpu_instancing`, per-instance TRS), or `standard` (one mesh per component, no merge/dedup). |
| `-j, --extract-json` | `false` | Dump the RVM structure as JSON (`<site>.json` + `base.json`) instead of GLB. Overrides `--mode`; honours `--level`. |
| `-x, --dry-run` | `false` | Parse only; do not write files. |
| `-l, --level <N>` | `0` | Hierarchy depth at which to split into separate GLB files (`0` = site). |
| `-r, --remove-empty <0\|1>` | `1` | Drop nodes/branches with no geometry (all modes). Disable with `-r 0`. |
| `-d, --cleanup-position <0\|1>` | `1` | Weld coincident vertices. Disable with `-d 0`. |
| `-p, --cleanup-precision <N>` | `3` | Decimal places for the vertex weld grid. |
| `-m, --meshopt-threshold <F>` | `0.75` | meshopt simplify target (fraction of indices to keep; `1.0` disables). All three modes. |
| `-e, --meshopt-target-error <F>` | `0.0` | meshopt simplify error budget. At `0` (default) only lossless collapses happen — raise it for real simplification. All three modes. |
| `-t, --tolerance <F>` | `0.01` | Tessellation chord-height tolerance. |
| `--line-width <W>` | `0.05` | Width (model units) of the "+" cross drawn for Line primitives. |
| `--align-segments` | `off` | Round circle tessellation up to a multiple of 4 segments so adjacent primitives share boundary vertices (better flat-shading alignment, ~25% larger output). |
| `--highlight-instance` | `off` | Instanced mode only: colour shapes shared by ≥2 occurrences yellow and one-offs grey, to visualise what got instanced. |

### Examples

```bash
# Default merged conversion → ./exports/
rvm2glb -i model.rvm

# Instanced output (shared meshes + node-per-occurrence)
rvm2glb -i model.rvm --mode instanced

# Custom output, looser tessellation (faster/coarser)
rvm2glb -i model.rvm -o ./out/ -t 0.05

# Disable simplification (keep full resolution)
rvm2glb -i model.rvm -m 1.0

# Wider line crosses (easier to click)
rvm2glb -i model.rvm --line-width 0.1

# Split one GLB per second-level container (e.g. zone)
rvm2glb -i model.rvm -l 1

# Extract the RVM structure as JSON (no tessellation) → base.json + <site>.json
rvm2glb -i model.rvm --extract-json

# Dry run — parse only
rvm2glb -i model.rvm -x
```

## Output

Each exported root writes `<root>.glb` plus a `status_file.json`. **Merged** GLBs
carry the web3d contract in `extras`: `asset.extras.web3dversion`, scene
`id_hierarchy` (`{ node_id: [name, parent_id] }`) for the treeview, and
`draw_ranges_node<N>` (`{ node_id: [start, count] }`) for per-component selection
against the merged index buffers. **Instanced** GLBs are plain glTF with no such
extras — selection is per glTF node via its `name` (= RVM node id). **GPU-instanced**
GLBs flatten the tree to one node per shared mesh (no per-component names) and require
the `EXT_mesh_gpu_instancing` extension in the viewer.

`status_file.json` lists one entry per exported model with: `root_name`,
`source_file_name` (the input RVM), `file_name`, `md5` (RVM stream hash),
`glb_md5` (hash of the written GLB), `export_lvl` (the `--level` used), `parent`
(container names above the root — empty at level 0), `parent_hash` (stable hash of
the parent path, folded into `file_name` when split deeper so same-named roots stay
distinct), and the model bbox (`min_*`/`max_*`).

### JSON extraction (`--extract-json`)

With `--extract-json` the converter writes one `<site>.json` per root (named like the
GLBs, with the same `parent_hash` suffix when split below site level) plus a `base.json`
index — no tessellation, no GLB.

`base.json` holds the RVM `header`, `source_file_name`, `export_lvl`, a `sites` string
array of root names, a `files` array (per-site `file_name`, `parent`/`parent_hash`, and
world bbox `min_*`/`max_*`), and `warnings`.

Each `<site>.json` is the full node tree. Every node carries `id`, `name`, `opacity`,
`color` (hex RGB), a `primitives` array, and a recursive `children` array. Each
primitive records its `type` (`Primitive` / `Insulation` / `Obstruction`), `kind`
(`Box`, `Cylinder`, `Snout`, …, `FacetGroup`), `opacity`, the raw 12-float column-major
RVM `matrix`, and `params` (the parametric fields). **FacetGroups are reduced to
`{ polygons, vertices }` counts** — the raw contour data is omitted, as it would dwarf
everything else.

## Status & roadmap

**Done**

- All four output modes (merged / instanced / gpu-instanced / standard); RVM Line
  primitives drawn as "+" crosses.
- Parse fidelity vs the C++ reference: connecting-tube recovery, COLR colour-override
  pre-scan, CNTB v4 offset realign, unknown-kind skip+warn, PRIM/INSU/OBST split,
  opacity→alpha, meshopt `LockBorder` parity, degenerate-triangle cull + vertex
  compaction, and **interface cap removal** (shared faces between connected primitives
  are dropped, à la cdyk/rvmparser). Default merged output is size-comparable to the C++ tool.
- `core / cli / capi / wasm` workspace: filesystem-free engine behind
  `convert()` + I/O traits; C ABI; WebAssembly + in-browser demo (Worker + OPFS); GitHub
  Pages auto-deploy.
- meshopt builds into wasm too (opt-in `optimize` feature, needs `clang`).
- `--extract-json`: dump the RVM structure (hierarchy + parametric primitives, matrices,
  colours) to JSON instead of GLB — `base.json` index + one `<site>.json` per root.

**Planned / ideas**

- [ ] Browser smoke-test of the wasm demo across the sample set.
- [ ] Optional **vertex-normal computation** for the GLB (currently POSITION-only).
- [ ] RVM **extract / split** (pull a subtree out into a new RVM), à la
      [rvmsplitter](https://github.com/vegarringdal/rvmsplitter).
