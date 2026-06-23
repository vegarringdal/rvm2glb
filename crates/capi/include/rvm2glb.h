/* rvm2glb C ABI — RVM (PDMS/E3D) → GLB with streaming I/O + progress.
 *
 * The host drives I/O through callbacks (all receive `user` verbatim):
 *   - read:  positional input read.
 *   - open/write/close: per-output sink. rvm2glb_convert opens one handle per exported
 *     site/level (one .glb each) plus a final "status_file.json", writes it in chunks,
 *     then closes it before opening the next. There is no temp file.
 *   - progress (optional): fires once per .glb written.
 *
 * Link against librvm2glb_capi (cdylib or staticlib). Hand-written to match
 * crates/capi/src/lib.rs; keep in sync (or generate with cbindgen).
 */
#ifndef RVM2GLB_H
#define RVM2GLB_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Read up to `len` bytes at `offset` into `buf`; return bytes read (0 = EOF, <0 = error). */
typedef int64_t (*rvm2glb_read_fn)(void *user, uint64_t offset, uint8_t *buf, size_t len);
/* Open output `name`; return a nonzero handle, or 0 on failure. */
typedef uint64_t (*rvm2glb_open_fn)(void *user, const char *name);
/* Append `len` bytes to `handle`; return 0 on success. */
typedef int (*rvm2glb_write_fn)(void *user, uint64_t handle, const uint8_t *buf, size_t len);
/* Close `handle` (no more writes follow). */
typedef void (*rvm2glb_close_fn)(void *user, uint64_t handle);
/* Per-.glb progress: `name` is the file just written, `nodes` its node count. */
typedef void (*rvm2glb_progress_fn)(void *user, uint32_t output_index, const char *name,
                                    uint32_t nodes);

/* mode: 0 = merged, 1 = instanced, 2 = standard. source_name may be NULL. */
typedef struct {
  uint8_t level;
  int mode;
  bool remove_empty;
  bool cleanup_position;
  uint8_t cleanup_precision;
  float meshopt_threshold;
  float meshopt_target_error;
  float tolerance;
  float line_width;
  /* Include RVM Line primitives. false (the default) skips them entirely —
   * they are numerous and add visual noise. */
  bool include_line;
  bool align_segments;
  bool highlight_instance;
  bool dry_run;
  /* Extract the RVM structure as JSON (<site>.json + base.json) instead of GLB.
   * Overrides `mode`; honours `level`. */
  bool extract_json;
  const char *source_name;
} rvm2glb_options;

/* Convert one RVM input.
 * Returns 0 on success, 1 for bad arguments (NULL callback/options or unknown mode),
 * 2 if the conversion failed. `progress` may be NULL. */
int rvm2glb_convert(void *user, uint64_t input_size, rvm2glb_read_fn read,
                    rvm2glb_open_fn open, rvm2glb_write_fn write, rvm2glb_close_fn close,
                    rvm2glb_progress_fn progress, const rvm2glb_options *opts);

/* ABI version (bumped on any breaking change to this surface). */
uint32_t rvm2glb_capi_abi_version(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RVM2GLB_H */
