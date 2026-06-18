//! Builds the `status_file.json` document (one entry per exported model + warnings +
//! header). Pure: returns a `String` so the same code serves the CLI, capi, and wasm —
//! the caller writes it through an `OutputSink`.

use crate::rvm_parser::{FileMeta, HeaderBlock};
use serde_json::{Value, json};

/// Build the `status_file.json` document. Pure — the caller writes the returned bytes
/// through an `OutputHandle` (so the same code path serves the CLI, wasm/OPFS, capi).
pub fn status_json(filemeta: &[FileMeta], warnings: &[String], header: &HeaderBlock) -> String {
    let models: Vec<Value> = filemeta
        .iter()
        .map(|m| {
            json!({
                "root_name": m.root_name,
                "source_file_name": m.source_file_name,
                "md5": m.md5,
                "glb_md5": m.glb_md5,
                "export_lvl": m.export_lvl,
                "parent": m.parent,
                "parent_hash": m.parent_hash,
                "file_name": m.file_name,
                "min_x": m.bbox.min_x,
                "min_y": m.bbox.min_y,
                "min_z": m.bbox.min_z,
                "max_x": m.bbox.max_x,
                "max_y": m.bbox.max_y,
                "max_z": m.bbox.max_z,
            })
        })
        .collect();

    let doc = json!({
        "models": models,
        "warnings": warnings,
        "header": {
            "date": header.date,
            "encoding": header.encoding,
            "info": header.info,
            "note": header.note,
            "user": header.user,
            "version": header.version,
        }
    });

    doc.to_string()
}
