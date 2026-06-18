// Streaming ZIP via fflate. A model can have 150+ sites, so we can't fire that many
// individual downloads — we bundle every output into one archive instead.
//
// `AsyncZipDeflate` compresses each entry on fflate's own worker pool (off the main
// thread, so the UI doesn't freeze), and `Zip` streams the archive out chunk-by-chunk as
// entries finish. Source blobs are read one at a time (sequential await) to bound peak
// memory, and the output chunks become a `Blob` (browser-managed, can spill to disk) —
// no single giant contiguous allocation.

import { AsyncZipDeflate, Zip } from 'fflate';

export interface ZipInput {
  name: string;
  blob: Blob;
}

/** Bundle `files` into a single ZIP blob (deflate, streamed off-thread). */
export function zipBlobs(files: ZipInput[]): Promise<Blob> {
  return new Promise<Blob>((resolve, reject) => {
    const chunks: Uint8Array[] = [];
    const zip = new Zip((err, chunk, final) => {
      if (err) {
        reject(err);
        return;
      }
      if (chunk.length) chunks.push(chunk);
      // fflate yields Uint8Array<ArrayBufferLike>; Blob wants ArrayBuffer-backed parts.
      if (final) resolve(new Blob(chunks as unknown as BlobPart[], { type: 'application/zip' }));
    });

    (async () => {
      try {
        for (const f of files) {
          const entry = new AsyncZipDeflate(f.name, { level: 6 });
          zip.add(entry);
          // read one source blob at a time, then hand it to fflate
          entry.push(new Uint8Array(await f.blob.arrayBuffer()), true);
        }
        zip.end();
      } catch (e) {
        reject(e as Error);
      }
    })();
  });
}
