/// <reference lib="webworker" />
//
// Conversion worker. One throwaway worker per conversion (the parent terminates it
// after, reclaiming the grow-only wasm heap). Streaming (OPFS) only.
//
// The input is staged into OPFS by the parent; here we open a *sync* access handle
// (worker-only) and read it by offset, so the whole file never sits in wasm memory.
// Outputs are posted back to the parent: the core writes one site at a time
// (open→write→close), so we buffer just that one file, then transfer it out on close.
// (We can't create OPFS output handles on demand — `createSyncAccessHandle` is async but
// the core's `open` is called synchronously inside wasm, and the per-site names aren't
// known up front. Buffering one site keeps the streaming memory guarantee anyway.)

import init, { Options, convert_streaming, version } from '../pkg/rvm2glb_wasm.js';
import wasmUrl from '../pkg/rvm2glb_wasm_bg.wasm?url';

const ctx = self as unknown as DedicatedWorkerGlobalScope;
const ready: Promise<string> = init(wasmUrl).then(() => version());

interface OptsDTO {
  mode: number;
  level: number;
  tolerance: number;
  lineWidth: number;
  removeEmpty: boolean;
  highlight: boolean;
  cleanupPosition: boolean;
  cleanupPrecision: number;
  meshoptThreshold: number;
  meshoptTargetError: number;
  alignSegments: boolean;
}

type WorkerRequest =
  | { kind: 'init' }
  | { kind: 'convert'; inputPath: string; sourceName: string; opts: OptsDTO };

type WorkerResponse =
  | { kind: 'ready'; version: string }
  | { kind: 'progress'; outputIndex: number; name: string; nodes: number }
  | { kind: 'file'; name: string; bytes: ArrayBuffer }
  | { kind: 'done'; count: number; info: string; ms: number }
  | { kind: 'error'; error: string };

function post(m: WorkerResponse, transfer: Transferable[] = []): void {
  ctx.postMessage(m, transfer);
}

const root = (): Promise<FileSystemDirectoryHandle> => navigator.storage.getDirectory();

async function openInputSync(path: string): Promise<FileSystemSyncAccessHandle> {
  const fh = await (await root()).getFileHandle(path, { create: false });
  return (fh as unknown as { createSyncAccessHandle(): Promise<FileSystemSyncAccessHandle> })
    .createSyncAccessHandle();
}

function readRange(h: FileSystemSyncAccessHandle, offset: number, len: number): Uint8Array {
  const buf = new Uint8Array(len);
  const n = h.read(buf, { at: offset });
  return n === len ? buf : buf.subarray(0, n);
}

function buildOptions(o: OptsDTO): Options {
  const opt = new Options();
  opt.mode = o.mode;
  opt.level = o.level;
  opt.tolerance = o.tolerance;
  opt.line_width = o.lineWidth;
  opt.remove_empty = o.removeEmpty;
  opt.highlight_instance = o.highlight;
  opt.cleanup_position = o.cleanupPosition;
  opt.cleanup_precision = o.cleanupPrecision;
  opt.meshopt_threshold = o.meshoptThreshold;
  opt.meshopt_target_error = o.meshoptTargetError;
  opt.align_segments = o.alignSegments;
  return opt;
}

function concat(chunks: Uint8Array[]): Uint8Array {
  const total = chunks.reduce((n, c) => n + c.length, 0);
  const all = new Uint8Array(total);
  let o = 0;
  for (const c of chunks) {
    all.set(c, o);
    o += c.length;
  }
  return all;
}

async function runStreaming(
  req: Extract<WorkerRequest, { kind: 'convert' }>,
): Promise<{ count: number; info: string }> {
  const inH = await openInputSync(req.inputPath);
  const names = new Map<number, string>();
  const buffers = new Map<number, Uint8Array[]>();
  let nextId = 0;
  let count = 0;

  const io = {
    size: () => inH.getSize(),
    read: (offset: number, len: number) => readRange(inH, offset, len),
    open: (name: string): number => {
      nextId += 1;
      names.set(nextId, name);
      buffers.set(nextId, []);
      return nextId;
    },
    // wasm reuses its linear memory after the call, so copy the chunk.
    write: (handle: number, bytes: Uint8Array): void => {
      buffers.get(handle)?.push(bytes.slice());
    },
    close: (handle: number): void => {
      const chunks = buffers.get(handle);
      const name = names.get(handle);
      if (!chunks || name === undefined) return;
      buffers.delete(handle);
      names.delete(handle);
      const all = concat(chunks);
      count += 1;
      post({ kind: 'file', name, bytes: all.buffer }, [all.buffer]);
    },
    progress: (outputIndex: number, name: string, nodes: number) =>
      post({ kind: 'progress', outputIndex, name, nodes }),
  };

  try {
    const info = convert_streaming(io as never, buildOptions(req.opts), req.sourceName);
    return { count, info };
  } finally {
    inH.close();
  }
}

ctx.onmessage = async (e: MessageEvent<WorkerRequest>) => {
  const req = e.data;
  try {
    const v = await ready;
    if (req.kind === 'init') {
      post({ kind: 'ready', version: v });
      return;
    }
    const t0 = performance.now();
    const { count, info } = await runStreaming(req);
    post({ kind: 'done', count, info, ms: Math.round(performance.now() - t0) });
  } catch (err) {
    post({ kind: 'error', error: err instanceof Error ? err.message : String(err) });
  }
};
