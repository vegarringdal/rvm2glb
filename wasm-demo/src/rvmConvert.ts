// Main-thread controller: spawns a throwaway worker per conversion, stages the input
// into OPFS, relays per-site progress, and collects the output files the worker posts
// back. Streaming (OPFS) is the only path. Modelled on step2glb's stepConvert.ts.

export interface ConvertOpts {
  mode: number; // 0 merged, 1 instanced, 2 standard
  level: number;
  tolerance: number;
  includeLine: boolean;
  lineWidth: number;
  removeEmpty: boolean;
  highlight: boolean;
  cleanupPosition: boolean;
  cleanupPrecision: number;
  meshoptThreshold: number;
  meshoptTargetError: number;
  alignSegments: boolean;
}

export interface ConvertProgress {
  outputIndex: number;
  name: string;
  nodes: number;
}

export interface OutputFile {
  name: string;
  blob: Blob;
}

export interface ConvertResult {
  files: OutputFile[];
  info: string;
  ms: number;
}

interface Hooks {
  onStatus?: (msg: string) => void;
  onProgress?: (p: ConvertProgress) => void;
}

type WorkerRequest =
  | { kind: 'init' }
  | { kind: 'convert'; inputPath: string; sourceName: string; opts: ConvertOpts };

type WorkerResponse =
  | { kind: 'ready'; version: string }
  | { kind: 'progress'; outputIndex: number; name: string; nodes: number }
  | { kind: 'file'; name: string; bytes: ArrayBuffer }
  | { kind: 'done'; count: number; info: string; ms: number }
  | { kind: 'error'; error: string };

export class RvmConverter {
  private root(): Promise<FileSystemDirectoryHandle> {
    return navigator.storage.getDirectory();
  }

  private spawn(): Worker {
    return new Worker(new URL('./rvmConvertWorker.ts', import.meta.url), { type: 'module' });
  }

  /** Probe the wasm version (also confirms the module loads). */
  version(): Promise<string> {
    return new Promise((resolve, reject) => {
      const w = this.spawn();
      w.onmessage = (e: MessageEvent<WorkerResponse>) => {
        const m = e.data;
        w.terminate();
        if (m.kind === 'ready') resolve(m.version);
        else reject(new Error(m.kind === 'error' ? m.error : 'unexpected'));
      };
      w.onerror = (e) => {
        w.terminate();
        reject(new Error(e.message));
      };
      w.postMessage({ kind: 'init' } satisfies WorkerRequest);
    });
  }

  async convert(file: File, opts: ConvertOpts, hooks: Hooks = {}): Promise<ConvertResult> {
    hooks.onStatus?.('staging input to OPFS…');
    const inputPath = await this.stage(file);
    const req: WorkerRequest = { kind: 'convert', inputPath, sourceName: file.name, opts };

    hooks.onStatus?.('converting…');
    const files: OutputFile[] = [];
    try {
      return await new Promise<ConvertResult>((resolve, reject) => {
        const w = this.spawn();
        const finish = (r: ConvertResult | Error) => {
          w.terminate();
          if (r instanceof Error) reject(r);
          else resolve(r);
        };
        w.onmessage = (e: MessageEvent<WorkerResponse>) => {
          const m = e.data;
          switch (m.kind) {
            case 'progress':
              hooks.onProgress?.({ outputIndex: m.outputIndex, name: m.name, nodes: m.nodes });
              break;
            case 'file':
              files.push({ name: m.name, blob: new Blob([m.bytes]) });
              break;
            case 'done':
              finish({ files, info: m.info, ms: m.ms });
              break;
            case 'error':
              finish(new Error(m.error));
              break;
            default:
              break;
          }
        };
        w.onerror = (e) => finish(new Error(e.message || String(e)));
        w.postMessage(req);
      });
    } finally {
      await this.remove(inputPath);
    }
  }

  // ── OPFS staging (main-thread async writable) ──────────────────────────────

  private async stage(file: File): Promise<string> {
    const name = `${crypto.randomUUID()}.rvm`;
    const fh = await (await this.root()).getFileHandle(name, { create: true });
    const writable = await fh.createWritable();
    await file.stream().pipeTo(writable);
    return name;
  }

  private async remove(name: string): Promise<void> {
    try {
      await (await this.root()).removeEntry(name);
    } catch {
      /* already gone */
    }
  }
}
