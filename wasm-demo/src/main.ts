import { RvmConverter, type ConvertOpts, type OutputFile } from './rvmConvert';
import { zipBlobs } from './zip';

const $ = <T extends HTMLElement>(id: string): T => document.getElementById(id) as T;

const statusEl = $('status');
const sourceEl = $('source');
const filesEl = $('files');
const fileInput = $<HTMLInputElement>('file');
const viewer = $('viewer');
const emptyEl = $('empty');
const downloadAll = $<HTMLButtonElement>('downloadAll');

const converter = new RvmConverter();
let current: OutputFile[] = [];
let viewerUrl: string | null = null;

// ── panel toggles ────────────────────────────────────────────────────────────
const toggle = (cls: string) => () => document.body.classList.toggle(cls);
$('menuToggle').addEventListener('click', toggle('config-open'));
$('infoToggle').addEventListener('click', toggle('info-open'));
$('configClose').addEventListener('click', () => document.body.classList.remove('config-open'));
$('infoClose').addEventListener('click', () => document.body.classList.remove('info-open'));
$('backdrop').addEventListener('click', () =>
  document.body.classList.remove('config-open', 'info-open'),
);

// ── live range read-outs ──────────────────────────────────────────────────────
const bindRange = (id: string, out: string, fmt: (v: number) => string) => {
  const el = $<HTMLInputElement>(id);
  const o = $(out);
  const update = () => (o.textContent = fmt(Number(el.value)));
  el.addEventListener('input', update);
  update();
};
bindRange('tolerance', 'tolVal', (v) => v.toFixed(3));
bindRange('lineWidth', 'lwVal', (v) => v.toFixed(2));
bindRange('meshoptThreshold', 'mtVal', (v) => v.toFixed(2));
bindRange('meshoptTargetError', 'meVal', (v) => v.toFixed(3));

// ── version probe ─────────────────────────────────────────────────────────────
converter
  .version()
  .then((v) => (statusEl.textContent = `rvm2glb ${v} — ready`))
  .catch((e) => (statusEl.textContent = `failed to load wasm: ${e.message}`));

// Inputs disabled while a conversion is running (incl. the file picker + mode select).
const CONTROLS = [
  'file', 'mode', 'level', 'tolerance', 'lineWidth', 'removeEmpty', 'highlight',
  'alignSegments', 'cleanupPosition', 'cleanupPrecision', 'meshoptThreshold', 'meshoptTargetError',
];

function setBusy(busy: boolean): void {
  for (const id of CONTROLS) $<HTMLInputElement>(id).disabled = busy;
  statusEl.classList.toggle('busy', busy); // yellow while converting
  if (busy) downloadAll.disabled = true;
}

function readOpts(): ConvertOpts {
  return {
    mode: Number($<HTMLSelectElement>('mode').value),
    level: Number($<HTMLInputElement>('level').value),
    tolerance: Number($<HTMLInputElement>('tolerance').value),
    lineWidth: Number($<HTMLInputElement>('lineWidth').value),
    removeEmpty: $<HTMLInputElement>('removeEmpty').checked,
    highlight: $<HTMLInputElement>('highlight').checked,
    cleanupPosition: $<HTMLInputElement>('cleanupPosition').checked,
    cleanupPrecision: Number($<HTMLInputElement>('cleanupPrecision').value),
    meshoptThreshold: Number($<HTMLInputElement>('meshoptThreshold').value),
    meshoptTargetError: Number($<HTMLInputElement>('meshoptTargetError').value),
    alignSegments: $<HTMLInputElement>('alignSegments').checked,
  };
}

function view(file: OutputFile): void {
  if (viewerUrl) URL.revokeObjectURL(viewerUrl);
  viewerUrl = URL.createObjectURL(file.blob);
  viewer.setAttribute('src', viewerUrl);
  emptyEl.style.display = 'none';
  for (const n of filesEl.querySelectorAll('.name')) n.classList.remove('active');
  document.getElementById(`name-${file.name}`)?.classList.add('active');
}

function download(file: OutputFile): void {
  const url = URL.createObjectURL(file.blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = file.name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 4000);
}

function renderFiles(files: OutputFile[]): void {
  filesEl.replaceChildren();
  for (const f of files) {
    const row = document.createElement('div');
    row.className = 'file-row';

    const meta = document.createElement('div');
    meta.className = 'meta';
    const name = document.createElement('div');
    name.className = 'name';
    name.id = `name-${f.name}`;
    name.textContent = f.name;
    const size = document.createElement('div');
    size.className = 'size';
    size.textContent = `${(f.blob.size / 1e6).toFixed(2)} MB`;
    meta.append(name, size);
    row.append(meta);

    if (f.name.toLowerCase().endsWith('.glb')) {
      const v = document.createElement('button');
      v.className = 'btn btn-primary';
      v.textContent = 'View';
      v.addEventListener('click', () => view(f));
      row.append(v);
    }
    const d = document.createElement('button');
    d.className = 'btn btn-ghost';
    d.textContent = 'Download';
    d.addEventListener('click', () => download(f));
    row.append(d);

    filesEl.append(row);
  }
}

fileInput.addEventListener('change', async () => {
  const file = fileInput.files?.[0];
  if (!file) return;
  sourceEl.textContent = file.name;
  filesEl.replaceChildren();
  current = [];
  setBusy(true);
  try {
    const result = await converter.convert(file, readOpts(), {
      onStatus: (m) => (statusEl.textContent = `${file.name}: ${m}`),
      onProgress: ({ outputIndex, name, nodes }) =>
        (statusEl.textContent = `${file.name}: #${outputIndex + 1} ${name} (${nodes} nodes)…`),
    });
    current = result.files;
    renderFiles(result.files);
    document.body.classList.add('info-open');
    const total = result.files.reduce((n, f) => n + f.blob.size, 0);
    statusEl.textContent = `${file.name}: ${result.files.length} file(s), ${(total / 1e6).toFixed(2)} MB in ${result.ms} ms`;
    // auto-view the first GLB
    const firstGlb = result.files.find((f) => f.name.toLowerCase().endsWith('.glb'));
    if (firstGlb) view(firstGlb);
  } catch (e) {
    statusEl.textContent = `${file.name}: error — ${(e as Error).message}`;
  } finally {
    setBusy(false);
    downloadAll.disabled = current.length === 0;
    fileInput.value = '';
  }
});

// Bundle every output into one ZIP — a model can have 150+ sites, and browsers block
// firing that many individual downloads.
downloadAll.addEventListener('click', async () => {
  if (current.length === 0) return;
  if (current.length === 1) {
    download(current[0]);
    return;
  }
  const prev = statusEl.textContent;
  downloadAll.disabled = true;
  statusEl.classList.add('busy');
  statusEl.textContent = `zipping ${current.length} files…`;
  try {
    const blob = await zipBlobs(current.map((f) => ({ name: f.name, blob: f.blob })));
    const base = (sourceEl.textContent || 'rvm2glb').replace(/\.[^.]*$/, '');
    download({ name: `${base}.zip`, blob });
    statusEl.textContent = prev;
  } catch (e) {
    statusEl.textContent = `zip failed — ${(e as Error).message}`;
  } finally {
    statusEl.classList.remove('busy');
    downloadAll.disabled = false;
  }
});
