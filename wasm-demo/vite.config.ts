import { defineConfig } from 'vite';

// OPFS sync access handles + ES-module Workers; allow importing the generated ../pkg.
export default defineConfig({
  base: process.env.VITE_BASE || '/',
  server: { fs: { allow: ['..'] } },
  optimizeDeps: { exclude: ['./pkg/rvm2glb_wasm.js'] },
  worker: { format: 'es' },
  build: { target: 'esnext' },
});
