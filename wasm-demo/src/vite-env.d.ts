/// <reference types="vite/client" />

// wasm-bindgen output imported as a URL (Vite `?url` suffix).
declare module '*.wasm?url' {
  const src: string;
  export default src;
}
