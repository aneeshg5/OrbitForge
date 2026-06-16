import { defineConfig } from 'vite'

export default defineConfig({
  server: {
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
  build: {
    target: 'es2022',
    assetsInlineLimit: 0,
    rollupOptions: {
      input: {
        main: 'index.html',
        sw: 'sw.ts',
      },
      output: {
        // sw.js must land at a predictable top-level path — service workers
        // are registered by URL (main.ts registers '/sw.js'), not imported,
        // so it can't get Rollup's default hashed chunk name.
        entryFileNames: (chunk) => (chunk.name === 'sw' ? 'sw.js' : 'assets/[name]-[hash].js'),
      },
    },
  },
  assetsInclude: ['**/*.wasm'],
})
