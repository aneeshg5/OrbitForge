// Service worker: caches the WASM binary, glue JS, and static shell so the
// app keeps working offline after the first load (CLAUDE.md §3's "works
// offline after first load" claim). Cache-first for the WASM/JS/texture
// assets (they're content-addressed by version below, not by URL hashing,
// so a CACHE_NAME bump is what invalidates them) and network-first for
// everything else.

const CACHE_NAME = 'orbitforge-v1'
const PRECACHE_URLS = ['/', '/index.html', '/manifest.json', '/orbitforge.js', '/orbitforge.wasm']

// lib.dom.d.ts (this project's configured lib — see tsconfig.json) doesn't
// know about service-worker-specific event types (ExtendableEvent,
// FetchEvent), and adding lib.webworker.d.ts alongside it would conflict
// over the global `self`/`addEventListener` declarations. Casting locally
// avoids needing a second tsconfig just for this one file.
interface ExtendableEventLike extends Event {
  waitUntil(promise: Promise<unknown>): void
}
interface FetchEventLike extends Event {
  readonly request: Request
  respondWith(response: Promise<Response> | Response): void
}

self.addEventListener('install', (event) => {
  const e = event as ExtendableEventLike
  e.waitUntil(caches.open(CACHE_NAME).then((cache) => cache.addAll(PRECACHE_URLS)))
})

self.addEventListener('activate', (event) => {
  const e = event as ExtendableEventLike
  e.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((key) => key !== CACHE_NAME).map((key) => caches.delete(key))),
    ),
  )
})

self.addEventListener('fetch', (event) => {
  const e = event as FetchEventLike
  e.respondWith(
    caches.match(e.request).then((cached) => {
      if (cached) return cached
      return fetch(e.request).then((response) => {
        if (!response.ok) return response
        const responseClone = response.clone()
        void caches.open(CACHE_NAME).then((cache) => cache.put(e.request, responseClone))
        return response
      })
    }),
  )
})
