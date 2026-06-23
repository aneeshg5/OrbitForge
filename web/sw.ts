const CACHE_NAME = 'orbitforge-v1'
const PRECACHE_URLS = ['/', '/index.html', '/manifest.json', '/orbitforge.js', '/orbitforge.wasm']

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
