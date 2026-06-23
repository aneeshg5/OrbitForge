const AUTO_DISMISS_MS = 6000
const LEAVE_ANIMATION_MS = 180

export type ToastKind = 'error' | 'info' | 'success'

function ensureContainer(): HTMLElement {
  let container = document.getElementById('toast-container')
  if (!container) {
    container = document.createElement('div')
    container.id = 'toast-container'
    document.body.appendChild(container)
  }
  return container
}

export function showToast(message: string, kind: ToastKind = 'error'): void {
  const container = ensureContainer()
  const toast = document.createElement('div')
  toast.className = `toast toast-${kind}`
  toast.textContent = message
  container.appendChild(toast)

  const remove = (): void => {
    toast.classList.add('toast-leaving')
    setTimeout(() => toast.remove(), LEAVE_ANIMATION_MS)
  }
  const timer = setTimeout(remove, AUTO_DISMISS_MS)
  toast.addEventListener('click', () => {
    clearTimeout(timer)
    remove()
  })
}
