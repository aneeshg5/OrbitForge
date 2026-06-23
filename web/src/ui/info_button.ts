function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

const INFO_POPOVER_WIDTH = 200
const INFO_POPOVER_MARGIN = 8

export function makeInfoButton(explanation: string, opts?: { width?: number; example?: string }): HTMLElement {
  const width = opts?.width ?? INFO_POPOVER_WIDTH
  const wrapper = el('span', 'info-btn-wrapper')
  const button = el('button', 'info-btn')
  button.type = 'button'
  button.textContent = 'i'
  button.setAttribute('aria-label', 'More info')
  const popover = el('div', 'info-popover')
  popover.style.width = `${width}px`
  const text = el('div')
  text.textContent = explanation
  popover.appendChild(text)
  if (opts?.example) {
    const pre = el('pre', 'info-popover-example')
    pre.textContent = opts.example
    popover.appendChild(pre)
  }
  popover.hidden = true
  document.body.appendChild(popover)

  function positionPopover(): void {
    const rect = button.getBoundingClientRect()
    const left = Math.min(
      Math.max(rect.right - width, INFO_POPOVER_MARGIN),
      window.innerWidth - width - INFO_POPOVER_MARGIN,
    )
    popover.style.top = `${rect.bottom + 6}px`
    popover.style.left = `${left}px`
  }

  button.addEventListener('click', (e) => {
    e.stopPropagation()
    popover.hidden = !popover.hidden
    if (!popover.hidden) positionPopover()
  })
  document.addEventListener('click', (e) => {
    if (e.target !== button) popover.hidden = true
  })
  window.addEventListener('resize', () => {
    if (!popover.hidden) positionPopover()
  })
  wrapper.append(button)
  return wrapper
}
