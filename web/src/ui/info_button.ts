// Small "(i)" button next to a label/control that toggles a plain-language
// explanation on click — for parameters and actions that aren't
// self-explanatory to someone who isn't already a GNC/filters person.
// Click-to-toggle rather than a native title= tooltip since the request
// was specifically for something a user can click, not just hover.
//
// Positioned via fixed coordinates computed in JS, not CSS-relative to the
// trigger button — #controls (the sidebar) has overflow-y:auto, which per
// the CSS spec also forces overflow-x to auto, so a plain position:absolute
// popover gets silently clipped whenever its trigger isn't right at the
// panel's edge (confirmed: this broke J2/Drag/SRP, which sit mid-row, while
// GPS/sim speed happened to work only because their buttons are at the
// row's far right). position:fixed escapes that clipping entirely;
// clamping against window.innerWidth keeps it on-screen regardless of
// where the trigger sits.

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

const INFO_POPOVER_WIDTH = 200
const INFO_POPOVER_MARGIN = 8 // min gap kept from the viewport edge when clamping

// `example`, when given, is appended as a separate monospace block below
// the prose — for showing the literal format of something (a TLE) rather
// than just describing it in words.
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
