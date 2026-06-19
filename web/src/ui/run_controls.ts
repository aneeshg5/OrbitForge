// Topbar Run/Pause/Reset controls. Run and Pause are a single toggle
// button rather than two buttons, since the two actions are never both
// valid at once — the button's label and color reflect which action it
// will take next, not the imperative name of a fixed control.

import type { ScenarioConfig } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'

export interface RunControlsOptions {
  postToWorker: (msg: WorkerRequest) => void
  getConfig: () => ScenarioConfig | undefined
}

type RunState = 'idle' | 'running' | 'paused'

function el<K extends keyof HTMLElementTagNameMap>(tag: K, className?: string): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag)
  if (className) e.className = className
  return e
}

function setButtonContent(button: HTMLButtonElement, icon: string, label: string): void {
  button.replaceChildren()
  const iconSpan = el('span', 'btn-icon')
  iconSpan.textContent = icon
  button.append(iconSpan, document.createTextNode(` ${label}`))
}

export class RunControls {
  private readonly postToWorker: (msg: WorkerRequest) => void
  private readonly getConfig: () => ScenarioConfig | undefined
  private readonly toggleButton: HTMLButtonElement
  private readonly resetButton: HTMLButtonElement
  private state: RunState = 'idle'

  constructor(container: HTMLElement, options: RunControlsOptions) {
    this.postToWorker = options.postToWorker
    this.getConfig = options.getConfig

    const root = el('div', 'run-controls')
    this.toggleButton = el('button', 'run-toggle-btn')
    this.resetButton = el('button', 'reset-btn')
    setButtonContent(this.resetButton, '⟳', 'Reset')
    root.append(this.toggleButton, this.resetButton)
    container.appendChild(root)

    this.toggleButton.addEventListener('click', () => this.onToggle())
    this.resetButton.addEventListener('click', () => this.onReset())

    this.render()
  }

  private onToggle(): void {
    if (this.state === 'running') {
      this.postToWorker({ type: 'pause' })
      this.state = 'paused'
      this.render()
      return
    }

    if (this.state === 'idle') {
      const cfg = this.getConfig()
      if (!cfg) return
      this.postToWorker({ type: 'init', payload: cfg })
    }
    this.postToWorker({ type: 'start' })
    this.state = 'running'
    this.render()
  }

  private onReset(): void {
    this.postToWorker({ type: 'reset' })
    this.state = 'idle'
    this.render()
  }

  private render(): void {
    this.toggleButton.classList.remove('state-idle', 'state-running', 'state-paused')
    this.toggleButton.classList.add(`state-${this.state}`)
    const [icon, label] =
      this.state === 'running' ? ['⏸', 'Pause'] : this.state === 'paused' ? ['▶', 'Continue'] : ['▶', 'Run']
    setButtonContent(this.toggleButton, icon, label)
  }
}
