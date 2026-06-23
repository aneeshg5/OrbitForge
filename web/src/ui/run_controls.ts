import type { ScenarioConfig } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'

export interface RunControlsOptions {
  postToWorker: (msg: WorkerRequest) => void
  getConfig: () => ScenarioConfig | undefined
  getRunDurationSec: () => number | undefined
  getSimSpeed: () => number
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
  private readonly getRunDurationSec: () => number | undefined
  private readonly getSimSpeed: () => number
  private readonly toggleButton: HTMLButtonElement
  private readonly resetButton: HTMLButtonElement
  private state: RunState = 'idle'
  private runEnabled = false
  private lastAutoStopAtSec: number | undefined

  constructor(container: HTMLElement, options: RunControlsOptions) {
    this.postToWorker = options.postToWorker
    this.getConfig = options.getConfig
    this.getRunDurationSec = options.getRunDurationSec
    this.getSimSpeed = options.getSimSpeed

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
      this.lastAutoStopAtSec = undefined
    } else {
      this.postToWorker({ type: 'set_sim_speed', payload: { simSpeed: this.getSimSpeed() } })
    }
    this.postToWorker({ type: 'start' })
    this.state = 'running'
    this.render()
  }

  private onReset(): void {
    this.postToWorker({ type: 'reset' })
    this.state = 'idle'
    this.lastAutoStopAtSec = undefined
    this.render()
  }

  setRunEnabled(enabled: boolean): void {
    this.runEnabled = enabled
    this.render()
  }

  checkAutoStop(simTimeSec: number): void {
    if (this.state !== 'running') return
    const target = this.getRunDurationSec()
    if (target === undefined || target <= 0) return
    if (this.lastAutoStopAtSec !== undefined && target <= this.lastAutoStopAtSec) return
    if (simTimeSec >= target) {
      this.lastAutoStopAtSec = target
      this.postToWorker({ type: 'pause' })
      this.state = 'paused'
      this.render()
    }
  }

  private render(): void {
    this.toggleButton.classList.remove('state-idle', 'state-running', 'state-paused')
    this.toggleButton.classList.add(`state-${this.state}`)
    const [icon, label] =
      this.state === 'running' ? ['⏸', 'Pause'] : this.state === 'paused' ? ['▶', 'Continue'] : ['▶', 'Run']
    setButtonContent(this.toggleButton, icon, label)

    const disabled = this.state === 'idle' && !this.runEnabled
    this.toggleButton.disabled = disabled
    this.toggleButton.title = disabled ? 'Select a satellite or paste a TLE first' : ''
  }
}
