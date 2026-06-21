// Topbar Run/Pause/Reset controls. Run and Pause are a single toggle
// button rather than two buttons, since the two actions are never both
// valid at once — the button's label and color reflect which action it
// will take next, not the imperative name of a fixed control.

import type { ScenarioConfig } from '../bridge/wasm_types.js'
import type { WorkerRequest } from '../worker.js'

export interface RunControlsOptions {
  postToWorker: (msg: WorkerRequest) => void
  getConfig: () => ScenarioConfig | undefined
  // Target simulated time (seconds, matching the T+ clock since the last
  // Reset) to auto-pause at — undefined/0 means run indefinitely, the
  // pre-existing behavior. Read fresh each call rather than passed once,
  // same pattern as getConfig: ScenarioEditor's duration input can change
  // mid-run without RunControls needing to be told explicitly.
  getRunDurationSec: () => number | undefined
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
  private readonly toggleButton: HTMLButtonElement
  private readonly resetButton: HTMLButtonElement
  private state: RunState = 'idle'
  // Whether ScenarioEditor.getConfig() would currently succeed — gates the
  // toggle button only while idle (once running/paused, a config was
  // already consumed, so pause/continue must stay clickable regardless).
  // Starts false so the button is visibly disabled instead of silently
  // doing nothing on a click before the first TLE has loaded.
  private runEnabled = false
  // Target (seconds) the last auto-stop fired at, so clicking Continue
  // doesn't immediately re-trigger it — simTime is still past that target
  // on the very next frame. Cleared on Reset/fresh Run so a new run is
  // governed by the duration again; re-armed automatically if the user
  // raises the duration value past this while paused.
  private lastAutoStopAtSec: number | undefined

  constructor(container: HTMLElement, options: RunControlsOptions) {
    this.postToWorker = options.postToWorker
    this.getConfig = options.getConfig
    this.getRunDurationSec = options.getRunDurationSec

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

  /** Called by ScenarioEditor whenever getConfig() availability changes. */
  setRunEnabled(enabled: boolean): void {
    this.runEnabled = enabled
    this.render()
  }

  /**
   * Called every render frame with the current T+ value (simTime, seconds
   * since the last Reset). Auto-pauses once it reaches the configured run
   * duration, the same transition onToggle() makes for a manual Pause
   * click — so a duration-stopped run can be resumed with Continue just
   * like a manually-paused one.
   */
  checkAutoStop(simTimeSec: number): void {
    if (this.state !== 'running') return
    const target = this.getRunDurationSec()
    if (target === undefined || target <= 0) return
    // Already stopped at this (or a later) target — Continue should carry
    // on past it, not get stuck re-pausing every frame.
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
