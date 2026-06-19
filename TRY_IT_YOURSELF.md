# Trying Out OrbitForge Yourself

This is a step-by-step guide to running OrbitForge on your own computer and
clicking on every single thing it can do, so you can decide what you like
and what needs to change before we worry about putting it on the internet.

No assumed knowledge. If a step says to type something, type it exactly,
press Enter, and look for the "what you should see" line right after it.

---

## Part 1 — Turning it on

### Step 1: Open a terminal

On a Mac, open the **Terminal** app (or you can use the terminal built into
your editor, if you have one open).

### Step 2: Go to the project folder

Type this and press Enter:

```bash
cd "/Users/aneeshg/Desktop/Personal Projects/orbitforge/web"
```

**What you should see:** nothing bad happens — your terminal prompt just
shows you're now "inside" the `web` folder.

### Step 3: Start the app

Type this and press Enter:

```bash
npm run dev
```

**What you should see:** a few lines of text ending in something like:

```
➜  Local:   http://localhost:5173/
```

That `http://localhost:5173/` is an address, just like a website address,
except it only works on your own computer. Leave this terminal window
open and running — closing it turns the app off.

### Step 4: Open it in your browser

Copy `http://localhost:5173/` and paste it into Chrome (Chrome works best
for this kind of app). Press Enter.

**What you should see:** a dark screen with a glowing blue Earth in the
middle, a starfield behind it, four small chart boxes on the right, and
some controls along the bottom. If you see that, it worked — keep going.

If instead the page is blank or shows an error, stop here and tell me
exactly what you see — don't try to fix it yourself.

---

## Part 2 — What you're looking at

Before you start clicking, here's what each part of the screen means, in
plain terms:

- **The glowing Earth (big area, left/center):** This is a 3D model of
  Earth, drawn live by your graphics card. Once you press "Run," a
  satellite will fly around it and you'll see colored lines tracing its
  orbit.
- **The four small charts (top right):** These update live, many times per
  second, while the simulation runs. Each one is comparing three different
  "guessing methods" (called filters — KF, EKF, UKF) at figuring out where
  the satellite actually is, using only noisy, imperfect sensor data — kind
  of like three people guessing where a moving target is, given blurry
  binoculars.
- **Scenario box (bottom left):** Lets you pick which real satellite to
  simulate, and tweak how noisy the pretend sensors are.
- **Fault Injection box (bottom middle):** Buttons that intentionally
  "break" something during the simulation — like jamming the GPS or
  giving the satellite an unexpected nudge — so you can watch how each
  guessing method reacts.
- **Monte Carlo (bottom, collapsed):** A "run it many times and check the
  statistics" mode, instead of watching one single run.

---

## Part 3 — Click through every feature

Go in this order. After each action, just look at what changed and form
an opinion — "this looks cool," "this looks broken," "I don't understand
this," anything is useful feedback.

### 3.1 — Pick a satellite

In the **Scenario** box, find the dropdown next to "Satellite:". Click it.

**What you should see:** a list of 5 real satellites (ISS, STARLINK-1008,
GPS BIIR-2, GOES-16, COSMOS 2251 DEB), each with a short description of
why it's interesting. Pick a different one than what's selected — try
"GOES-16" since it orbits much farther out.

**What's happening:** the app is fetching real, live orbital data for
that satellite from a public satellite-tracking website
(CelesTrak) — not made-up numbers.

### 3.2 — Try pasting your own satellite (optional)

There's a text box below the dropdown that says "Paste a 2-line TLE here."
You can skip this — it's for advanced users who have a specific
satellite's two-line orbital data to paste in. Just know it's there.

### 3.3 — Move the "GPS σ" slider

This controls how *inaccurate* the pretend GPS sensor is, in meters.
Slide it from 10 up toward 100.

**What's happening:** you're telling the simulation "pretend our GPS is
worse than usual." You won't see a visual change yet — this only matters
once you press Run.

### 3.4 — Move the "Sim speed" slider

This controls how fast time passes in the simulation once it's running.
1x = real time, higher = fast-forward. Leave it low for your first run so
you can actually watch what happens.

### 3.5 — Toggle the checkboxes: J2, Drag, SRP

These turn on/off different real physical effects acting on the
satellite:
- **J2** = Earth isn't a perfect sphere, it bulges at the equator, which
  tugs satellites slightly.
- **Drag** = thin air at low altitude slows the satellite down over time.
- **SRP** = sunlight itself pushes very slightly on the satellite.

Leave J2 and Drag checked (they're on by default), try checking SRP too.

### 3.6 — Press "▶ Run"

**What you should see:**
- The status text in the top-right corner changes to "running."
- A line of small text under the Run/Pause/Reset buttons says something
  like "Running." or shows the satellite name.
- The four charts on the right start drawing live, moving lines.
- (You may or may not see an orbit line drawn around the Earth depending
  on camera angle — this is one thing to give feedback on if it's not
  visible or hard to see.)

Let it run for 10–15 seconds and just watch the charts. They should be
small wiggly lines, not flat at zero and not enormous spikes.

### 3.7 — Press "⏸ Pause," then "▶ Run" again, then "⟳ Reset"

**What you should see:** Pause freezes everything (charts stop moving).
Run again resumes it. Reset should snap the satellite and all the charts
back to the very beginning, as if you just loaded the page.

### 3.8 — Try each Fault Injection button

Press **Run** again first if you pressed Reset. Then, one at a time, try:

- **GPS Spike** — pretends the GPS sensor gave one wildly wrong reading.
  Watch the Position Error chart — you should see a brief jump.
- **GPS Dropout (60s)** — pretends GPS goes dark for 60 simulated seconds.
  Watch the Covariance Trace chart — it should climb (the filters get
  less "sure" of themselves without GPS).
- **Maneuver (5 m/s)** — pretends the satellite secretly fired a thruster.
  The filters don't know this happened — watch for a temporary spike in
  error that should recover.
- **Drag Coeff Error (+50%)** — pretends our drag estimate is wrong by
  50%. This causes a slow, gradual drift rather than a sudden jump.
- **Sensor Bias** — adds a persistent offset to the accelerometer sensor.

After clicking one, the button should visually show it's "active" (a
highlighted border). Watch the charts react over the next several
seconds. This is the single most "wow" feature of the app — this is
where you should spend the most time looking closely.

### 3.9 — Open the Monte Carlo panel

At the very bottom of the screen, click on "▸ Monte Carlo" to expand it.

**What you should see:** a "Runs:" slider (defaults to 500), and a
"▶ Run MC" button.

### 3.10 — Press "▶ Run MC"

This does NOT use the live Run/Pause simulation above — it's a separate,
one-shot experiment: run the *entire* simulated scenario hundreds of times
in a row, each with different random sensor noise, and check whether the
filters are honest about their own uncertainty.

**What you should see:** after a few seconds, it says "Done — 500 runs,
500 steps" and shows:
- A bar-chart histogram of final position error across all 500 runs.
- A small table of RMS (typical) error at a few points in time.
- Two more live-style charts: "NEES" and "NIS" — these are the
  "is the filter being honest about its own uncertainty" scores. Good
  values for NEES hover near 6, and for NIS near 3 (the app actually
  shows dashed bound lines on these charts — the wiggly line should
  mostly stay between the dashed bounds, not constantly poke outside
  them).

You can change the "Runs" slider to a smaller number like 100 and run it
again if you want it to finish faster.

---

## Part 4 — Things you can safely ignore

- If you open your browser's developer console (not necessary, but if you
  do), you might see a warning about a manifest icon. That's a known,
  harmless cosmetic quirk — not a real bug.
- A missing favicon (the little tab icon) 404 error is also harmless.

## Part 5 — When you're done

Go back to the terminal window from Step 3 and press `Control + C` to stop
the app. It's safe to close the terminal after that.

---

## Part 6 — Giving feedback

As you go through Part 3, jot down anything that:
- Looks visually broken, confusing, or ugly
- Doesn't behave the way the description here said it would
- You don't understand the purpose of
- You wish existed but doesn't
- You really like and want to make sure we don't break later

Bring all of that back and we'll go through it together before touching
deployment.
