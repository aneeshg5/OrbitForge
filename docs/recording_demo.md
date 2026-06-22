# How to record a demo GIF for the README

This walks through recording OrbitForge running and turning that into a GIF
for the README. No special software experience needed. Just follow the
steps in order.

## What you need

- A Mac (you already have one)
- About 10 minutes
- The OrbitForge dev server running locally (`npm run dev` inside `web/`)

## Step 1: Decide what to show

Keep it short. 10 to 20 seconds is plenty. A good demo:

1. Page loads with a satellite already picked
2. Click **Run**
3. Let it run for a few seconds so the orbit path and charts start moving
4. Click a fault button, like **GPS Spike**, so something visibly changes

That's it. Don't try to show every feature in one clip.

## Step 2: Record your screen

Macs have a built in screen recorder. You don't need to install anything
for this part.

1. Press **Cmd + Shift + 5**
2. A small toolbar appears at the bottom of the screen
3. Click **Record Selected Portion**
4. Drag a box around just the browser window showing OrbitForge. Not your
   whole screen, not your menu bar, not your dock.
5. Click **Record**
6. Do the steps from Step 1
7. Press **Cmd + Shift + 5** again and click **Stop**, or click the stop
   icon in the menu bar at the top of the screen
8. The recording saves to your Desktop as a `.mov` file

## Step 3: Turn the video into a GIF

A GIF is a video that loops on its own and shows up directly in the
README, so nobody has to click play.

1. Download [Gifski](https://gif.ski). It's free and made for exactly this.
2. Open Gifski
3. Drag your `.mov` file from the Desktop into the Gifski window
4. Set the width to about **800** pixels. This keeps the file size small.
5. Click **Save GIF**
6. Save it to your Desktop for now

## Step 4: Check the file size

1. Find the GIF in Finder
2. Right click it and choose **Get Info**
3. Look at the size. Try to keep it under **8 MB**
4. If it's too big, go back to Gifski, lower the width to 600, or trim the
   video shorter in Step 2, then save again

## Step 5: Add it to the project

1. Rename the file to `demo.gif`
2. Move it into the `docs/` folder in the OrbitForge project. In Finder,
   drag it into `docs/demo.gif`. Or in Terminal:

   ```bash
   mv ~/Desktop/demo.gif docs/demo.gif
   ```

3. Let me know it's there and I'll update the README to show it instead
   of the static screenshot.

## Faster alternative, if you're comfortable with Terminal

If you have [ffmpeg](https://ffmpeg.org) installed, you can skip Gifski
and convert the `.mov` directly:

```bash
ffmpeg -i ~/Desktop/your_recording.mov -vf "fps=12,scale=800:-1" docs/demo.gif
```
