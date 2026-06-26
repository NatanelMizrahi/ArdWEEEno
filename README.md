# ArdWEEEno 🎶
Code for Arduino controlled audio emitting swings 

## [Demo 🎥👈](https://photos.google.com/share/AF1QipOoZUkBwacRxa_by7Bs9Dapoc73FGxg-hd67foIXAgzonKhvW4bxuMG6y5_qXriDA/photo/AF1QipPLCPlyGbMgPkFP9kpvnbAjoSnfSbd16_C__WXU?key=bHRFa0RjRWZpSVNIVkJUSng4UmVXeDlvOUhQVTVn)

**This is a prototype**. The final version is not videoed for privacy reasons

<a href="https://photos.google.com/share/AF1QipOoZUkBwacRxa_by7Bs9Dapoc73FGxg-hd67foIXAgzonKhvW4bxuMG6y5_qXriDA/photo/AF1QipPLCPlyGbMgPkFP9kpvnbAjoSnfSbd16_C__WXU?key=bHRFa0RjRWZpSVNIVkJUSng4UmVXeDlvOUhQVTV">
  <img width="266" height="380" alt="Screenshot 2025-10-20 at 19 28 02" src="https://github.com/user-attachments/assets/fa723440-03d9-4e83-b2d5-918870523b6c" />
</a>


[Album link 👈](https://photos.app.goo.gl/XLQYSxPNwAZhtYaN8)

## How it works

The swing tracks pitch angle via a Kalman-filtered IMU. A progress bar fills as you swing — the harder and higher you swing, the faster it fills. As the bar rises through levels, escalating audio clips play from a voice assigned to that session.

**Voices & levels:** Each voice has 8–11 levels. The higher the level, the more intense the audio. Reaching the top level ends the session and triggers a reset.

**Voice rotation:** On reset (or after 10 seconds of idle with at least one sound played), a new voice is randomly selected. All voices are cycled through before any repeats.

**Multi-unit setup:** Each physical unit is flashed with `SWING_INDEX` selecting its voice pool:
- `0` — test unit (voice 0 only)
- `1` / `2` — backup units (9 voices each, 50/50 split)
- `3` — main unit (all 18 production voices)

**Easter egg:** Hold the swing at 80°+ for 10 seconds to lock in a special voice for one session. It then returns to normal random rotation.
