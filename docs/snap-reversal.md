# Mat-crossing suspicion and sudden wheel reversal (24–27)

The user proposed noticing when sustained driving should have carried a robot off the mat, and keeping one wheel forward while abruptly reversing the other. Four standalone runtime-v3 controllers test this with unchanged stock build, physics, sensor models, referee and batch concurrency. Archived sources and hashes preserve failures.

The new crossing watchdog accumulates both-wheel forward-command time, independently of fresh ultrasonic echoes. At the current nominal 850 deg/s command and 56 mm tires, 3.5 seconds implies approximately 1454 mm, exceeding the 1219.2 mm ring with margin. It alerts only with at least 3.5 seconds credited in the recent 4.2-second history and gyro span at most 50 degrees. It clears alongside history after white/recovery. This is a nominal-effort contradiction, not a measured chassis position or proof that the robot should have fallen: slip, motor load, gyro error and yaw variation invalidate exact distance. The existing four-second close-engagement gate still handles circular duels.

The [official Motor reference](https://docs.pybricks.com/en/stable/pupdevices/motor.html#pybricks.pupdevices.Motor.dc) supports signed duty from -100 to 100 and distinguishes speed control, coasting, braking and holding. The pulse commands one wheel +100%, the other -100%, without first braking both, until 35 degrees of achieved gyro turn or a 350 ms deadline. The motor model retains bounded torque and inertia; a command is not an instantaneous velocity change. White/tilt protection retains priority. Front-only color sensing cannot guarantee rear clearance, and the transients remain uncalibrated.

## Shared-opponent training

Each variant runs seeds 11/23/37, both sides, 120 seconds against fixed full-duty (03) and close-memory (01): 12 each, 48 total. Score counts draws as half a win.

| Variant | Change | vs duty W–D–L | vs memory W–D–L | Pooled score |
|---|---|---|---|---|
| 24 `crossing_hook` | Added detector, existing forward hook | 3–2–1 | 6–0–0 | 83.3% |
| 25 `snap_reverse` | Full opposing-duty pulse | 3–1–2 | 2–0–4 | 45.8% |
| 26 `snap_coast` | Pulse, 150 ms coast, then recharge | 2–0–4 | 3–1–2 | 45.8% |
| 27 `snap_steering` | Pulse follows recent gyro rotation | 3–1–2 | 2–0–4 | 45.8% |

There are **zero crossing-watchdog invocations** in all 48 bouts. The bots' large heading spans explain why nominal straight crossing is inapplicable to these moving circular encounters. The control retains loop 21's outcomes; its record does not prove the new gate helps. A controlled complete-worker fixture supplies a fixed close echo and heading on a wide mat: it proves crossing activation with a valid echo, white suppression before activation and white interruption during the pulse. That fixture is not a physical bout or part of scoring.

Actual recorded motor speeds acquire opposite signs during every observed pulse window: 25 has 102/102, 26 has 97/97, and 27 has 92/92. These independent-review counts verify the requested maneuver executes in the current model. They do not establish real hardware reversal timing.

Against close-memory, new contact-free intervals lasting at least 0.5 seconds increase from zero in control 24 to 13 (25), 10 (26) and 2 (27). None simultaneously sustains chassis separation above 180 mm. Variants 25 and 27 each record one strict new separation against duty, in a bout that ultimately draws. Pressure release can break touch without creating a tactically useful gap.

In 25's seed-11 original-side memory bout, the first alert is at **9.705 s**. A changes from +875 deg/s at the preceding 9.700 s frame to -712 deg/s at 9.750 s while B remains forward. At recovery completion, 10.283 s, range is 2000 and yaw has changed -50.4 degrees; the bots visibly separate briefly. The candidate nevertheless loses at **21.046 s**. The saved recovery strip and full replay make that sequence inspectable.

Opponent inversions occur in some pulse-program bouts, but they do not by themselves establish the intended destabilization mechanism. Inspected first inversions occur at least 1.789 seconds after the most recent alert, often after further search/edge maneuvers, and an inverted opponent can still win or draw. No causal tipping benefit is established. A coast can continue rotation substantially: the 26 seed-37 swapped-side pulse ends at 21.394 s with +38.2 degrees, but recovery completes at 21.866 s with +204.5 degrees. This overshoot is an observed downside of unloading, not a reason to alter physics to favor a controller.

## Frozen direct challenge

The three snap approaches tie in aggregate training score. Freeze the simplest requested pulse (25), which also records one strict separation, and challenge the current 21b `weave_hook` on source-fresh seeds 881/991/1103/1217, both sides. **25 scores 3W/0D/5L (37.5%)**, with no program/rule faults. No tuning follows validation. Retain `weave_hook` as the selected incumbent; new variants are available as experimental presets rather than promoted champions.

`experiments/snap-reversal.json` records exact training/direct scores, hashes, posture, crossing counts and recovery counters. Full local replays live under `artifacts/strategy-tournament/loop-24-duty` through `loop-27-memory` and `snap-validation-25-vs-21`. The existing viewer can load them and seek program/contact events. The combined project audit verifies 370 recorded bouts in 58 reports: 346 distinct trials and 24 separately labeled compatibility repeats, all without program/rule faults.

## Interpretation and next step

The new commands make meaningful motion, but the selected forward hook remains stronger in these comparisons. Constant motor effort can suggest a blocked straight traverse; it does not diagnose circular pushing. Breaking contact and winning must be evaluated separately. Next candidates should limit achieved turn and residual yaw, then reacquire a fresh echo, rather than allow a coast to rotate through an about-face. Calibrating real motor reversal, traction, collision sensing and controller timing remains necessary before transferring these rankings to hardware.

## Verification

Thirty-three focused Python tests pass, including the complete-worker crossing/white-preemption fixture; all candidate sources compile. Independent review recomputes all 48 training metrics and verifies all eight direct sources/hashes/side mappings and outcomes. The actual UI exposes all four presets, loads the exact archived failed bout, seeks the 9.705-second alert and retains the `weave_hook` selection after inspection without page errors. The wiki motor source summary and living project pages are updated and synced. No simulator, batch-concurrency or viewer implementation changed.
