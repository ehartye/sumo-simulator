# Lockup lab: telemetry and physics-only escape search

Thirty-odd controller variants plateaued on lockup. This round changes the *instrument* before changing the controller: observer telemetry that says what a lockup physically is, and a physics-only search that can test hundreds of breakouts in minutes. All numbers are uncalibrated simulator observations; no physics, sensor model or runtime was changed.

## Tools

| Tool | Purpose |
|---|---|
| `tools/lockup_lab.py` | Full-match observer telemetry (50 Hz): chassis distance, wheel rim speed, motor current, tilt, per-support normal force (tire A/B, caster, other) and bot-to-bot force. Lock episodes are distance-based (chassis centres < 190 mm for >= 2 s with the pair centre moving < 0.03 m/s). Classifies every floor-exit as **forced** (opponent engaged within 1.5 s) or **unforced**. `--frictions` sweeps tire friction; `--jitter-mm/--jitter-deg` spreads start placement because sensor-noise seeds alone give exact physical repeats. |
| `tools/maneuver_lab.py` | Branches from a snapshotted head-on lock (48 contact geometries, friction sweep) and replays open-loop wheel maneuvers or scripted policies in physics only: no Python RPC, no sensors, thousands of candidates per hour. Opponents: blind `duty` / `run850`, and `trk[...]`, an observer-truth tracker with perfect bearing and a rim guard (deliberately stronger than the real ±35° sonar). |

`contact_started` events undercount lockup: pressed contacts sit at `dist` slightly above 0, so only ~10% of a 100-second lock is flagged touching. Use chassis distance.

## What a lockup is

In `weave_hook` vs `duty_hook` (seed 11, swapped sides) the robots sat 154 mm apart, eyes-to-eyes, for about 100 s. Pair-centre drift was 0.003–0.013 m/s, tilt under 1 degree, and both drove near full effort. The contact between them is the ultrasonic eyes parts, re-striking about every 12 ms.

Tire normal force is not steady. Rest support equals the 3.51 N weight exactly, but a *lone* robot at full duty has both drive tires unloaded 15–25% of the time, with support force swinging ±2 N. At 30% duty that falls to 0.5%, at 50% duty 1.4–5%. The result does not change with timestep (0.001 / 0.0005 / 0.00025 s all give 22–25%), so it is not a solver step artifact. Whether real LEGO rubber hops like this is unknown, and the tire model is the least certain part of the simulator (see "Drag" below).

## Results

All escape searches below assume the same stock build and a persistent full-duty pusher from a settled lock.

1. **Lower throttle does not escape.** Head-on duels at 40/55/70/85 vs 100 duty are won by 100, monotonically; equal duty at any level stays a stalemate for the whole 4 s.
2. **Open-loop dodge/back-off search** (1500 three-segment candidates, 8 geometries x 4 frictions x 3 opponents): the best candidate won 30 of 96 validation bouts, almost all against the blind pusher, and only 3–5 wins per candidate were forced. Against the tracker nearly every candidate lost 6–8 of 8. The blind pusher drives itself off the rim after a side-step because it has no rim guard; a real incumbent does. This is the bullfighter effect, not an escape.
3. **Pre-contact flank/dodge search** (1200 candidates from the opening gap vs the tracker): best was 2W/1L of 4, i.e. noise; the null lost 1.
4. **Perfect-bearing re-aim and pivot** as challenger (nine variants, 72 cells each): identical to the null (0 wins at friction 0.8 and 1.0). Forward duty 70 lost 45 of 72.
5. **PWM duty modulation** (144 variants, 4–80 ms periods, wheel phase offsets, lo = -100..40): 0 wins, no better than steady duty.
6. **Contact geometry matters, chaotically.** From identical-duty locks, the outcome changed with 12–25 mm offset / 3–6 degree yaw at contact, and the effect is not a clean left/right rule. A 48-geometry x 3-friction check gave blind `duty` as challenger vs tracker 0W/28L of 144, but tracker as challenger vs `duty` only 5W/3L (0 forced). That role asymmetry is unexplained and is a first thing to check before trusting any single-role number.

Conclusion for this simulator: **a symmetric head-on lock between equal robots at equal effort is a true stalemate; no breakout tested forces a win against an opponent that adapts.** Wins from lock exist only against opponents that are blind or lack a rim guard. Further numeric tuning of recovery hooks is unlikely to help, which matches the previous plateau.

## Drag and friction

Tire contact is one Coulomb coefficient (0.8) for LEGO rubber on painted wood, with no static/kinetic split, no load dependence and negligible rolling resistance (`docs/calibration.md`). It has not been measured. Lower friction raised the hop fraction (0.5: 32%, 0.65: 26%, 0.8: 25%, 1.0: 19% in the lone-drive test) and changed which cells were won, so conclusions must be checked across the range. Every search above ran at 0.5/0.65/0.8/1.0 or states which single value it used.

Needed before any ranking is trusted: a measured pull-slip test of the real wheel on the real ring (static/kinetic coefficient, load dependence), a real full-duty traction test (does the robot hop?), and a head-on stall test between two real robots.

## Recommended next steps

1. Measure friction and the hop on hardware (or accept that the symmetric-lock result is a model property, not a hardware fact).
2. Resolve the 28-vs-5 role asymmetry; if the world has a left/right or bot-index asymmetry, every paired result needs side-balancing at the physics level.
3. Treat draws as the realistic ceiling against matched controllers; spend effort on **not entering** the lock (arrival geometry, rim-aware flanks) with a closed-loop controller judged against `weave_hook`/memory/duty over a friction sweep and jittered starts, using `lockup_lab` forced/unforced classification.
4. Add the observer measures here to the strategy tournament report so a "win" is labeled forced or unforced.

Reproduce: `python -m tools.lockup_lab A.py B.py --frictions .5,.8,1.0 --jitter-mm 5 --jitter-deg 2`; `python -m tools.maneuver_lab --challengers duty,tracker --opponents duty,tracker --frictions .65,.8,1.0 --fixtures 0,1,2,3`. Raw outputs live in ignored `artifacts/lockup-lab` and `artifacts/maneuver-lab`.
