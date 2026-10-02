# SPIKE Prime sumo lab

Two LEGO part assemblies, a fixed-step MuJoCo match engine, and two independent Python robot programs. Run, pause, step, inspect telemetry, scrub a bout, save its replay, or compare strategies in paired headless matches.

**Uncalibrated simulator, not yet a validated digital twin.** Geometry and catalog masses are sourced. Motor control, tire contact, battery sag, sensor errors and mass distribution remain approximations. Rest-position connector alignment is checked; physical assembly and cable routing need a bench check. See [calibration and evidence](docs/calibration.md).

## Run locally

Requires Python 3.12+, Node 24+, and sibling `../agent-meshes` with the new `assemble` command. That checkout was updated from remote main `f5fa7ee` (0.6.0), then extended on `feat/spike-robot-assets`.

```powershell
python -m venv .venv
.venv/Scripts/python.exe -m pip install -e '.[test]'
npm ci
npm --prefix ../agent-meshes ci
npm run build:robot
.venv/Scripts/python.exe -m sumo.cli serve
```

Open **http://127.0.0.1:8765** for matches or **http://127.0.0.1:8765/ui/robot.html** for the robot review. On Linux/macOS use `.venv/bin/python`. The service binds to loopback. Programs must be trusted local Python: worker processes and import restrictions are **not a security sandbox**.

Edits apply on run/reset. Pause stops after an in-flight step finishes. Scrubbing inspects recorded states; resuming continues from the latest live state. Saved JSON contains programs, hashes, robot hash, seed, rules, engine/settings, telemetry and body poses. Browser tabs share one local match session.

## The minimal build

Both bots use the same stock hardware: hub and battery, two medium angular motors, two 56 mm wheels, ultrasonic eyes, downward color sensor, caster, and supporting beams, frames and pins. No extra active devices or ballast.

| Component | Quantity | Catalog mass each |
|---|---:|---:|
| Hub body / rechargeable battery | 1 each | 63.12 / 86.70 g |
| Medium angular motor, cable included | 2 | 49.53 g |
| Ultrasonic eyes / color sensor | 1 each | 34.38 / 12.85 g |
| 56 mm wheel | 2 | 14.30 g |
| Caster holder / ball | 1 each | 4.11 / 4.99 g |
| 7-hole beam / 3L axle | 2 each | 1.74 / 0.43 g |
| 5 × 7 frame / 3 × 3 connector block | 2 each | 5.30 / 2.00 g |
| 2L slotted pin / 3L pin | 22 / 4 | approximately 0.15 / 0.37 g |

**43 rendered parts; approximately 357.53 g; 103.6 × 80 × 158.8 mm** (width × height × length). Hub geometry includes its battery, whose mass is counted once. Flexible cables are omitted visually; their mass is included in catalog electronics values. These are catalog sums and nominal bounds, not measured tolerances.

`hardware/spike-prime-inventory.json` preserves the full 45678-1 replacement-parts manifest. `hardware/build-inventory.json` selects specific stock rows without adding alternate molds together. `tools/build_reference.py` regenerates `hardware/minimal-spike.json` from checked LDraw hole coordinates and checks stock quantities. Then `npm run build:robot` produces `model.glb`, `robot.json`, verification and attribution under `artifacts/robot/`.

The model has 61 pin, axle and socket mates forming one connected graph. Checks cover rest-position anchors, axes and compatibility. They do not certify assembly sequence, stiffness or moving connector fidelity: motor output-disc visuals remain chassis-bound while wheel pins and axles rotate. Physical fit remains unverified.

Mass URLs are in the manifest. LDraw geometry, original licenses and file hashes are pinned under `assets/`; assembly needs no network fetch. The loader rejects exported hardware that differs from the reference source. Editing the source changes that reference: this is a local research tool, not a tamper-proof competition judge.

## Python compatibility

The target is **synchronous Pybricks v4 style**, implemented with CPython device shims. It does not run upstream firmware or claim complete MicroPython equivalence.

| Supported API | Behavior |
|---|---|
| `Motor(Port.A/Port.B, positive_direction=...)` | `run`, `dc`, `stop`, `brake`, `hold`, `angle`, `speed`, `reset_angle` |
| `run_time`, `run_angle`, `run_target` | `then=Stop.COAST/BRAKE/HOLD`, `wait=True/False`; approximate controller |
| `ColorSensor(Port.C).reflection()` | Downward ring reflectance 0–100 |
| `UltrasonicSensor(Port.D).distance()` | Dense 3D cone, millimeters; 2000 for no valid return |
| `PrimeHub()` | Relative/unwrapped clockwise-positive `imu.heading()`, `imu.tilt()` pitch/roll in degrees ([assumptions](docs/imu.md)), `battery.voltage()` in mV |
| `wait(ms)`, `StopWatch.time/reset` | Shared simulation clock, independent bot waits |
| `Port`, `Direction`, `Stop`, `math`, limited `random` | Explicit enums and per-worker seeded PRNG |

Use CLOCKWISE on A and COUNTERCLOCKWISE on B for forward motion. Encoders are integer degrees at 100 Hz; speed is sampled finite difference. Hardware calls consume a 1 ms scheduling slot, a declared approximation. Sensors sample on the simulation clock at 100 Hz, including when no program reads them. Noise depends on seed/bot/sample time, so viewer polling does not affect outcomes.

Runtime `sync-subset-v4` captures diagnostic prints without adding simulated ticks and moves live JSONL writes to a bounded background thread. Hardware scheduling and physics rates are unchanged. Logging status exposes backlog, overflow and disk errors; full observations remain in replay memory. Real hub print costs remain uncalibrated. [Logging timing and compatibility](docs/observation-logging.md) describes the limits and selectable legacy v3 profile. The frozen original's [16-bout v4 revalidation](docs/logging-v4-validation.md) retains a 65.625% pooled score; historical v3 records remain preserved.

Async tasks, `multitask`, `DriveBase`, gears, advanced control tuning and unlisted modes/APIs fail explicitly. The device interface exposes no opponent pose. Python introspection can bypass the convenience restrictions; only run trusted scripts.

## Sensor diagnostics

The **Sensors** viewer button shows the selected echo ray and hit point. Each bot reports echo/no-echo/below-rated-range status; hover that status for sample timing and rejected-ray counts. `/api/diagnostics` exposes the current detailed log. Saved runs include `ultrasonicSamples`. Every UI match automatically writes a unique `.cache/runs/*-ultrasonic.jsonl` plus metadata JSON containing programs, seed, model assumptions and robot hash; reset preserves prior logs. CLI batches write sensor logs beside their output under `sensor-logs/` and reference them from the report.

The sensor uses LEGO's nominal range, resolution, sampling rate and entrance-angle specification. Circular cone shape, reflection rejection, near-field policy and noise are explicitly uncalibrated. See [sensor evidence and diagnostic fields](docs/ultrasonic.md).

The **Match record** timeline logs opponent echo acquisition/loss, program `print()` messages, opening phases, robot contact/separation and the result. Click a timestamp to inspect the nearest 50 ms replay frame; filter detections, contact or program decisions. Event timestamps use simulation time: sensor transitions at 100 Hz, physical contact at each 1 ms step. Contact means inter-robot collision geometry touching/penetrating (`dist <= 0`), excluding floor and ring contacts; tiny separations can create multiple contact episodes. Opponent identity is an observer annotation, never data exposed to robot programs. “Charge” messages show a controller decision, not proof of physical contact.

`/api/events`, saved replay `events`, and automatic `.cache/runs/*-events.jsonl` preserve every event, including those between browser frames. The UI shows the latest 300 matching events up to the selected replay frame. CLI batch reports include the same event stream. The examples log decision changes and acquire any valid echo (`distance() < 2000`), retaining border-first priority. Their former 550/400 mm thresholds left them scanning despite valid echoes farther across the ring.

**Load replay** opens a saved run for local inspection and export. Run/Step/Reset then begin a fresh live match. **Load latest best** selects the promoted strategy and its compared opponent; **Inspect winning bout** loads the recorded promotion win. A new promotion loads automatically while paused with untouched editors, preserving active matches, edits and loaded replays. The selection comes from `experiments/champion.json`; "best" refers to the declared head-to-head promotion test, not universal dominance.

The [ten-loop strategy report](docs/strategy-tournament.md) preserves every hypothesis, result and limitation. Reproduce a paired trial with `python -m tools.strategy_tournament <challenger.py> <incumbent.py> --out artifacts/strategy-tournament/new-trial`. Each trial saves exact program text/hashes, engine/robot/rule metadata, every match event and 50 ms replay frames with sensor diagnostics; these tournament replays do not contain the live viewer's full 100 Hz ultrasonic sample stream. Generated evidence remains local under `artifacts/strategy-tournament/`, served by `/trials/`. Capture the resolution sequence with `node tools/review_replay.mjs <bout.json>`.

Three [lockup recovery presets](docs/stall-recovery.md) are available: `stall_hook` (brief release/gyro hook), `stall_geometry` (uncertain ring/echo history for direction), and `stall_pressure` (fresh close return and forward hook). Logs record suspicion, direction basis, turn error/timeouts and travel since white. These infer suspected lockup; wheel slip and gyro drift prevent exact position or opponent exclusion. The [results](experiments/results.json) preserve the original bouts and the [ten additional variants](docs/recovery-round-two.md), fresh-seed comparisons and separately marked compatibility reruns. Capture recovery frames with `node tools/review_replay.mjs --recovery <bout.json>`.

The additional search uses shallow forward arcs with one driven wheel at a time, rather than stationary spinning. Sustained blind-drive and close-engagement watchdogs log suspicion; bounded gyro hooks and stock-hub pitch/roll guards test responses. [Offline recovery analysis](tools/recovery_analysis.py) measures actual posture, travel and separation from saved observer frames, never controller inputs. The selected `weave_hook` is a training-best complete program, with weaker fresh-seed results and timing-sensitive outcomes. It is not a proven universal or hardware champion.

The [mat-crossing and snap-reversal experiments](docs/snap-reversal.md) add `crossing_hook`, `snap_reverse`, `snap_coast` and `snap_steering`. Their 48 training bouts and eight fresh direct bouts distinguish executed reversal/contact release from improved results. `weave_hook` remains selected.

The [damping and targeted counter experiments](docs/damping-counters.md) add seven presets and 66 paired bouts. Passive braking beats active damping in training but loses the fresh direct challenge 3–5. Mirrored opening and patient full-duty pressure each recorded six training draws; patient pressure's logged gaps gain only about 2 mm, not a useful flank. No candidate replaces `weave_hook`. Exact sources, hashes, scores and limitations are retained in the [counter results](experiments/damping-counters.json).

The [numeric self-play sweep](docs/weave-tuning.md) adds eight copies of the same build/program with altered timing, speeds and hook angles/deadlines. A 20 ms loop wait beats the frozen original 4W/4D/0L on new seeds (50% raw wins, 75% score), and is selected for that UI matchup. All wins come from one starting side. Its memory/full-duty pooled score regresses to 37.5%, so the original remains the stronger tested general competitor. The 68-bout evidence preserves exact values, sources, failures and repeated controls.

Runtime v3 corrects the hardware heading API to Pybricks' clockwise-positive convention. Live feedback presets explicitly convert to their existing internal CCW yaw. Archived v1/v2 programs and replay fields retain their historical convention: use them for inspection with the recorded profile, or adapt signed heading reads before a new v3 run. [IMU conventions and assumptions](docs/imu.md) distinguish the hardware API from the legacy absolute observer yaw field. No mechanics or random streams changed.

## Match rules and physics

The UI and CLI use a **provisional Atomic Eagles 2022 profile with the user's 180° pivot clarification**. The [rules page](https://www.atomiceagles.com/rules) links the [August 18, 2022 rules](https://docs.google.com/document/d/1gM1sZfyIzWYAL7DgNhXlfSiJHiPrDX5llAFiOCDmdSY/edit): 4-foot ring (1219.2 mm), 2-inch white border (50.8 mm), 3-second wait and 2-minute bout. Robots face away with chassis origins 152.4 mm either side of center. First motion is forward: touch the white border on the starting half then turn, or pivot about one stationary wheel approximately 180°. Completion allows 5° angular tolerance and at most 5° stationary-shaft drift. Examples demonstrate both paths.

The source ambiguously describes a minimum-radius arc; the user explicitly specified the stationary-wheel about-face used here. Ring height is assumed 25.4 mm because the source conflicts between 1 inch and 1.5 inches. Source spacing also says “gap”; current placement uses center separation. Tournament identity remains unconfirmed. The 10-second start deadline, 15-second deadlock reset, disabled-robot judgments and tournament scoring are not implemented. Full provenance and limitations are returned by `/api/info` and saved in replays.

First physical outside-floor contact loses; simultaneous contact draws. Program faults and opening violations lose; same-tick faults draw. A completed program coasts while the bout continues. CPU-only code and diagnostic prints must reach a hardware yield within one wall second; the per-program cap is 100,000 operations, including prints. `World()` alone retains the original 770 mm test arena and has no referee unless a rule profile is supplied.

Physics uses 1 ms steps, gravity, 3D chassis motion, wheel hinges, a passive caster ball and inter-bot contact. Torque comes from voltage/back-EMF, never imposed chassis velocity. Primitive collision proxies carry uniform-density inertia estimates. Same-robot parts do not self-collide; deformation, backlash, cable forces and chassis flex are absent.

```powershell
.venv/Scripts/python.exe -m sumo.cli batch examples/aggressive.py examples/patient.py --pairs 5 --seed 7 --out artifacts/comparison.json
.venv/Scripts/python.exe -m pytest -q
node tests/browser.mjs
```

Batch runs each seed twice with program positions swapped. Repeatability is scoped to the pinned engine/runtime environment, not promised across platforms. `requirements-lock.txt` and `package-lock.json` record dependencies.

## Architecture and verification

- `sumo/robot.py`: fixed-build validation and MuJoCo conversion.
- `sumo/physics.py`: motor, contact, sensor and floor-contact judging models.
- `sumo/rules.py`: sourced arena profile, start protocol, provenance and enforcement limits.
- `sumo/runtime.py`: spawned Python workers synchronized through RPC barriers.
- `sumo/server.py`, `web/`: local API, GLB viewer, editors and replay.
- `tests/`: stock quantities, masses, mates, source hashes, motion, braking, contact, timestep convergence, repeatability, APIs, clocks, faults, opening rules and replay.

The 0.4-second straight acceleration test differs by less than 2 mm across 1, 0.5 and 0.25 ms timesteps. This checks numerical behavior, not physical accuracy. Next steps are physical build validation, rotor articulation and measured calibration with held-out trials. Wiki architecture and roadmap: `wiki/authored/sumo-simulator/overview.md`.
