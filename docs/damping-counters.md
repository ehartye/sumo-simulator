# Short-pulse damping and winner-targeted counters (28–34)

The user authorized continued recovery experiments and counterstrategies targeting the selected `weave_hook` (21b). All candidates keep the connected stock build, power bounds, referee, sensor models, runtime v3 and 1 ms timestep. Two-worker batches and at most two concurrent batches remain unchanged. Sources are frozen in `experiments/strategies/` before their first trial; standalone live presets use the same source.

## Damping comparison

Loops 28–30 share a shortened opposing-duty pulse: achieved internal gyro yaw 25 degrees or a 150 ms deadline. A following stage has a 250 ms deadline checked each control iteration and may finish after 60 ms continuously below an estimated 25 deg/s yaw rate; then a 300 ms forward recharge follows. Scheduled calls and loop sampling can cross nominal deadlines: completed damping stages commonly take 261–263 ms. White and tilt handling precede these stages. The 100 ms endpoint heading slope uses only existing `StopWatch.time()` and unwrapped `imu.heading()` calls. It can lag or conceal reversals/oscillation: low estimated average rate is not proven instantaneous rest.

28 applies symmetric corrective duty; 29 adds a common 40% forward duty; 30 uses passive brakes with the same measurement, settling criteria, logging and API-call count. Feedback may produce different elapsed stage durations. The active gain and deadlines are experimental assumptions, not measured hardware settings. Motor torque and inertia remain bounded; commanded reversal is not instantaneous speed reversal.

Training uses seeds 11/23/37, both physical sides, 120 seconds against exact archived 21b. Score credits a draw as half a win.

| Variant | Hypothesis | W–D–L | Score |
|---|---|---:|---:|
| 28 `snap_damped` | Oppose residual yaw | 2–0–4 | 33.3% |
| 29 `snap_drive_damped` | Dampen while retaining forward pressure | 3–1–2 | 58.3% |
| 30 `snap_brake_control` | Passive-brake control | 4–1–1 | 75.0% |

No candidate inverted in these 18 bouts. New contact-free intervals lasting at least 0.5 seconds count 2/8/7; strict new separation (also above 180 mm chassis distance) is zero in all three. Most alerts occur after prior separation, rather than during physical close duels. Breaks in touch and winning remain separate measures.

Independent observer recomputation over completed damping stages gives median physical end yaw rates of 24.80/13.83/1.35 deg/s and median physical yaw spans of 13.44/17.05/24.25 degrees. Passive braking had lower median residual rate and higher median added yaw in these bouts. Stage populations differ as later encounters change, so these pooled values are descriptive rather than a clean causal ablation. All variants share their first seed-11 pulse ending at 7.590 s with about -28.1 degrees and estimated -306.5 deg/s. By damping end, 28 reaches -49.3 degrees and estimated -41.7 deg/s, 29 -55.5 degrees and +6.5 deg/s, and 30 -60.1 degrees and -1.8 deg/s. Timing and controller evolution prevent attributing an entire bout's win to one local improvement.

Active damping plateaued below the passive control. The passive candidate qualifies for validation; it is not a demonstrated universal replacement from these six bouts.

## Targeting the incumbent

Three hardware-observable counter hypotheses follow:

- 31 `mirror_open`: reverse the legal opening's handedness by holding A and driving B forward. Target the incumbent's fixed opening/initial contact path; the same absolute 180-degree criterion and deadline apply.
- 32 `heading_press`: anchor heading when a charge begins and apply bounded positive duties with proportional heading and derivative-rate correction. Target sustained circular pushing; heading is a direction reference, not measured ground progress.
- 33 `echo_feint`: on a fresh 180–450 mm echo, make an alternating forward-wheel outward arc (25 degrees/200 ms), then an opposite forward-wheel return (4-degree residual/300 ms), with five-second cooldown. Target unconditional straight charging on any echo and short close-target memory. A single range does not supply enemy bearing, identity or position; the flank hypothesis remains uncertain.

No counter reads simulator pose, physical contact, opponent code, perfect speed or seed. Gyro feedback and front white sensing do not guarantee rear-edge clearance. Logged maneuver deadlines, posture, enemy recognition, contact and outcome are retained in replay evidence.

Training finishes at 0W/6D/0L (31), 0W/5D/1L (32), and 2W/1D/3L (33). The mirror produces no strict or new contact-free escapes and invokes 183 blind alerts after prior separation. Heading-pressure produces 139 close-duel alerts and five new contact-free intervals; the feint invokes 11 times and has nine new contact-free intervals. Neither has a strict new separation. None inverts. These attempts do not qualify as winning counters.

Following loop30's failed frozen direct validation, a bounded fourth counter, 34 `patient_pressure`, keeps both wheels at full duty during fresh echoes or 1500ms close-memory, delays its own close-engagement gate to eight seconds and retains 8.2 seconds of history. Its hypothesis is to retain pressure while a shorter-gated opponent unloads a wheel. It observes only its own sensors/history; it cannot know that the opponent actually began recovery. All three changes are tested as one whole program, rather than isolating their causal contributions.

34 finishes **0W/6D/0L**, with four threshold-qualified new sustained separations and 18 new contact-free intervals. None inverts; maximum own tilt is 2.82 degrees. A closer check matters: these four flags start with chassis centers already about 182.8–183.3 mm apart and end around 185.1–185.3 mm, only about 2.0–2.5 mm farther apart. They meet the existing 180 mm / 0.5-second observer criterion but do not demonstrate a useful flank or substantial physical escape. They never convert into wins. The observer gives a descriptive association, not proof that this controller caused the separation or correctly predicted the enemy's release. As a training-only draw counter it can foil victory in this small matchup set; it is not independently validated or superior by win score. Its 50% score does not qualify for conditional fresh testing.

## Hardware boundary

The [official hub API](https://docs.pybricks.com/en/stable/hubs/primehub.html#pybricks.hubs.PrimeHub.imu.angular_velocity) supplies direct calibrated angular rate along robot-frame axes, as well as clockwise-positive start-relative heading. Current runtime v3 exposes heading/tilt and does not implement that rate method; these candidates derive slopes rather than silently add a new sensor channel. Existing official and pinned firmware wiki captures already cover this capability; discovery deduplicated them and ingestion expanded the hub source summary. A future direct-rate implementation needs separate mounting, sign, noise, latency and API tests.

These rankings remain uncalibrated simulator results. Real reversal, gyro error under impacts, traction, motor/battery limits and the assumed 1 ms API/print scheduling need measured validation before hardware transfer.

## Validation and selection

Freeze 30, the sole 75%-qualifying candidate, and run seeds 1301/1423/1553/1667 on both sides against 21b, duty03 and memory01. Its source remains unchanged after validation.

| Frozen 30 opponent | W–D–L | Score |
|---|---:|---:|
| Selected 21b | 3–0–5 | 37.5% |
| Full duty03 | 4–0–4 | 50.0% |
| Close memory01 | 6–0–2 | 75.0% |
| Pooled | 13–0–11 | 54.2% |

Retain selected `weave_hook` and its existing representative replay. Seven new presets are available for inspection; none earns a champion promotion. The three shared pool scores do not represent a new head-to-head ranking for every controller. Counter variants were developed against training evidence; the last bounded counter was added after 30's validation failed, rather than retuning 30 or optimizing known seeds.

`experiments/damping-counters.json` preserves 42 new training and 24 frozen validation bouts with source hashes, scores, separation/posture metrics and local replay paths. `experiments/results.json` audits **436 recorded bouts in 68 reports**, comprising **412 distinct trials plus 24 separately labeled compatibility repeats**. All 66 new bouts have zero program or referee faults. Raw replays remain local ignored artifacts, not files available from a fresh Git clone.

## Verification

All **143 Python tests pass**, with one third-party Starlette/AnyIO deprecation warning. The actual local UI exposes all seven exact archived presets, imports the failed seed1301 frozen bout, seeks the 7.590-second pulse-end event, and restores selected21b source/runtime without page errors. Independent review audits the 18 damping bouts and 24 counter bouts, plus frozen comparisons; source hashes, side mappings, outcomes and observer counters agree. The physical-resolution strip for 30's seed11 training win shows the final white-edge encounter, not a proven pulse-caused flank. Project wiki pages and the hardware-rate summary preserve these results and limits.
