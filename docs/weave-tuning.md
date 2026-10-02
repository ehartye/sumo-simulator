# Numeric self-play tuning

Two copies of the stock winning build ran unchanged physics and devices. One program stayed frozen as `weave_hook`; the other changed only numeric values. The original source hash remains `5f8eef50653d05caf4931043c5e7500e9bc5e49be7d0717a9b42ed02f56182d6`. AST checks preserve the entire nonnumeric structure, strings and imports. Runtime v4, sensor models, 1 ms physics, friction 0.8, 120-second bouts, three-second wait and opening stayed fixed. Batches used two workers.

## Screening

Each row ran seeds 1901/2017 with starting sides swapped, four bouts. The identical-program control scored 1W/2D/1L: 25% raw wins and 50% score. Its swapped runs exactly repeat the same physical configurations; those two repeats are labeled separately.

| Variant | Changed values | W–D–L | Score |
|---|---|---:|---:|
| 35 `weave_fast_loop` | Loop wait 10 → 5 ms | 2–2–0 | 75% |
| 36 `weave_slow_loop` | Loop wait 10 → 20 ms | 2–2–0 | 75% |
| 37 `weave_soft_charge` | Charge target 850 → 700 deg/s | 2–2–0 | 75% |
| 38 `weave_fast_charge` | Charge target 850 → 1000 deg/s | 1–3–0 | 62.5% |
| 39 `weave_short_hook` | Hook 35 → 20°, deadline 450 → 250 ms | 2–0–2 | 50% |
| 40 `weave_wide_hook` | Hook 35 → 55°, deadline 450 → 650 ms | 2–0–2 | 50% |
| 41 `weave_early_hook` | Engagement 4000 → 2400 ms, history 4200 → 2600 ms, cooldown 2500 → 1500 ms | 1–2–1 | 50% |
| 42 `weave_patient_hook` | Engagement 4000 → 6000 ms, history 4200 → 6200 ms, cooldown 2500 → 3500 ms | 0–2–2 | 25% |

Raw win percentage counts wins divided by every bout. Score counts each draw as half; a 75% score is not a 75% win rate. Speed values are targets for the existing voltage/torque-limited motor model, not extra motor power or imposed chassis speeds. Loop wait is additional to existing hardware-call scheduler slots; it does not change the sensor's 100 Hz sampling rate.

## Frozen validation

The two loop-timing finalists were selected by screening score, raw wins and then median winning duration, and frozen before validation. Seeds 2203/2309/2411/2531 were new to prior project trials and disjoint from screening; each ran both starting sides.

| Finalist versus frozen original | W–D–L | Raw wins | Score | Median winning duration |
|---|---:|---:|---:|---:|
| 36: 20 ms wait | 4–4–0 | 50% | 75% | 14.192 s |
| 35: 5 ms wait | 3–5–0 | 37.5% | 68.75% | 81.068 s |

Variant 36 passes the existing >=75% score/no-fault bar and is the selected self-play matchup in the UI, against the frozen original. Its only source change is the final `wait(20)` instead of `wait(10)`. The original remains selectable and its prior selection/results are preserved. All four wins are from the original starting side; every swapped-side bout times out. The eight variant-36 validation bouts contain only two exact physical trajectories, so they are not eight diverse independent physical tests. The result is a measured advantage in this particular matchup, not a general or hardware champion claim.

## Opponent-pool regression

Variant 36 then faced the same memory/full-duty pool as the original, seeds 503/607/709/811, both sides. These pool seeds were already used; they are a matched regression check, not fresh validation.

| Opponent | Original v4 W–D–L | 20 ms variant W–D–L |
|---|---:|---:|
| Memory | 6–0–2 | 3–3–2 |
| Full duty | 2–5–1 | 1–1–6 |
| Combined | 8–5–3 | 4–4–8 |

Combined score falls from 65.625% to **37.5%**. The 20 ms program is a specialist against the original and performs worse against these other opponents. Improving self-play does not establish a stronger general policy. A later search should include these opponents in qualification, preserve new validation seeds and examine the starting side that still draws. The hook-angle/deadline bundles do not isolate which individual value causes their outcomes, and altered scheduling alone does not establish a physical mechanism.

## Evidence and UI

All **68 new bouts** are free of program and opening-rule faults: four controls, 32 screening, 16 finalist validation and 16 pool checks. Every bout preserves exact programs/hashes, settings, timestamped decisions/contact and replay frames. [The compact report](../experiments/weave-tuning.json) includes side counts, raw percentages, recovery observations and exact physical-trajectory hashes; [the plan](../experiments/weave-tuning-plan.json) pins edits and the finalist freeze. The master audit retains its original 70 reports and appends 13, totaling **520 bouts: 494 distinct configurations, 24 compatibility repeats and two self-play control repeats**.

Eight experimental presets are available. **Load latest best** selects `weave_slow_loop` versus `weave_hook`; **Inspect winning bout** loads the seed-2203 original-side win at 14.192 seconds. Active matches, edited programs and imported replays keep the existing update guards. A local four-frame strip shows contact through resolution. The simulator remains uncalibrated, and no mechanical, sensor or Pybricks API capability changed.
