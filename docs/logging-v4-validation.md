# Weave hook with observation-neutral logging

Sixteen 120-second bouts rechecked the unchanged `weave_hook` against the same close-memory and full-duty programs, seeds 503/607/709/811 with both starting sides. Default runtime `sync-subset-v4` removes diagnostic print ticks. Source text/hashes, engine, robot, arena, rules, friction, sensor/IMU models and timestep match the corresponding archived v3 bouts. Two batch workers were used; no extra concurrency or physics changes.

| Opponent | Previous v3 W–D–L | Current v4 W–D–L | Current score |
|---|---:|---:|---:|
| Close-memory | 6–0–2 | 6–0–2 | 75% |
| Full-duty | 3–3–2 | 2–5–1 | 56.25% |
| Pooled | 9–3–4 | 8–5–3 | 65.625% |

All 16 bouts finished without program or opening-rule faults. The pooled score is unchanged, but there are fewer wins and more timeout draws. Six full-duty outcomes changed; every memory win/loss stayed the same. Removing diagnostic timing has not established a stronger policy.

Memory wins took a median 64.000 seconds, compared with 37.920 seconds under v3. The original-side wins grew from 43.405 to 92.258 seconds; swapped-side wins grew from 32.435 to 35.741 seconds. The first observer opponent-echo event occurs at 4.000 seconds; initial physical contact occurs around 4.52 seconds. The longer duration is an altered controller trajectory in virtual time, not disk writing slowing down simulation time.

## Recovery observations

Against memory, 80 suspected-lockup alerts included 76 close-duel alerts, but none produced a new sustained contact-free interval or sustained separation by the existing offline analyzer. This confirms that useful resolution can coexist with prolonged pushing and unsuccessful escape attempts. It does not prove a hardware stall or a causal benefit from the hook.

Against duty, 134 alerts included 106 close-duel alerts; four new sustained contact-free intervals and three sustained separations qualified under the existing thresholds. Five bouts nevertheless timed out. These are observer measurements, not controller inputs or proof that the logged recovery caused the eventual outcome.

The tests reuse old seeds rather than a fresh statistical holdout. Eight memory bouts contain only four distinct exact physical trajectories; eight duty bouts contain seven. Repeated trajectories reduce evidence diversity. Mechanics, sensor errors and hardware call timing remain uncalibrated.

## UI and evidence

At this revalidation stage, the selected pair was `weave_hook` versus `pressure_memory`, with a v4 seed-607 swapped winning replay at 35.741 seconds. The subsequent [numeric tuning experiment](weave-tuning.md) selects a self-play specialist against the original; it has a weaker pooled record. This exact prior selection, including its `historicalValidation` and v3 training profile, is preserved in `experiments/weave-tuning-previous-selection.json` and the current selection's `previousSelection`. This revalidation did not rerank every earlier candidate.

[Tracked comparison](../experiments/logging-v4-validation.json) includes all paired before/after outcomes, source hashes, recovery totals, contact durations and trajectory hashes. Exact raw programs/events/frames remain under `artifacts/strategy-tournament/logging-v4-memory/` and `logging-v4-duty/`. Local recovery and resolution image strips accompany the selected replays. The master audit retains its original 68 reports unchanged, appends these two reports and records 452 bouts: 428 distinct configurations and 24 separately labeled compatibility repeats. Same-seed v4 reruns are labeled runtime revalidation, not untouched holdout evidence.
