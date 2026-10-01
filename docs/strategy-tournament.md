# Ten strategy improvement loops

The hardware, physics, sensor model and referee remain unchanged from commit `93aab99`. Training uses seeds 11, 23 and 37 on both starting sides, 120-second bouts, stock 357.53 g robots and friction 0.8. A challenger is promoted at >=75% score (win=1, draw=0.5) with no program/rule faults. Seed variation changes sensor noise, not hardware tolerances. These are simulator comparisons, not physical performance claims.

Each numbered source is preserved in `experiments/strategies/`; full event logs and 50 ms replay frames live under `artifacts/strategy-tournament/loop-NN/`. `summary.json` records source hashes, winner mapping, errors and evidence paths. Existing Pybricks-only device interfaces are unchanged; no controller receives observer identity or contact truth.

| Loop | Challenger / hypothesis | Incumbent | W–D–L | Decision |
|---|---|---|---|---|
| 01 | Close-echo memory: continue pressure for 400 ms after a return below 200 mm | Original aggressive | 6–0–0 | Promote |
| 02 | Extend close-echo commitment to 1000 ms | Loop 01 | 0–6–0 | Reject |
| 03 | Full duty during charge, same 400 ms memory | Loop 01 | 3–3–0 | Promote at threshold |
| 04 | Brief differential hook after 1800 ms continuous attack | Loop 03 | 3–0–3 | Reject; maneuver never fired |
| 05 | Retain duel timer across short echo gaps | Loop 03 | 3–0–3 | Reject |
| 06 | Gyro heading hold during pressure | Loop 03 | 0–6–0 | Reject |
| 07 | Continuous one-direction search | Loop 03 | 3–0–3 | Reject |
| 08 | Sweep and align to shortest observed range | Loop 03 | 1–4–1 | Reject |
| 09 | White-line opening with full-duty pressure | Loop 03 | 2–3–1 | Below promotion threshold |
| 10 | Pivot about the opposite stationary wheel | Loop 03 | 0–6–0 | Reject |

## Loop 01 evidence

First contact occurred at 6.003 s. Challenger wins completed at 22.174–30.345 s. In seed 11, original sides, the original aggressive program logged 11 switches to search while the challenger retained forward pressure through intermittent/near-field echo losses. Near the 22.174 s result the original bot was pushed backward over the rim; its front color sensor could not see the trailing edge. This supports bounded commitment, not a claim that 400 ms is a hardware-calibrated optimum. Opening and search behavior also differ between these two complete programs; later comparisons isolate policy changes.

Next experiment: lengthen the same close-echo memory to 1000 ms and try to beat the incumbent directly.

## Loop 02 evidence

All six bouts reached the full 120 s limit, with first contact at 4.521 s. In seed 11 original sides both bots were still commanding 850 degrees/s at the end, with spinning wheels and nearly unchanged opponent ranges around 109 mm. Extra commitment alone did not overcome a sustained pushing stalemate. The 400 ms incumbent remains. Next experiment changes the actuation policy: use legal full motor duty during charge, with the same short memory and border priority. This tests the modeled torque limit; it does not increase available motor power.

## Loop 03 evidence

Full duty won all three swapped-side bouts at 78.540 s and drew the original-side bouts at 120 s. This meets the declared 75% promotion score, but the strong side dependence is a limitation, not general dominance. First contact was 4.483 s. The strategy still spends long periods pushing before a resolution. Next experiment introduces a 220 ms differential hook after 1800 ms of sustained attack, aiming to break head-on stalemates by changing the contact angle. The input remains elapsed attack time, never observer contact truth.

Visual strips for loops 01 and 02 confirm the reported ring-out and late head-on pushing respectively. Load the saved JSON into the web UI for the full sequence; strip captures round to the nearest 50 ms recorded frame.

## Loop 04 evidence

All bouts ended at 51.384 s with physical blue winning, regardless of which side ran the challenger. Event logs contained zero hook decisions: brief echo losses reset the continuous-attack timer before it reached 1800 ms. The candidate therefore behaved like its parent in these scenarios; side-swapped scoring correctly prevented a false promotion. Loop 05 retains duel timing across short signal gaps and clears it at border recovery or an absence longer than 1000 ms, so the new maneuver can actually be tested.

## Loop 05 evidence

Hooks now occurred (seed 11: 7.219, 47.370 and 49.172 s on the original side; 6.070, 7.885 and 30.299 s when swapped). Results remained side-dependent: three wins at 55.409 s and three losses at 32.401 s. Making the maneuver execute changed trajectories but did not establish superiority. Next experiment abandons the hook and uses gyro feedback to retain the heading at target acquisition while pushing, reducing accidental yaw without pretending the distance sensor supplies bearing.

## Loop 06 evidence

Gyro holding drew all six 120 s bouts despite physical contact at 4.466–4.495 s. Stabilizing the original attack direction did not overcome the static pushing duel. Reject the extra feedback complexity. Loop 07 changes reacquisition instead: scan continuously in one direction rather than alternate each second, avoiding repeated coverage of the same small arc after an echo disappears.

## Loop 07 evidence

Continuous search shortened matches but did not beat the incumbent: three wins at 14.785 s and three losses at 22.901 s, first contact 4.455 s. Faster resolution alone is not a promotion criterion. Next experiment explicitly searches for a better attack heading: sweep for 160 ms after acquiring an echo, remember the shortest measured range and its gyro heading, return to that heading within a 400 ms limit, then charge. This is a heuristic from successive range measurements, not direct target bearing. Border checks interrupt both sweep and alignment.

## Loop 08 evidence

The range-alignment approach won one bout at 22.051 s, lost one at 55.358 s, and drew the other four at 120 s. Unlike several earlier policies, its outcome changed with sensor-noise seed, exposing sensitivity to the shortest noisy reading and acquisition timing. The extra sweep/turn latency and alignment heuristic did not beat immediate pressure. Loop 09 tests a different opening geometry: touch white and turn, then apply the incumbent's full-duty/close-memory policy, instead of entering through the pivot opening.

## Loop 09 evidence

The white-line approach scored 58.3%: two wins at 19.318 s, three 120 s draws, and one loss at 40.402 s. That is the strongest distinct challenger so far, but below the predeclared 75% promotion threshold. It will be the distinct rival in held-out validation if loop 10 does not establish a stronger one. This result also warns that the incumbent is not a proven universal winner. Loop 10 tests entry geometry without the extra trip to the border: pivot about the opposite stationary wheel, then use the same pressure policy.

## Loop 10 evidence

Changing the stationary pivot wheel led to six draws at 120 s, with contact at 4.442 s. The changed lateral entry geometry did not defeat the incumbent. All ten planned loops are complete: 60 training bouts with zero program or rule faults. The selected incumbent is loop 03, and loop 09 is the strongest distinct challenger by training score. Programs are now frozen for fresh-seed validation; no further tuning uses held-out outcomes.

The inspected loop-10 replay exposes a separate failure mode: after the last contact at 45.797 s both bots remained apart, returning 2000 mm and oscillating through their search arcs until timeout. No-contact searching and prolonged physical pushing are distinct draw causes, even though the scoreboard labels both timeout.

The UI loads `pressure_full_duty` against its predecessor `pressure_memory` by default. `experiments/champion.json` records the promotion, compared opponent, training score and winning replay URL. The UI polls for new selections every 10 seconds and applies them when paused with untouched editors; explicit controls can load the latest best or inspect its recorded win. Edited programs, active matches and imported replays are protected from automatic replacement.

## Fresh-seed validation

Programs were frozen before using seeds 101, 211, 307 and 401 on both sides at nominal friction 0.8 and the same 120 s limit.

| Strategy | Opponent | W–D–L | Score |
|---|---|---|---|
| Full-duty pressure (03) | Original aggressive | 4–0–4 | 50% |
| Full-duty pressure (03) | White-line pressure (09) | 3–4–1 | 62.5% |
| Close-echo memory (01) | Original aggressive | 8–0–0 | 100% |
| Full-duty pressure (03) | Close-echo memory (01) | 4–4–0 | 75% |

The baseline split exposed a limitation of sequential head-to-head promotion: beating the previous incumbent does not guarantee better performance against earlier opponents. This triggered two additional validation matchups of the frozen programs, not new improvement loops or tuning: loop 01 versus the original baseline, and loop 03 versus loop 01. The fresh-seed loop-01 result above confirms the strongest direct baseline improvement. Scores remain descriptive across a few sensor-noise seeds; they are not independent physical trials or a statistical guarantee of real-world dominance.

## Friction sensitivity and original-loop conclusion

At tire friction 0.64 and 0.96, frozen full-duty pressure versus close-memory drew all four bouts per scenario (seeds 101/211, both sides). These are arbitrary sensitivity values, not measured distributions of real tire friction. The nominal head-to-head advantage did not survive these scenarios.

The original ten-loop investigation therefore comprises 100 completed bouts: 60 training, 32 fresh-seed comparisons and eight friction checks, with no recorded program/rule faults. Close-memory is the strongest demonstrated improvement over the original aggressive program; full duty is the sequential incumbent based on its direct predecessor matchup. Neither is a universal or hardware-validated winner. A fixed opponent pool is preferable to relying on sequential promotions alone.

An interruption occurred after both friction groups had saved their four complete 120-second replay files but before writing summaries. The two summaries were reconstructed from those intact replays, validating seed, source hashes/order, final results and end times. They explicitly record recoveredAfterInterrupt=true and wallSeconds=null; no timings or bouts were invented.

## Continuing with stall recovery

User review of the long pushing matches requested stall/lockup escape variants, geometry for left/right choices, travel since white, and hardware-grounded gyro use. This is an extension after the ten completed loops. Historical sources and all prior results remain immutable. See the stall-recovery plan for the two new controllers and the distinction between motor stall, tire slip, and repeated reacquisition.

Before implementing, official current hub/motor documentation and pinned v4.0.1 IMU code were researched and ingested into the wiki. Heading is clockwise-positive and unwrapped; gyro is not translation. Powered pushing cannot be detected with the IMU's stationary calibration flag, and wheel rotation can continue during chassis lockup. [Hardware contract and geometric limits](stall-recovery.md) describes the exact capabilities and inferred policy.

The first pilots retained sources `11_stall_hook.py` and `12_stall_geometry.py`. Review found that persistent white could cause unbounded reversing and the opening pivot had no deadline. The revised sources `11b_stall_hook.py` and `12b_stall_geometry.py` cap border retreat and opening, increase slip uncertainty across brief invalid returns, and add a repeated-encounter watchdog. Pilot records were retained unchanged. A third source `13_stall_pressure.py` then requires a fresh <140 mm return and uses forward wheel duty during escape, based on the pilot replays' false alerts and blocked counterrotation. No physics or hardware parameters changed.

| Experiment vs full-duty (03) | W–D–L | Decision / recovery evidence |
|---|---|---|
| 11 pilot: alternating release/hook | 3–3–0 | Score qualifies; not selected because bounded-phase fixes were needed |
| 12 pilot: uncertain geometry/full sweeps | 4–1–1 | Score qualifies; same phase fixes needed |
| 11b: bounded release/hook + watchdog | 5–1–0 | Load into UI as matchup incumbent |
| 12b: bounded geometry + watchdog | 0–3–3 | Reject; retain experimental preset |
| 13: fresh close return + forward hook | 0–1–5 | Reject; retain experimental preset |

Each row is six 120-second bouts, seeds 11/23/37 on both sides, nominal friction 0.8. The geometry candidate was compared against the same fixed opponent (03), not promoted 11b; these are parallel policy comparisons rather than additional sequential-incumbent promotions. The first two rows cannot be credited as verified lockup improvements simply from their score. Changed call timing and search/border behavior can change trajectories, even when a recovery action never fires.

In loop 11b, the original-side wins occurred at 27.362 s without a lockup alert. Swapped-side bouts alerted at 13.287 s, completed the escape at 14.013 s, and two won at 52.122 s while seed 11 drew. Visual inspection shows both bots already separated at that first alert: a false positive, not a physical pushing lockup. Seed 11 also alerted at 56.793 s and timed out its turn. This is why the program's score does not establish causal benefit from the escape routine.

Loop 12b's swapped-side replays show a close duel and a repeated-encounter alert at 32.647 s (reported 55–61 mm). The 28-degree turn did not execute: error remained about 28 degrees at the 800 ms deadline, followed by the bounded arc and continued searching/contact. All three swapped bouts drew. Its original-side bouts lost at 50.049 s. Loop 13 tested forward-powered yaw on the same encounter, still timed out, and lost five bouts. It did not solve the mechanically blocked duel. The revision's logs use the relevant watchdog history for heading span; older 11b/12b logs report the shorter pressure-history span, not necessarily the full five-second watchdog span.

Recovery visual strips are saved alongside `loop-11b/seed-23-swap-1.json`, `loop-12b/seed-11-swap-1.json` and `loop-13/seed-11-swap-1.json` as `*-recovery.png`. Generate with `node tools/review_replay.mjs --recovery <bout.json>`. They show before suspicion, alert, turn and completion; normal `*-sequence.png` strips show contact through final resolution. Full JSON replays remain the definitive event record.

## Frozen recovery validation and final selection

The revised hook was frozen before the following comparisons, seeds 101/211/307/401 on both sides, 120 s, friction 0.8. These seeds were fresh for this source but already used in the original investigation; they are not a new untouched project-wide statistical holdout.

| Strategy | Opponent | W–D–L | Score |
|---|---|---|---|
| Release/hook (11b) | Full-duty pressure (03) | 5–2–1 | 75% |
| Release/hook (11b) | Close-memory (01) | 0–4–4 | 25% |

The hook is a useful challenger to full duty in this model, but close-memory remains a stronger opponent for it. The UI's default is `stall_hook` versus `pressure_full_duty`; its label records the training score and direct opponent, not universal dominance. `stall_geometry`, `stall_pressure`, `pressure_memory` and historical baselines remain selectable. Refresh the page to populate newly added experimental presets in an already-open tab. The winning replay link points to seed 23 swapped in loop 11b, where the false alert is visible rather than hidden.

There are **146 completed bouts total**: the original 100, 30 recovery training/pilot bouts and 16 recovery validation bouts. No program or rule faults were recorded. [Tracked results](../experiments/results.json) retain source hashes, side mappings, scores, first-contact and lockup timestamps; full replay files are local generated artifacts rather than Git content. All five original physics/runtime/referee/sensor source hashes still match the frozen `93aab99` baseline. Tests and visual evidence verify software behavior, not physical gyro, traction, construction or universal strategy quality.
