# Damping and counterstrategies implementation plan

> Execute on the existing `feat/spike-sumo` feature workspace. User authorized continued experiments and a pivot to attacking the winner when improvements plateau.

**Goal:** Find a reproducible counter to selected loop 21b without changing hardware, physics, sensors, referee, or concurrency.

**Architecture:** Standalone Pybricks-subset example programs and immutable strategy archives. Derive yaw rate from the existing unwrapped heading and clock; observer data is used only in post-match analysis.

**Verification:** Paired sides, unchanged 1 ms timestep and two-worker batches, source hashes, zero program/rule faults, saved replays, fresh-seed validation, actual UI source/replay checks.

- [x] Test yaw estimation and corrective motor sign before implementing three matched short-pulse comparisons: active damping, forward-biased damping, passive braking control.
- [x] Run six bouts per candidate against loop 21b on seeds 11/23/37. Inspect rate, heading drift, separation and posture. Active damping plateaued below the qualifying passive control; the control proceeded to frozen validation.
- [x] Test three targeted counters: mirrored legal opening, heading-stabilized pushing, and an echo-triggered forward feint. All score at most 50% and fail qualification.
- [x] Bounded follow-up after passive control validates at only 3W/0D/5L: test patient full-duty pressure with eight-second close gate and 1500ms memory. Six draws fail qualification, so conditional seeds1777/1879/1993/2081 were not run. Do not retune loop30.
- [x] Freeze the strongest candidate and run eight fresh bouts against21b and eight each against duty03 and memory01 (seeds1301/1423/1553/1667). Result13W/0D/11L pooled,3W/0D/5L directly; no promotion or tuning. No reliably winning counter found.
- [x] Audit every added result and source hash; obtain independent code/evidence review. Publish compact evidence and limitations; verify143 Python tests and actual seven UI presets/replay event seeking. Threshold-qualified patient gaps gain only2.0–2.5mm; do not claim useful escape.
- [x] Update project wiki on main and sync (hardware-rate summary b7a9559; project evidence ecc3d1a), validate retrieval/links, commit feature changes, and report the observed outcome. Retain21b; no candidate earns promotion. The experiment round is complete; the aim of a reliably winning counter is unmet.
