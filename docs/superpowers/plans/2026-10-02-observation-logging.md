# Observation logging implementation plan

**Goal:** Keep diagnostic capture and file persistence from delaying simulated robot reactions, with deterministic evidence and bounded background work.

**Design:** Drain controller print RPCs at the current simulated instant; real hardware RPCs retain one 1 ms slot. A bounded host-side JSONL writer owns disk I/O using copied batches, never reads or changes World, and exposes backlog/errors/dropped batches. Reset and shutdown preserve each match's path/order and attempt a bounded drain on shutdown. No robot threads or new Pybricks APIs.

**Compatibility:** Default runtime v4 declares zero-time observation prints. Optional legacy v3 preserves historical scheduling for reproducible old evidence. CLI/tournament metadata uses the actual runner profile. Existing selected 21b scores/replays remain explicitly v3 history until separately revalidated; never relabel them as v4 results.

- [x] Add failing print-neutrality, legacy timing and logging-flood tests; implement deterministic zero-tick print servicing without changing hardware scheduling or physics/sensor rates.
- [x] Add failing slow-writer, immutable batch, ordering/reset, error/overflow and shutdown tests; move live JSONL persistence to the background writer with observable status and replay fallback.
- [x] Test actual v4 controller trajectories with/without diagnostic prints; validate legacy v3 against an archived replay. Run applicable/full tests and independent review.
- [x] Verify UI sources/events, new metadata and historical score label; restart local server preserving the selected pair. Update wiki architecture/roadmap and source evidence, commit feature changes.

**Verification:** 155 Python tests pass. Browser match, saved replay, selection preservation and quiet-event logging-status checks pass. Printed/quiet `weave_hook` trajectories, sensor values and non-print events match exactly for 12 simulated seconds. Legacy reproduction matches 412 JSON-normalized frames and 177 events through the archived 20.527-second result. Independent review found and verified a fix for print bursts resetting the CPU timeout; v4 now shares a monotonic deadline. UI error-cache regression failed before its fix and passes afterward. Wiki capture/ingest and architecture/roadmap updates are committed and synced on main.

Real hub printing/transport/formatting costs remain uncalibrated. Zero-time diagnostic capture is an observation policy, not a claim that hardware print calls cost nothing. A background writer can reduce wall-time stalls; log-neutral scheduling removes the artificial simulated delay. Hardware calls still take the existing assumed 1 ms; strategy rankings need new-profile validation.
