# Observation logging and reaction timing

The live server sends copied event and ultrasonic batches to a background JSONL writer. The writer never reads or changes the physics world. A step request queues observations without waiting for disk formatting or writes; replay capture and event timestamps stay on the deterministic simulation thread.

Default runtime `sync-subset-v4` captures controller `print()` calls at the current simulated instant, then services the next hardware call. Adding diagnostic messages therefore does not consume extra simulated ticks. Hardware calls still use the existing assumed 1 ms slot, sensors still sample at 100 Hz, and physics still advances at 1 ms. Robot programs gain no threading or additional Pybricks APIs. Host formatting, copying and RPC transport still require wall time.

This is a simulator observation policy, not a claim that printing on a physical hub is free. Pybricks sends terminal output over its connection, and documents cooperative multitasking; its documentation does not establish a fixed print latency. Hardware output costs require measurement. Keep physical controller diagnostics sparse until calibrated.

The writer queue holds at most 128 batches. `/api/step`, `/api/replay`, `/api/events` and `/api/diagnostics` expose server-lifetime `observationLog` counters for pending, failed and dropped batches plus the latest error. Files may lag behind the live view. Overflow or disk failure does not stop the robots: the UI reports incomplete disk logging, and the full in-memory observations remain available through **Save replay**. Save before resetting if disk logging failed. Each batch retains its match's path, so an old queued batch cannot enter a new match's file. Shutdown attempts a bounded five-second drain; a stalled disk can leave files incomplete.

Prints count toward the 100,000-operation program budget. A burst of more than 1,024 prints without a hardware yield is rejected, and all CPU work/print transport before that yield shares the existing one-second wall-time budget. This bounds logging floods without adding virtual reaction delays.

## Historical results

Existing strategy scores and archived replays were measured under `sync-subset-v3`, where each print used an extra 1 ms scheduler slot. The selected `weave_hook` score is labeled historical in the UI; it has not been revalidated under v4. Rankings can change with scheduling, even when physics is identical. Archived programs and evidence remain unchanged.

Select the legacy profile explicitly to reproduce those trials:

```powershell
.venv/Scripts/python.exe -m tools.strategy_tournament challenger.py incumbent.py --runtime-profile sync-subset-v3 --out artifacts/legacy-check
.venv/Scripts/python.exe -m sumo.cli batch challenger.py incumbent.py --runtime-profile sync-subset-v3 --out artifacts/legacy-batch.json
```

Verification includes exact logged-versus-quiet motion, sensor and physical-event comparisons over a 12-second refereed `weave_hook` match; controlled blocked-disk tests; copied-batch ordering across reset paths; overflow/error and shutdown behavior; and the print-flood CPU deadline.

An archived v3 bout reproduced all 412 JSON-normalized frames and 177 events exactly, including its 20.527-second resolution. This is a compatibility check, not another independent strategy trial. `node tests/browser-observation.mjs` checks current metadata, the historical-score label, error visibility when the event count is unchanged, and continued robot execution after a disk-error notice.
