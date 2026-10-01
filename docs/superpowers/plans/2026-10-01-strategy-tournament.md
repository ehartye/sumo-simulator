# Ten strategy improvement loops

**Goal:** Complete ten evidence-driven challenger experiments, identify the strongest demonstrated approach, then try to defeat it with a different approach.

**Architecture:** Keep the hardware, sensor model, motor model, rules and runtime frozen. Evolve plain synchronous Pybricks programs. A headless tournament harness records paired trials, events, source hashes and replay frames. Use existing 3D replay for visual inspection.

**Execution:** Follow writing-plans / subagent-driven-development for the independent replay-support task, then sequential experiment construction and evaluation. Current feature branch isolates this work from main.

**Comparison contract:** Training seeds 11, 23, 37, each side swapped; 120-second match limit; score win=1, draw=0.5, loss=0. Promote only at score >= 0.75 with zero challenger rule/program faults. Record failures as completed experiments. Choose each new hypothesis using the preceding event evidence. Do not change physics to favor a strategy. Validate the final selected strategy on unused seeds 101, 211, 307, 401, both sides, against the original baseline and strongest distinct rival. Separately stress tire friction at 0.64/0.96; report sensitivity without tuning on these holdouts. Seeds vary sensor noise, not manufacturing tolerances.

- [x] Add tested experiment scoring and reproducible paired-bout runner in `tools/strategy_tournament.py`; retain every candidate under `experiments/strategies/`, immutable numbered results/replays under `artifacts/strategy-tournament/`.
- [x] Add saved-run loading to existing viewer, preserving live match controls and explicitly separating loaded replay from a new live run; test and visually inspect.
- [ ] Execute loops 01–10 sequentially. Record hypothesis, exact program, incumbent, six outcomes, event-based diagnosis, visual evidence, promotion decision and next hypothesis in `docs/strategy-tournament.md`.
- [ ] Run held-out comparisons and traction stress checks. Audit legality, contact, decision timing and losses, including at least one visual sequence leading to resolution.
- [ ] Put the winning program in examples without overwriting historical baselines. Add regression coverage for legal startup and physical engagement; run relevant tests and browser verification; request independent code/evidence review.
- [ ] Update wiki evidence/architecture/roadmap, publish report and leave the UI ready to inspect the winning approach.

**Limits:** Wins are model-conditioned. A short-lived echo can vanish below the model's rated minimum range; any persistence policy must stay bounded and retain border priority. The controller must never receive the observer's target identity, pose, contact truth or outcome. Draws and non-transitive matchups remain visible; ten loops do not imply ten promotions or universal dominance.
