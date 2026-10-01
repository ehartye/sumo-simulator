# Verification record — 2026-10-01

- `npm run build:robot`: valid GLB and physical export, 43 parts, 61 mates, one connected component, approximately 357.53 g. Nominal envelope 103.6 × 80.0 × 158.8 mm.
- `.venv/Scripts/python -m pytest -q`: 68 passed. One dependency deprecation warning from Starlette/AnyIO.
- `node tests/browser.mjs`: passed; 12-second bounded bout, 241 replay frames, both opening paths engaged, no browser errors. Sensor revision also verified 2402 clock-driven samples, selected echo rays, status and complete diagnostic export. Verified run/pause/step, completion, fault replay, reset, mobile overflow and standalone robot review. An initial attempt during the full Agent Meshes integration suite timed out after the bout; the standalone rerun passed.
- Paired CLI bouts, seed 7, 30-second limit: both completed as timeout draws with no program errors. These smoke runs demonstrate execution, not strategy quality or calibrated performance.
- Independent runtime review additionally checked seeds 0, 7 and 42: both opening examples completed legally.
- Agent Meshes assembly/connectivity: 33 tests passed; TypeScript typecheck passed. Feature commits follow remote main f5fa7ee and are recorded on `feat/spike-robot-assets`.
- Full Agent Meshes suite is **not green**: Blender/Unreal integration errors and timeouts; run interrupted. An unchanged upstream archive reproduces the Blender skin-sample failure because installed Blender 4.2.3 lacks `AnimData.action_slot`. Its seven carving tests pass in isolation. Temporary baseline files remain under the sibling's `.cache/spike-baseline` after automatic approval blocked cleanup; they are not committed.

Independent static review found and verified fixes for tire/bracket overlap, caster/frame overlap, excessive axle insertion and duplicate pins in shared frame holes. The revised source anchors align within approximately 1.3e-12 m; that residual reflects nominal authoring arithmetic, not physical manufacturing precision. Actual axle tips clear the modeled socket stops by approximately 0.2 mm.

No physical specimen has been assembled, weighed or calibrated. Static connections do not certify insertion sequence, cable routing, strength or all possible interference. The motor output disc remains visually fixed while wheel axles/pins rotate. MuJoCo uses explicit collision proxies and approximate inertia/contact/control/sensors. See [calibration](calibration.md) before interpreting predicted wins as real robot performance.

Wiki evidence, architecture and roadmap were committed and pushed on wiki `main` through f275940a. Code remains on feature branches; no code push or merge was performed.

Ultrasonic revision: the old nine-ray fan's angular blind strips are covered by regression tests. New tests cover a dense 3D cone, mounting orientation, vertical detection, occlusion, whole-robot self-exclusion, hardware-clock sampling, log/metadata persistence and CLI logs. A reviewed 12-second seed-7 browser run produced 442 opposite-robot echoes and 1960 no-echo samples. Hardware specifications and uncalibrated policies are separated in [ultrasonic.md](ultrasonic.md).
