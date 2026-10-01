# Evidence and the accuracy boundary

The current profile is **uncalibrated**. Do not use match win rate as proof that it predicts a physical fight.

## Sourced inputs

- LDraw official library downloaded 2026-10-01; archive hash and 279-file dependency closure recorded in `assets/ldraw-provenance.json`. Nominal scale is 0.4 mm/LDU; Agent Meshes rotates 180° around X to make +Y up, then centers each part. [LDraw specification](https://www.ldraw.org/article/218.html).
- LEGO medium angular motor 45603 at 7.2 V: nominal free speed 185 rpm, stall torque 18 N·cm, stall current 800 mA. Free-speed tolerance ±15%; encoder 1° resolution, ±3° accuracy and 100 Hz update. The model linearly interpolates the torque-speed endpoints, which is an approximation. [LEGO sheet](https://assets.education.lego.com/v3/assets/blt293eea581807678a/blt692436dd1e8fa71c/5f8801d5c8a27c1d9614c27e/techspecs_technicmediumangularmotor.pdf?locale=en-us).
- Nominal ultrasonic range 50–2000 mm, ±20 mm accuracy and approximately ±35° cone, with range-dependent response. The simulator samples a dense circular 3D cone, not acoustic propagation. [LEGO sheet](https://assets.education.lego.com/v3/assets/blt293eea581807678a/blt64c2b9534cf10f68/5f8801b8bc43790f5c4389ea/techspecs_technicdistancesensor.pdf?locale=en-us).
- Hub nominal 56 × 32 × 88 mm, consistent with imported source bounds. [LEGO sheet](https://assets.education.lego.com/v3/assets/blt293eea581807678a/bltf512a371e82f6420/5f8801baf4f4cf0fa39d2feb/techspecs_techniclargehub.pdf?locale=en-us).
- Part-level mass records: BrickLink IDs `53444c01`, `55422c01`, `54696c01`, `37308c01`, `37316c01`, `39367pb01`, `39370`, `52629`, `32524`, `2780`, `4519`, `64179`, `39793`, `6558`. All exact URLs are in the assembly manifest; retail packaging dimensions/mass were not used.
- [Pybricks synchronous/cooperative tools](https://docs.pybricks.com/en/stable/tools/index.html). Upstream virtual motor simulation does not provide a ready-made external chassis-load coupling in the examined [pinned driver](https://github.com/pybricks/pybricks-micropython/blob/81bb7272c8296e087085fef4d71d4f9c98c8ebc7/lib/pbio/drv/motor_driver/motor_driver_virtual_simulation.c). The current CPython subset is a deliberate initial implementation; native firmware coupling remains future work.

The base-set stock manifest preserves 112 replacement-catalog rows; alternative molds are not additive inventory. The build uses 43 rendered parts, 61 rest-position mates and approximately 357.53 g catalog mass (slotted 2780 pin uses the source's approximate 0.15 g note). The motor output disc is still visually fixed to the motor case, so moving connector fidelity is incomplete. Current checks do not certify interference freedom or physical assembly.

## Assumed parameters

| Model | Current assumption | Measurement needed |
|---|---|---|
| Mass distribution | Catalog mass at each part's box center, uniform proxy inertia | Assembled mass, balance points, oscillation/inertia estimate |
| Mechanics | Perfect rigid attachments, no self-contact or flex | Rest-position anchors checked; verify assembly sequence, fit and stiffness |
| Tire contact | Sliding coefficient .8, torsional .0005, rolling .00001 | Pull/slip tests on actual ring, wheel load dependence |
| Passive contact | Caster .25; plastic body .3 | Caster drag and plastic/ring friction |
| Battery | 7.2 V supply minus .35 Ω × total motor current, 5 V floor | Loaded voltage/current traces and protection behavior |
| Motor control | Linear back-EMF model, simple proportional speed/hold loops | Loaded ramps, stall/push force, braking, turn response |
| Color | 6 mm border transition, black 8/white 88, 0.8-point noise | Actual reflectance vs surface, height, tilt and ambient light |
| Ultrasonic | Fixed circular ±35° cone sampled at 1°; 45° per-ray surface-incidence rejection; 5 mm Gaussian noise; below-rated-range returns marked invalid | Measured beam shape vs range, surface/material response, near field, cross-talk and latency |
| IMU | .05°/s drift plus .08° sample noise | Heading bias/drift under motion and vibration |
| Encoders | 1° quantization at 100 Hz, finite-difference speed | Real filtered speed, latency and zeroing behavior |

These values are centralized in the small physics/runtime modules. They are not fitted or certified. A 2100 mAh battery rating is capacity, not a maximum power specification. The current model has torque and estimated current limits per motor, but does not reproduce battery protection or thermal shutdown.

## Calibration procedure

1. Assemble this exact inventory; verify connector fit, wheel alignment and sensor orientation. Record any manifest corrections and new hashes. Weigh the complete robot and measure envelope/center of mass.
2. Record wheel angle, current, supply voltage and ground displacement for forward/reverse ramps at several battery states. Record coast, brake and hold stopping distances. Use separate low/high-friction surfaces to distinguish drive and contact errors.
3. Measure static pull force and pushing/slip under known loads. Measure turns and head-on/glancing contact, wheel lift and ring departure. Fit mass/contact/controller parameters against a designated training subset.
4. Map color readings and ultrasonic detections over height, tilt, angle and distance. Measure cadence, latency and missed detections. Fit observation models separately from strategies.
5. Before fitting, set tolerances for speed, stopping distance, turn angle, push force and detection error based on intended use and measurement uncertainty. Keep separate, previously unseen trials for final evaluation. Publish predicted/measured traces, errors and parameter confidence, including failed trials.

Only after the held-out results meet those predeclared tolerances should the profile be labeled calibrated. A native Pybricks runtime would improve software equivalence; it would not remove the need for these physical measurements.

Ultrasonic source specifications, live experimental evidence, model assumptions and automatic diagnostic logs are documented in [ultrasonic.md](ultrasonic.md). The 50 mm rated minimum does not establish real sensor behavior at contact.
