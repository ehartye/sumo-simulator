# SPIKE ultrasonic sensor: evidence and diagnostics

The model fixes numerical blind strips in the original nine-ray horizontal fan. It does not constitute measured acoustic calibration.

## Hardware and API evidence

[LEGO's Technic Distance Sensor specification](https://assets.education.lego.com/v3/assets/blt293eea581807678a/blt64c2b9534cf10f68/5f8801b8bc43790f5c4389ea/techspecs_technicdistancesensor.pdf?locale=en-us) gives normal distance sensing a 50–2000 mm range, ±20 mm accuracy, 1 mm resolution, 100 Hz sensor sampling, and approximately ±35° entrance angle that varies with distance. Its separate fast-range specification is 50–300 mm with ±15 mm accuracy; the simulator does not expose a separate fast mode.

[Pybricks v4 `UltrasonicSensor.distance()`](https://docs.pybricks.com/en/stable/pupdevices/ultrasonicsensor.html) returns millimeters, or 2000 when no valid reading was measured. A displayed 2000 therefore need not mean an object is exactly two meters away.

Afrel published two primary experiments in 2019, with the sensor 50 mm above the floor and a flat steel wall. Its [range test](https://afrel.co.jp/product/spike/technology-spike/prime/hardware/distancesensor/726/) reports approximately ±20 mm error over 50–1800 mm, invalid readings from 1850 mm, and 40 mm indicated at contact. Its [angle test](https://afrel.co.jp/product/spike/technology-spike/prime/hardware/distancesensor/45941/) shows increasingly inaccurate and asymmetric returns with angle. These are one experimental setup, not guaranteed limits; firmware and the robot's plastic surfaces differ. Do not substitute that test's range or angle for universal hardware limits. The wiki capture tool could not retain the short Japanese articles; those observations were checked on the live pages and original figures.

## Explicit model assumptions

- A fixed circular ±35° cone uses a 1° angular grid (3897 rays). The real beam's range dependence and vertical/horizontal shape have not been measured here.
- Origin and orientation follow the eyes part and its body transform. All parts of the emitting robot are excluded. Queries use the existing primitive collision proxies.
- Rays stop at their first surface. Surface normals reject rays more than 45° from the normal, approximating the loss of grazing reflections. **45° is an uncalibrated assumption, not a LEGO specification or a fitted Afrel result.** It prevents a horizontal sensor's downward cone from treating the smooth ring as a nearby opponent. No ray continues through a rejected opaque surface.
- The nearest remaining geometric echo is used. Echo strength, target area, material absorption, multipath and interference between sensors are absent.
- Below 50 mm, the model reports invalid/2000 and logs `below_minimum`. This is a conservative out-of-rated-range policy, **not measured near-field behavior**. The Afrel contact result demonstrates that actual hardware may return a short valid-looking number instead. Strategies depending on contact-range readings need bench validation.
- Valid distances have 5 mm standard-deviation Gaussian noise and 1 mm integer quantization. This assumed distribution is not equivalent to LEGO's ±20 mm accuracy specification and does not model angle-dependent bias.
- Both sensors sample at simulation-clock 10 ms boundaries; programs and the viewer receive the cached sample. Sampling and logging continue without reads. At arbitrary physics timesteps that do not divide 10 ms, sampling occurs at the first completed step reaching each boundary.

## Automatic logs

The local server creates a unique `.cache/runs/<timestamp>-<id>-ultrasonic.jsonl` for each match. A matching `.metadata.json` preserves programs, seed, robot hash, engine, rules and sensor assumptions. Reset creates new files. The logs are local and are not uploaded.

`GET /api/diagnostics` returns current samples and file paths. Saved replay JSON includes every sample under `ultrasonicSamples`; each frame also includes the cached diagnostic under `bots[i].ultrasonic`. Robot programs receive only the normal device values, not these diagnostic scene details. CLI batches store separate JSONL files under `sensor-logs/` beside the batch report; the report identifies each file, seed, side swap and sensor model.

Each sample records:

| Field | Meaning |
|---|---|
| `bot`, `sampleTick`, `sampleTime` | Which sensor sampled and when |
| `origin`, `axis` | World-space sensor face and forward axis |
| `distanceMm`, `reason` | Program-visible reading and `echo`, `no_echo` or `below_minimum` |
| `idealDistanceMm` | Selected geometric distance before noise, including an invalid near echo |
| `hitObject`, `hitPoint` | Selected proxy and world-space intersection, or null |
| `rayCount`, `counts` | Sampling density, misses, grazing rejections, too-near and in-range hits |

The viewer's Sensors toggle draws the selected ray and its intersection, including a selected invalid near echo. No echo draws no ray. The lines show the computed geometric return, not the full acoustic beam.

## Regression evidence

Before the fix, a robot one meter away alternated between an approximately 817 mm reading and 2000 at 0°, 4°, 8° and 12° because nine rays left 8.75° gaps. Tests cover those gaps, vertical targets, out-of-cone rejection, mounting rotation, opaque occlusion, near-field policy, fixed sample timing, disk/replay equivalence and reset preservation. These are simulation correctness tests; passing them does not validate acoustic fidelity.
