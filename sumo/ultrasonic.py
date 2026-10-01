"""Geometric echo approximation, with explicit diagnostics; not acoustic calibration."""
import math

import mujoco
import numpy as np


HALF_ANGLE = 35
MIN_RANGE = .05
MAX_RANGE = 2.
# A dense angular grid closes the old 8.75-degree blind strips. Keep only
# directions inside a circular cone, not a square horizontal/vertical frustum.
angles = np.radians(np.arange(-HALF_ANGLE, HALF_ANGLE+1))
yaw, pitch = np.meshgrid(angles, angles)
_directions = np.column_stack([np.sin(yaw.ravel())*np.cos(pitch.ravel()),
                              np.sin(pitch.ravel()), np.cos(yaw.ravel())*np.cos(pitch.ravel())])
DIRECTIONS = _directions[_directions[:, 2] >= math.cos(math.radians(HALF_ANGLE))-1e-12]
# Smooth surfaces at grazing incidence reflect sound away from the receiver.
# This 45-degree cutoff is an uncalibrated approximation, not a LEGO spec.
MIN_INCIDENCE_COSINE = math.cos(math.radians(45))
PROFILE = {
    'model': 'geometric-cone-v2', 'calibrated': False, 'rangeMm': [50, 2000],
    'halfAngleDegrees': HALF_ANGLE, 'angularGridDegrees': 1, 'rays': len(DIRECTIONS),
    'sampleRateHz': 100, 'resolutionMm': 1, 'specifiedAccuracyMm': 20, 'noiseStandardDeviationMm': 5,
    'assumptions': ['Fixed circular cone; real entrance angle depends on range and target.',
                    'Nearest first-surface echo with incidence within 45 degrees of its normal.',
                    'Primitive collision proxies; no multipath, material absorption or sensor cross-talk.',
                    'Below the rated 50 mm minimum, this model reports invalid (2000); this is not a hardware specification. Actual close-contact readings may differ.',
                    '5 mm Gaussian noise is assumed, not a fit to the specified +/-20 mm accuracy.'],
    'source': 'https://assets.education.lego.com/v3/assets/blt293eea581807678a/blt64c2b9534cf10f68/5f8801b8bc43790f5c4389ea/techspecs_technicdistancesensor.pdf?locale=en-us',
    'apiSource': 'https://docs.pybricks.com/en/stable/pupdevices/ultrasonicsensor.html',
    'measurementReference': {
        'source': 'https://afrel.co.jp/product/spike/technology-spike/prime/hardware/distancesensor/726/',
        'conditions': 'Afrel July 2019 test: flat steel wall, sensor 50 mm above floor; firmware not specified.',
        'observations': 'Approximately +/-20 mm at 50..1800 mm, no valid reading at 1850 mm or more, and 40 mm indicated at contact.',
        'scope': 'One experiment, explicitly not a guaranteed range; not fitted to this robot or Pybricks firmware.',
    },
}


def sample(model, data, bot, origin, rotation, rng):
    directions = np.ascontiguousarray(DIRECTIONS @ rotation.T)
    groups = np.ones(6, dtype=np.uint8)
    groups[bot+1] = 0  # Exclude the entire emitting robot, including moving bodies.
    ids = np.empty(len(directions), dtype=np.int32)
    distances = np.empty(len(directions))
    mujoco.mj_multiRay(model, data, origin, directions.reshape(-1), groups, 1, -1,
                      ids, distances, len(directions), MAX_RANGE)
    hits = (ids >= 0) & (distances >= 0) & (distances <= MAX_RANGE)
    echo = np.zeros(len(directions), dtype=bool)
    # MuJoCo 3.3's batch ray API has no normals output. Reconstruct normals
    # for the primitive proxies actually used by this simulator.
    for geom in np.unique(ids[hits]):
        indices = np.flatnonzero(hits & (ids == geom))
        transform = data.geom_xmat[geom].reshape(3, 3)
        points = origin + distances[indices, None]*directions[indices]
        local = (points-data.geom_xpos[geom]) @ transform
        normals = np.zeros_like(local)
        shape = model.geom_type[geom]
        if shape == mujoco.mjtGeom.mjGEOM_BOX:
            axis = np.argmax(np.abs(local)/model.geom_size[geom], axis=1)
            normals[np.arange(len(indices)), axis] = np.sign(local[np.arange(len(indices)), axis])
        elif shape == mujoco.mjtGeom.mjGEOM_SPHERE:
            normals = local / np.maximum(np.linalg.norm(local, axis=1)[:, None], 1e-12)
        elif shape == mujoco.mjtGeom.mjGEOM_CYLINDER:
            caps = np.abs(np.abs(local[:, 2])-model.geom_size[geom, 1]) < 1e-7
            normals[caps, 2] = np.sign(local[caps, 2])
            normals[~caps, :2] = local[~caps, :2] / np.maximum(np.linalg.norm(local[~caps, :2], axis=1)[:, None], 1e-12)
        elif shape == mujoco.mjtGeom.mjGEOM_PLANE:
            normals[:, 2] = 1
        else:
            raise ValueError('Ultrasonic normals require a supported primitive proxy')
        incidence = -np.sum(directions[indices]*(normals @ transform.T), axis=1)
        echo[indices] = incidence >= MIN_INCIDENCE_COSINE
    candidates = np.flatnonzero(echo)
    selected = int(candidates[np.argmin(distances[candidates])]) if len(candidates) else None
    ideal = float(distances[selected]) if selected is not None else None
    reason = 'no_echo' if ideal is None else 'below_minimum' if ideal < MIN_RANGE else 'echo'
    measured = int(np.clip(round(ideal*1000+rng.normal(0, 5)), 50, 2000)) if reason == 'echo' else 2000
    return {
        'bot': bot, 'sampleTime': float(data.time), 'origin': origin.tolist(),
        'axis': rotation[:, 2].tolist(), 'coneHalfAngleDegrees': HALF_ANGLE,
        'rayCount': len(directions), 'distanceMm': measured, 'reason': reason,
        'idealDistanceMm': None if ideal is None else ideal*1000,
        'hitObject': None if selected is None else model.geom(int(ids[selected])).name,
        'hitPoint': None if selected is None else (origin+ideal*directions[selected]).tolist(),
        'counts': {'miss': int((~hits).sum()), 'grazing': int((hits & ~echo).sum()),
                   'belowMinimum': int((echo & (distances < MIN_RANGE)).sum()),
                   'inRange': int((echo & (distances >= MIN_RANGE)).sum())},
    }
