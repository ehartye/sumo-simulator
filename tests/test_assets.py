import copy
import hashlib
import json

import pytest

from sumo.physics import Arena
from sumo.robot import ROOT, load_robot, make_model
import mujoco


def test_actual_spike_library_and_export_have_verified_dimensions_and_mass():
    robot = load_robot()
    assert robot['totalMassKg'] == pytest.approx(.35753)
    hub = next(p for p in robot['parts'] if p['name'] == 'hub')
    assert [b-a for a,b in zip(hub['sourceBounds']['min'], hub['sourceBounds']['max'])] == pytest.approx([.056,.032,.088], abs=1e-7)
    assert json.loads((ROOT/'artifacts/robot/verification.json').read_text())['ok']
    provenance = json.loads((ROOT/'assets/ldraw-provenance.json').read_text())
    # The committed dependency closure is pinned, not fetched at runtime.
    for path, digest in provenance['files'].items():
        assert hashlib.sha256((ROOT/'assets/ldraw'/path).read_bytes()).hexdigest() == digest
    assert {p['ldraw'] for p in robot['parts']} == set(provenance['roots'])


def test_connected_build_uses_stock_inventory_and_aligned_mates():
    from collections import Counter
    robot = load_robot()
    usage = json.loads((ROOT/'hardware/build-inventory.json').read_text())['usage']
    counts = Counter(p['ldraw'] for p in robot['parts'])
    assert set(counts) == {row['ldraw'] for row in usage}
    assert all(counts[row['ldraw']] == row['used'] <= row['available'] for row in usage)
    assert robot['connectivity']['allPartsConnected']
    assert len(robot['connectivity']['residuals']) == 61
    assert max(r['positionErrorMeters'] for r in robot['connectivity']['residuals']) < 1e-7


def test_illegal_mass_or_sensor_addition_cannot_enter_match(tmp_path):
    robot = load_robot()
    for mutation in ('mass', 'sensor', 'joint_rotation'):
        bad = copy.deepcopy(robot)
        if mutation == 'mass':
            bad['parts'][0]['massKg'] += .1
        elif mutation == 'sensor':
            bad['parts'].append({**bad['parts'][0], 'name': 'extra_sensor'})
        else:
            bad['bodies'][1]['rotation'] = [1, 0, 0, 0]
        path = tmp_path/f'{mutation}.json'
        path.write_text(json.dumps(bad))
        with pytest.raises(ValueError, match='Minimal-build|inventory'):
            load_robot(path)


def test_export_body_order_cannot_disconnect_wheels():
    robot = load_robot()
    robot['bodies'].reverse()
    model = mujoco.MjModel.from_xml_string(make_model(robot, Arena(), .001, .8))
    assert model.body('b0_left_wheel').parentid == model.body('b0_chassis').id
