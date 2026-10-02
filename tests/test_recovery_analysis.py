import importlib
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest


def replay(contact_events, end=4, alert=2, separated_at=None, heading_change=0):
    frames = []
    for tick in range(round(end*10)+1):
        time = tick/10
        heading = 350 + (heading_change if time >= alert+1.5 else 0)
        angle = math.radians(heading)/2
        bots = [{'bodies': {'chassis': {'position': [0, .03, 0],
                                      'rotation': [0, math.sin(angle), 0, math.cos(angle)]}},
                 'sensors': {'heading': heading % 360}}]
        distance = .5 if separated_at is not None and time >= separated_at else .15
        bots.append({'bodies': {'chassis': {'position': [distance, .03, 0],
                                           'rotation': [0, 0, 0, 1]}},
                     'sensors': {'heading': 0}})
        frames.append({'time': time, 'bots': bots})
    return {'metadata': {'programHashes': ['challenger', 'opponent'], 'seed': 11},
            'frames': frames, 'events': sorted(contact_events + [
                {'type': 'program_log', 'bot': 0, 'time': alert,
                 'message': 'suspected lockup repeated close encounters range mm 60 turn 1'}],
                key=lambda event: event['time'])}


def analyze(data, **kwargs):
    module = importlib.import_module('tools.recovery_analysis')
    return module.analyze_replay(data, **kwargs)


def contact(kind, time):
    return {'type': 'contact_'+kind, 'time': time}


def test_alert_after_separation_does_not_credit_existing_no_contact_as_escape():
    data = replay([contact('started', .5), contact('ended', 1)], separated_at=1)
    alert = analyze(data, source_hash='challenger')['alerts'][0]
    assert alert['lastPhysicalContactTime'] == 1
    assert alert['contactLagSeconds'] == 1
    assert alert['wasCloseDuel'] is False
    assert alert['priorNoContactSeconds'] == 1
    assert alert['newSustainedSeparation'] is False
    assert alert['sustainedContactFreeIntervals'][0]['beganBeforeAlert'] is True
    assert alert['detectionReason'] == 'repeated close encounters'
    assert alert['event'] == data['events'][-1]


def test_short_proxy_contact_gaps_are_jitter_not_sustained_separation():
    events = [contact('started', .5)]
    for time in (2.1, 2.4, 2.7, 3, 3.3):
        events += [contact('ended', time), contact('started', time+.1)]
    alert = analyze(replay(events), bot=0)['alerts'][0]
    assert alert['physicalContactAtAlert'] is True
    assert alert['wasCloseDuel'] is True
    assert alert['newSustainedSeparation'] is False
    assert alert['sustainedContactFreeIntervals'] == []
    assert len(alert['shortContactFreeGaps']) == 5
    assert alert['preGroundDisplacementM'] == 0
    assert alert['minPairDistanceBeforeM'] == pytest.approx(.15)
    assert alert['minPairDistanceAfterM'] == pytest.approx(.15)


def test_contact_then_sustained_separation_records_achieved_heading_and_timing():
    data = replay([contact('started', .5), contact('ended', 2.1),
                   contact('started', 3)], separated_at=2.2, heading_change=30)
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['lastPhysicalContactTime'] == 2
    assert alert['contactLagSeconds'] == 0
    assert alert['newSustainedSeparation'] is True
    interval = alert['sustainedContactFreeIntervals'][0]
    assert interval['start'] == 2.1
    assert interval['end'] == 3
    assert interval['durationSeconds'] == pytest.approx(.9)
    assert alert['physicalHeadingChangeDegrees'] == pytest.approx(30)
    assert alert['sensorHeadingChangeDegrees'] == pytest.approx(30)
    assert alert['postSampleTimes'] == [2, 3.5]
    assert alert['pairDistanceAfterM'] == pytest.approx(.5)
    assert alert['maxPairDistanceAfterM'] == pytest.approx(.5)
    assert alert['pairDistanceGrowthM'] == pytest.approx(.35)


def test_short_replay_marks_incomplete_motion_windows_unavailable():
    data = replay([], end=.6, alert=.2)
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['preGroundDisplacementM'] is None
    assert alert['physicalHeadingChangeDegrees'] is None
    assert alert['sensorHeadingChangeDegrees'] is None
    assert alert['newSustainedSeparation'] is None
    assert alert['lastPhysicalContactTime'] is None
    assert alert['contactLagSeconds'] is None
    assert alert['wasCloseDuel'] is None
    assert alert['preWindowComplete'] is False
    assert alert['postWindowComplete'] is False


def test_cli_maps_source_hash_on_both_sides_and_preserves_bout_bytes(tmp_path):
    left = replay([contact('started', .5)])
    right = replay([contact('started', .5), contact('ended', 2.1)], separated_at=2.2)
    right['metadata']['programHashes'].reverse()
    right['events'][-2]['bot'] = 1
    right['events'][-2]['message'] = 'watchdog repeated close encounters range mm 60'
    paths = [tmp_path/'left.json', tmp_path/'right.json']
    for path, data in zip(paths, [left, right]):
        path.write_text(json.dumps(data), encoding='utf-8')
    original = [path.read_bytes() for path in paths]
    command = [sys.executable, str(Path(__file__).resolve().parents[1]/'tools/recovery_analysis.py'),
               *map(str, paths), '--source-hash', 'challenger', '--aggregate']
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    report = json.loads(result.stdout)
    assert [bout['bot'] for bout in report['bouts']] == [0, 1]
    assert report['aggregate']['alerts'] == 2
    assert report['aggregate']['newSustainedSeparations'] == 1
    assert report['bouts'][1]['alerts'][0]['event']['message'].startswith('watchdog')
    assert [path.read_bytes() for path in paths] == original


def test_equal_start_end_heading_does_not_hide_achieved_turn_and_return():
    data = replay([contact('started', .5)])
    # Turn 45 degrees then return before the end of the observation window.
    for frame in data['frames']:
        if 2.2 <= frame['time'] <= 3:
            frame['bots'][0]['sensors']['heading'] = 35
            angle = math.radians(395)/2
            frame['bots'][0]['bodies']['chassis']['rotation'] = [0, math.sin(angle), 0, math.cos(angle)]
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['physicalHeadingChangeDegrees'] == pytest.approx(0)
    assert alert['physicalHeadingSpanDegrees'] == pytest.approx(45)
    assert alert['sensorHeadingSpanDegrees'] == pytest.approx(45)


def test_close_but_dithering_chassis_is_not_low_travel_close_duel():
    data = replay([])
    for index, frame in enumerate(data['frames']):
        frame['bots'][0]['bodies']['chassis']['position'][0] = .015 if index % 2 else 0
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['preGroundDisplacementM'] == 0
    assert alert['preGroundTravelM'] > .02
    assert alert['wasCloseDuel'] is False
    assert alert['newSustainedSeparation'] is False


def test_missing_pose_heading_and_ambiguous_hash_are_unavailable_or_rejected():
    data = replay([])
    for frame in data['frames']:
        del frame['bots'][0]['bodies']['chassis']['rotation']
        del frame['bots'][0]['sensors']['heading']
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['physicalHeadingChangeDegrees'] is None
    assert alert['sensorHeadingChangeDegrees'] is None
    data['metadata']['programHashes'] = ['challenger', 'challenger']
    with pytest.raises(ValueError, match='exactly one'):
        analyze(data, source_hash='challenger')


def test_long_contact_proxy_gap_without_chassis_separation_is_not_escape():
    data = replay([contact('started', .5), contact('ended', 2.1), contact('started', 3.2)])
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['newSustainedContactFreeInterval'] is True
    assert alert['newSustainedSeparation'] is False
    assert alert['sustainedDistantContactFreeIntervals'] == []


def test_separation_requiring_missing_pair_pose_is_unknown():
    data = replay([contact('started', .5), contact('ended', 2.1)])
    for frame in data['frames']:
        if frame['time'] >= 2:
            del frame['bots'][1]['bodies']['chassis']['position']
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['newSustainedContactFreeInterval'] is True
    assert alert['newSustainedSeparation'] is None


def test_open_contact_at_last_frame_is_still_observed_contact():
    data = replay([contact('started', .5)], end=2)
    alert = analyze(data, bot=0)['alerts'][0]
    assert alert['physicalContactAtAlert'] is True


def test_inverted_chassis_is_reported_separately_from_close_duel():
    data = replay([contact('started', .5)])
    for frame in data['frames']:
        if frame['time'] >= 1.5:
            frame['bots'][0]['bodies']['chassis']['rotation'] = [1, 0, 0, 0]
            frame['bots'][0]['bodies']['chassis']['position'][1] = .069
    report = analyze(data, bot=0)
    posture = report['posture'][0]
    assert posture['maxTiltDeg'] == pytest.approx(180)
    assert posture['firstTiltAbove45'] == 1.5
    assert posture['firstTiltAbove90'] == 1.5
    alert = report['alerts'][0]
    assert alert['wasCloseDuel'] is True
    assert alert['preTiltDeg'] == pytest.approx(180)
    assert alert['postTiltDeg'] == pytest.approx(180)
    assert alert['preRootYM'] == pytest.approx(.069)
    assert alert['postRootYM'] == pytest.approx(.069)
