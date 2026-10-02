"""Offline recovery diagnostics from saved replays; never supplies bot inputs.

Contact transitions are the simulator's collision-proxy observations. Chassis
positions, quaternion headings and sensor headings are sampled replay evidence,
not calibrated hardware measurements. A new sustained separation is descriptive,
not proof that a recovery caused an escape or improved the match outcome.
"""
import argparse
import json
import math
from pathlib import Path


PRE_SECONDS = 1.
POST_SECONDS = 1.5
CONTACT_RECENCY_SECONDS = .2
SEPARATION_SECONDS = .5
CLOSE_DISTANCE_M = .18
LOW_TRAVEL_M = .02


def _contact_intervals(events, start, end):
    """Return contact and contact-free intervals from exact transition times."""
    touching = False
    since = start
    contacts, free = [], []
    for event in events:
        if event['type'] not in ('contact_started', 'contact_ended'):
            continue
        time = event['time']
        if time > end:
            break
        following = event['type'] == 'contact_started'
        if following != touching:
            if time >= start:
                (contacts if touching else free).append((since, time, True))
            touching, since = following, max(start, time)
    (contacts if touching else free).append((since, end, False))
    return contacts, free


def _position(frame, bot):
    return frame['bots'][bot].get('bodies', {}).get('chassis', {}).get('position')


def _distance(left, right):
    if left is None or right is None:
        return None
    return math.hypot(left[0]-right[0], left[2]-right[2])


def _heading(frame, bot, sensor):
    body = frame['bots'][bot]
    if sensor:
        return body.get('sensors', {}).get('heading')
    quaternion = body.get('bodies', {}).get('chassis', {}).get('rotation')
    if quaternion is None:
        return None
    x, y, z, w = quaternion  # Saved poses use xyzw, with local +Z forward.
    forward_x = 2*(x*z+w*y)
    forward_z = 1-2*(x*x+y*y)
    if math.hypot(forward_x, forward_z) < 1e-8:
        return None  # Upright heading is undefined for a vertical forward axis.
    return math.degrees(math.atan2(forward_x, forward_z))


def _heading_motion(frames, bot, sensor):
    headings = [_heading(frame, bot, sensor) for frame in frames]
    if len(headings) < 2 or any(value is None for value in headings):
        return None, None
    unwrapped = [0.]
    for left, right in zip(headings, headings[1:]):
        unwrapped.append(unwrapped[-1] + (right-left+180) % 360-180)
    return unwrapped[-1], max(unwrapped)-min(unwrapped)


def _tilt(frame, bot):
    quaternion = frame['bots'][bot].get('bodies', {}).get('chassis', {}).get('rotation')
    if quaternion is None:
        return None
    x, _, z, _ = quaternion
    # Dot(local +Y rotated into world, world +Y); 180 degrees means inverted.
    return math.degrees(math.acos(max(-1., min(1., 1-2*(x*x+z*z)))))


def _posture(frames, bot):
    samples = [(frame['time'], _tilt(frame, bot)) for frame in frames]
    available = [(time, tilt) for time, tilt in samples if tilt is not None]
    return {'bot': bot, 'maxTiltDeg': max((tilt for _, tilt in available), default=None),
            'firstTiltAbove45': next((time for time, tilt in available if tilt > 45), None),
            'firstTiltAbove90': next((time for time, tilt in available if tilt > 90), None),
            'availableSamples': len(available), 'totalSamples': len(samples)}


def _window(frames, start, end):
    complete = frames[0]['time'] <= start+1e-8 and frames[-1]['time'] >= end-1e-8
    first = min(range(len(frames)), key=lambda index: abs(frames[index]['time']-start))
    last = min(range(len(frames)), key=lambda index: abs(frames[index]['time']-end))
    return frames[first:last+1], complete


def _pair_distances(frames):
    distances = [_distance(_position(frame, 0), _position(frame, 1)) for frame in frames]
    return distances if distances and all(value is not None for value in distances) else None


def _distant_intervals(frames, start, end, closed):
    """Sampled spans beyond the chassis proximity threshold, within one free gap."""
    intervals, run = [], []
    available = True
    samples = [frame for frame in frames if start <= frame['time'] <= end and
               not (closed and frame['time'] == end)]
    for frame in samples + [None]:
        distance = _distance(_position(frame, 0), _position(frame, 1)) if frame else None
        if frame and distance is None:
            available = False
        if distance is not None and distance > CLOSE_DISTANCE_M:
            run.append(frame['time'])
        else:
            if run and run[-1]-run[0] >= SEPARATION_SECONDS-1e-8:
                intervals.append({'start': run[0], 'end': run[-1], 'durationSeconds': run[-1]-run[0]})
            run = []
    return intervals, available and bool(samples)


def analyze_replay(replay, bot=None, source_hash=None):
    """Analyze one selected source/physical side without changing the replay.

    Full 1 s pre / 1.5 s post windows are required for motion deltas. Endpoints
    use nearest saved frames and include their timestamps. None denotes missing
    evidence; it must not be treated as zero achieved motion. Source hashes must
    match exactly one side, preventing swapped-side attribution errors.

    wasCloseDuel means contact within 0.2 s, or chassis separation <=0.18 m
    with <=0.02 m pre-alert ground path length. It says nothing about posture.
    newSustainedContactFreeInterval needs recent physical contact and a new
    >=0.5 s collision-proxy gap. newSustainedSeparation additionally needs a
    >=0.5 s sampled distance >0.18 m within that gap. Existing gaps are excluded.
    pairDistanceGrowthM compares the alert frame with the post-window endpoint;
    preTiltDeg/preRootYM likewise describe the sampled alert frame.
    """
    if source_hash is not None:
        matches = [index for index, value in enumerate(replay.get('metadata', {}).get('programHashes', []))
                   if value == source_hash]
        if len(matches) != 1 or (bot is not None and bot != matches[0]):
            raise ValueError('Source hash must identify exactly one matching bot')
        bot = matches[0]
    if bot not in (0, 1):
        raise ValueError('Select --bot 0/1 or one exact --source-hash')
    frames = sorted(replay.get('frames', []), key=lambda frame: frame['time'])
    if not frames:
        raise ValueError('Replay has no saved frames')
    events = sorted(replay.get('events', []), key=lambda event: event['time'])
    contact_intervals, free_intervals = _contact_intervals(events, frames[0]['time'], frames[-1]['time'])
    alerts = []
    for event in events:
        message = event.get('message', '')
        if (event['type'] != 'program_log' or event.get('bot') != bot or
                not ('suspected lockup' in message.lower() or 'watchdog' in message.lower())):
            continue
        time = event['time']
        before, pre_complete = _window(frames, time-PRE_SECONDS, time)
        after, post_complete = _window(frames, time, time+POST_SECONDS)
        displacement = _distance(_position(before[0], bot), _position(before[-1], bot)) if pre_complete else None
        # Net displacement alone can hide dithering; low-travel classification uses path length.
        positions = [_position(frame, bot) for frame in before]
        travel = (sum(_distance(left, right) for left, right in zip(positions, positions[1:]))
                  if pre_complete and all(value is not None for value in positions) else None)
        pair_distance = _distance(_position(after[0], 0), _position(after[0], 1))
        touching = any(start <= time and (time < end or not closed and time == end)
                       for start, end, closed in contact_intervals)
        previous = [min(end, time) for start, end, _ in contact_intervals if start <= time]
        last_contact = max(previous) if previous else None
        lag = time-last_contact if last_contact is not None else None
        recent_contact = lag is not None and lag <= CONTACT_RECENCY_SECONDS+1e-8
        close_duel = (True if recent_contact else
                      (pair_distance <= CLOSE_DISTANCE_M and travel <= LOW_TRAVEL_M)
                      if pair_distance is not None and travel is not None else None)
        prior_gap = next((time-start for start, end, _ in free_intervals if start <= time < end), 0.)
        sustained, short = [], []
        horizon = min(frames[-1]['time'], time+POST_SECONDS)
        for start, end, closed in free_intervals:
            if end <= time or start >= horizon:
                continue
            stop = min(end, horizon)
            interval = {'start': start, 'end': stop, 'durationSeconds': stop-start,
                        'postAlertSeconds': stop-max(start, time), 'beganBeforeAlert': start < time,
                        'endedByContact': closed and end <= horizon}
            if interval['durationSeconds'] >= SEPARATION_SECONDS-1e-8:
                sustained.append(interval)
            elif interval['endedByContact']:
                short.append(interval)
        new_free_interval = recent_contact and any(not interval['beganBeforeAlert'] for interval in sustained)
        distant, distance_available = [], True
        for interval in sustained:
            spans, available = _distant_intervals(frames, max(time, interval['start']), interval['end'],
                                                  interval['endedByContact'])
            distant += [dict(span, beganBeforeAlert=interval['beganBeforeAlert']) for span in spans]
            distance_available = distance_available and available
        new_separation = recent_contact and any(not interval['beganBeforeAlert'] for interval in distant)
        if not new_free_interval and not post_complete:
            new_free_interval = None
        if not new_separation and (not post_complete or new_free_interval and not distance_available):
            new_separation = None
        physical_change, physical_span = _heading_motion(after, bot, False) if post_complete else (None, None)
        sensor_change, sensor_span = _heading_motion(after, bot, True) if post_complete else (None, None)
        before_distances = _pair_distances(before) if pre_complete else None
        after_distances = _pair_distances(after) if post_complete else None
        pair_after = after_distances[-1] if after_distances is not None else None
        pre_position, post_position = _position(after[0], bot), _position(after[-1], bot)
        reason = message.split('range mm', 1)[0].strip()
        if reason.lower().startswith('suspected lockup'):
            reason = reason[len('suspected lockup'):].strip()
        alerts.append({'event': dict(event), 'time': time, 'bot': bot, 'detectionReason': reason,
                       'physicalContactAtAlert': touching, 'lastPhysicalContactTime': last_contact,
                       'contactLagSeconds': lag, 'wasCloseDuel': close_duel,
                       'priorNoContactSeconds': prior_gap,
                       'preGroundDisplacementM': displacement, 'preGroundTravelM': travel,
                       'pairDistanceAtAlertM': pair_distance,
                       'minPairDistanceBeforeM': min(before_distances) if before_distances is not None else None,
                       'minPairDistanceAfterM': min(after_distances) if after_distances is not None else None,
                       'maxPairDistanceAfterM': max(after_distances) if after_distances is not None else None,
                       'pairDistanceAfterM': pair_after,
                       'pairDistanceGrowthM': pair_after-pair_distance if pair_after is not None and pair_distance is not None else None,
                       'physicalHeadingChangeDegrees': physical_change, 'physicalHeadingSpanDegrees': physical_span,
                       'sensorHeadingChangeDegrees': sensor_change, 'sensorHeadingSpanDegrees': sensor_span,
                       'preTiltDeg': _tilt(after[0], bot),
                       'postTiltDeg': _tilt(after[-1], bot) if post_complete else None,
                       'preRootYM': pre_position[1] if pre_position is not None else None,
                       'postRootYM': post_position[1] if post_complete and post_position is not None else None,
                       'preWindowComplete': pre_complete, 'postWindowComplete': post_complete,
                       'preSampleTimes': [before[0]['time'], before[-1]['time']],
                       'postSampleTimes': [after[0]['time'], after[-1]['time']],
                       'sustainedContactFreeIntervals': sustained, 'shortContactFreeGaps': short,
                       'sustainedDistantContactFreeIntervals': distant,
                       'newSustainedContactFreeInterval': new_free_interval,
                       'newSustainedSeparation': new_separation})
    return {'bot': bot, 'sourceHash': replay.get('metadata', {}).get('programHashes', [None, None])[bot],
            'seed': replay.get('metadata', {}).get('seed'), 'alerts': alerts,
            'posture': [_posture(frames, side) for side in (0, 1)],
            'thresholds': {'preSeconds': PRE_SECONDS, 'postSeconds': POST_SECONDS,
                           'recentContactSeconds': CONTACT_RECENCY_SECONDS,
                           'sustainedContactFreeSeconds': SEPARATION_SECONDS,
                           'closeChassisDistanceM': CLOSE_DISTANCE_M, 'lowPreGroundTravelM': LOW_TRAVEL_M},
            'limits': 'Offline uncalibrated simulator observations; contact-proxy gaps and proximity '
                      'do not prove a physical escape, causation, upright posture, or hardware lockup. '
                      'Tilt crossings and maxima are sampled replay observations.'}


def aggregate_reports(reports):
    alerts = [alert for report in reports for alert in report['alerts']]
    return {'bouts': len(reports), 'alerts': len(alerts),
            'closeDuelAlerts': sum(alert['wasCloseDuel'] is True for alert in alerts),
            'notCloseDuelAlerts': sum(alert['wasCloseDuel'] is False for alert in alerts),
            'unknownCloseDuelAlerts': sum(alert['wasCloseDuel'] is None for alert in alerts),
            'priorSeparationAlerts': sum(alert['priorNoContactSeconds'] >= SEPARATION_SECONDS for alert in alerts),
            'newSustainedSeparations': sum(alert['newSustainedSeparation'] is True for alert in alerts),
            'newSustainedContactFreeIntervals': sum(alert['newSustainedContactFreeInterval'] is True for alert in alerts),
            'jitterOnlyAlerts': sum(bool(alert['shortContactFreeGaps']) and
                                   not alert['sustainedContactFreeIntervals'] for alert in alerts),
            'incompletePostWindows': sum(not alert['postWindowComplete'] for alert in alerts)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bouts', nargs='+', type=Path)
    selector = parser.add_mutually_exclusive_group(required=True)
    selector.add_argument('--bot', type=int, choices=(0, 1), help='Physical replay side, 0 blue or 1 red')
    selector.add_argument('--source-hash', help='Exact metadata program hash; follows the source across side swaps')
    parser.add_argument('--aggregate', action='store_true', help='Include descriptive pooled alert counters')
    args = parser.parse_args()
    reports = []
    for path in args.bouts:
        try:
            replay = json.loads(path.read_text(encoding='utf-8'))
            report = analyze_replay(replay, bot=args.bot, source_hash=args.source_hash)
        except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
            parser.error(f'{path}: {error}')
        reports.append({'replay': str(path), **report})
    output = {'bouts': reports}
    if args.aggregate:
        output['aggregate'] = aggregate_reports(reports)
    print(json.dumps(output, separators=(',', ':'), allow_nan=False))


if __name__ == '__main__':
    main()
