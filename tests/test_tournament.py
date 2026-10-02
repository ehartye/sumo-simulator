from tools.strategy_tournament import score_bouts, run_bout
import pytest


def test_challenger_scoring_counts_draws_and_both_sides():
    bouts = [{'winningProgram': winner, 'challengerFault': False} for winner in [0, 0, None, 1]]
    assert score_bouts(bouts) == {'wins': 2, 'draws': 1, 'losses': 1, 'score': .625, 'faults': 0, 'promote': False}
    bouts = [{'winningProgram': winner, 'challengerFault': False} for winner in [0, 0, 0, 0, None, 1]]
    assert score_bouts(bouts)['promote']
    bouts[0]['challengerFault'] = True
    assert not score_bouts(bouts)['promote']


def test_opponent_fault_wins_cannot_promote_a_challenger():
    bouts = [{'winningProgram': 0, 'challengerFault': False, 'programErrors': [None, None]}
             for _ in range(6)]
    assert score_bouts(bouts)['promote']
    bouts[0]['programErrors'][1] = 'Opponent program failed'
    score = score_bouts(bouts)
    assert score['wins'] == 6 and score['faults'] == 1 and not score['promote']


def test_recorded_rule_fault_result_blocks_promotion():
    bouts = [{'winningProgram': 0, 'challengerFault': False,
              'result': {'winner': 0, 'reason': 'rule_violation'}} for _ in range(6)]
    score = score_bouts(bouts)
    assert score['faults'] == 6 and not score['promote']


@pytest.mark.parametrize('runtime_profile', ['sync-subset-v3', 'sync-subset-v4'])
def test_recorded_trial_maps_swapped_winner_and_preserves_visual_replay(tmp_path, runtime_profile):
    import json
    trial = run_bout({
        'programs': ['raise ValueError("test fault")', 'pass'], 'names': ['challenger', 'incumbent'],
        'seed': 11, 'swapped': True, 'timeout': 1, 'friction': .8,
        'path': str(tmp_path/'bout.json'),
        'runtime_profile': runtime_profile,
    })
    assert trial['winningProgram'] == 1 and trial['challengerFault']
    replay = json.loads((tmp_path/'bout.json').read_text())
    assert replay['metadata']['names'] == ['incumbent', 'challenger']
    assert replay['metadata']['runtime'] == runtime_profile
    assert replay['metadata']['programs'][1].startswith('raise')
    assert replay['frames'][-1]['result']['winner'] == 0
    assert replay['events'][-1]['type'] == 'match_ended'
    assert len(replay['metadata']['robotHash']) == 64
