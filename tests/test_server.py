from fastapi.testclient import TestClient

from sumo.server import app


def test_promoted_strategy_selection_and_scoped_replay_access():
    with TestClient(app) as client:
        selection = client.get('/api/strategy-selection')
        assert selection.status_code == 200
        data = selection.json()
        assert len(data['revision']) == 64
        assert data['loop'] >= 1
        from sumo.robot import ROOT
        import json
        recorded = json.loads((ROOT/'experiments/champion.json').read_text())
        assert data['testedRuntime']==recorded['testedRuntime']
        assert data['currentRuntime']=='sync-subset-v4'
        examples = client.get('/api/info').json()['examples']
        for role in ('champion', 'opponent'):
            assert data[role]['source'] == examples[data[role]['name']]
        assert data['replayUrl'].startswith('/trials/')
        assert client.get('/trials/missing.json').status_code == 404


def test_http_match_replay_validation_and_program_failure():
    with TestClient(app) as client:
        info = client.get('/api/info').json()
        assert len(info['examples']) >= 2
        assert info['robot']['totalMassKg'] > .3
        assert client.get('/').status_code == 200
        assert client.get('/assets/model.glb').content[:4] == b'glTF'
        assert client.post('/api/reset', json={'programs': ['pass'], 'seed': 0}).status_code == 422
        reset = client.post('/api/reset', json={'programs': ['raise ValueError("bad program")', 'from pybricks.tools import wait\nwait(1000)'], 'seed': 13})
        assert reset.status_code == 200
        result = client.post('/api/step', json={'seconds': .02}).json()
        assert result['result']['reason'] == 'program_error'
        assert result['result']['winner'] == 1
        replay = client.get('/api/replay').json()
        assert replay['metadata']['seed'] == 13
        assert len(replay['metadata']['robotHash']) == 64
        assert replay['frames'][-1]['result'] == result['result']
        assert result['events'][-1]['type'] == 'match_ended'
        assert replay['events'] == client.get('/api/events').json()['events']
        assert replay['events'][-1]['result'] == result['result']
        from pathlib import Path
        import json
        assert client.post('/api/step', json={'seconds': .02}).json()['events'] == []
    # File persistence is eventual; shutdown drains queued records.
    saved = [json.loads(line) for line in Path(replay['metadata']['eventLogFile']).read_text().splitlines()]
    assert saved == replay['events']
