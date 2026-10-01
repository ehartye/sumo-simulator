from fastapi.testclient import TestClient

from sumo.server import app


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
