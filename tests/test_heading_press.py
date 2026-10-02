import ast
from pathlib import Path


def test_heading_press_corrects_error_and_rate_without_exceeding_motor_power():
    path = Path(__file__).resolve().parents[1]/'examples/heading_press.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    namespace = {}
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[]), str(path), 'exec'), namespace)
    duty = namespace['press_duties']
    assert duty(180, 180, 0) == (90, 90)
    a, b = duty(210, 180, 0)
    assert 0 <= a < b <= 100
    a, b = duty(150, 180, 0)
    assert 0 <= b < a <= 100
    assert duty(370, 10, 0) == (90, 90)
    a, b = duty(180, 180, 500)
    assert a < b
    assert duty(270, 180, 1000) == (30, 100)
