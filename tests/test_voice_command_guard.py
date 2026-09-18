import ast
import pathlib
import types
import unittest


MAIN = (pathlib.Path(__file__).resolve().parents[1] /
        'jetson/llm/main.py')


def load_guard():
    tree = ast.parse(MAIN.read_text(encoding='utf-8'))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and
        node.name == 'transcription_session_is_open')
    env = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 '<voice-command-guard>', 'exec'), env)
    return env['transcription_session_is_open']


class VoiceCommandGuardTests(unittest.TestCase):
    def test_accepts_command_only_during_open_wake_session(self):
        guard = load_guard()
        self.assertTrue(guard(False, types.SimpleNamespace(is_awake=True)))

    def test_rejects_late_result_after_wake_session_closed(self):
        guard = load_guard()
        self.assertFalse(guard(False, types.SimpleNamespace(is_awake=False)))

    def test_rejects_result_while_robot_is_speaking(self):
        guard = load_guard()
        self.assertFalse(guard(True, types.SimpleNamespace(is_awake=True)))


if __name__ == '__main__':
    unittest.main()
