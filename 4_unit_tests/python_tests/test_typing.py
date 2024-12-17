import subprocess

from unittest import main, TestCase


class TypingTest(TestCase):
    def test_typing(self):
        result = subprocess.run(
            ["mypy", "./vehicle_model/suspension_model", "--show-traceback"],
            capture_output=True,
            text=True
        )
        
        self.assertEqual(result.returncode, 0, result.stdout)