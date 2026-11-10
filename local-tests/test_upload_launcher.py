"""Exercise the macOS launcher with isolated fake build/upload commands; no robot."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class LauncherChecks(unittest.TestCase):
    def run_launcher(self, args=(), build_exit=0, upload_exit=0):
        with tempfile.TemporaryDirectory(prefix='ftc-launcher-') as directory:
            root = Path(directory)
            shutil.copy2(ROOT / '无线传程序.command', root / 'launch.command')
            (root / 'gradlew').write_text('#!/bin/zsh\nprint -r -- "build:$*" >> "$TRACE"\nexit "$BUILD_EXIT"\n')
            (root / 'gradlew').chmod(0o755)
            (root / 'upload-offline.py').write_text(
                'import os,sys\nfrom pathlib import Path\n'
                'with Path(os.environ["TRACE"]).open("a") as f: f.write("upload:"+" ".join(sys.argv[1:])+"\\n")\n'
                'sys.exit(int(os.environ["UPLOAD_EXIT"]))\n')
            jdk = root / 'jdk'
            (jdk / 'bin').mkdir(parents=True)
            (jdk / 'bin/java').write_text('#!/bin/sh\nexit 0\n')
            (jdk / 'bin/java').chmod(0o755)
            trace = root / 'trace'
            env = dict(os.environ, JAVA_HOME=str(jdk), GRADLE_USER_HOME=str(root / 'cache'),
                       TRACE=str(trace), BUILD_EXIT=str(build_exit), UPLOAD_EXIT=str(upload_exit))
            result = subprocess.run(['/bin/zsh', str(root / 'launch.command'), *args],
                                    input='', text=True, capture_output=True, env=env, timeout=10)
            return result.returncode, trace.read_text().splitlines() if trace.exists() else []

    def test_build_precedes_upload(self):
        code, events = self.run_launcher()
        self.assertEqual(code, 0)
        self.assertEqual(events, ['build:clean :TeamCode:assembleDebug --offline --console=plain', 'upload:'])

    def test_failed_build_never_uploads(self):
        code, events = self.run_launcher(build_exit=19)
        self.assertEqual(code, 19)
        self.assertEqual(len(events), 1)
        self.assertTrue(events[0].startswith('build:'))

    def test_build_only_never_connects(self):
        code, events = self.run_launcher(['--build-only'])
        self.assertEqual(code, 0)
        self.assertEqual(events[-1], 'upload:--check')

    def test_check_skips_build(self):
        self.assertEqual(self.run_launcher(['--check']), (0, ['upload:--check']))

    def test_upload_failure_preserved(self):
        self.assertEqual(self.run_launcher(upload_exit=7)[0], 7)

    def test_unknown_args_do_not_build_or_upload(self):
        self.assertEqual(self.run_launcher(['--bad']), (2, []))
        self.assertEqual(self.run_launcher(['--check', '--bad']), (2, []))

if __name__ == '__main__':
    unittest.main()
