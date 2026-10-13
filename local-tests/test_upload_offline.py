"""Offline regression checks. No real adb server or robot is used."""
import contextlib
import importlib.util
import io
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('upload', pathlib.Path(__file__).resolve().parents[1] / 'upload-offline.py')
upload = importlib.util.module_from_spec(spec)
spec.loader.exec_module(upload)
SERIAL = '192.168.43.1:5555'
DIGEST = 'a' * 64


class UploadChecks(unittest.TestCase):
    def setUp(self):
        self.output = io.StringIO()
        redirect = contextlib.redirect_stdout(self.output)
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)

    def test_offline_already_connected_requires_real_response_and_server_recovery(self):
        with patch.object(upload, 'device_states', return_value={SERIAL: 'offline'}), \
                patch.object(upload, 'wait_ready', side_effect=[False, True]), \
                patch.object(upload, 'run', return_value='already connected') as run, \
                patch.object(upload.time, 'sleep'):
            upload.connect_wireless(SERIAL)
        self.assertIn(('kill-server',), [c.args for c in run.call_args_list])
        self.assertEqual(2, sum(c.args == ('connect', SERIAL) for c in run.call_args_list))
        self.assertFalse(any('install' in c.args for c in run.call_args_list))

    def test_unresponsive_connection_fails_without_upload(self):
        with patch.object(upload, 'device_states', return_value={SERIAL: 'offline'}), \
                patch.object(upload, 'wait_ready', return_value=False), \
                patch.object(upload, 'run', return_value='already connected'), \
                patch.object(upload.time, 'sleep'), \
                self.assertRaisesRegex(ValueError, '尚未开始上传'):
            upload.connect_wireless(SERIAL)

    def test_cleanup_failure_never_skips_connect_including_after_server_reset(self):
        failures = [
            subprocess.CalledProcessError(1, ['adb', 'disconnect', SERIAL],
                                         stderr="error: no such device '" + SERIAL + "'"),
            subprocess.CalledProcessError(1, ['adb', 'disconnect', SERIAL],
                                         stderr='error: transport error'),
            subprocess.TimeoutExpired(['adb', 'disconnect', SERIAL], 5),
        ]
        for failure in failures:
            with self.subTest(failure=str(failure)):
                def fake_run(*args, **kwargs):
                    if args[0] == 'disconnect':
                        raise failure
                    return 'connected' if args[0] == 'connect' else ''
                with patch.object(upload, 'device_states', return_value={}), \
                        patch.object(upload, 'wait_ready', side_effect=[False, True]), \
                        patch.object(upload, 'run', side_effect=fake_run) as run, \
                        patch.object(upload.time, 'sleep'):
                    upload.connect_wireless(SERIAL)
                commands = [call.args for call in run.call_args_list]
                self.assertEqual(commands, [
                    ('disconnect', SERIAL), ('connect', SERIAL),
                    ('kill-server',), ('start-server',),
                    ('disconnect', SERIAL), ('connect', SERIAL),
                ])

    def test_fresh_connection_succeeds_despite_no_old_device(self):
        def fake_run(*args, **kwargs):
            if args[0] == 'disconnect':
                raise subprocess.CalledProcessError(1, args, stderr='error: no such device')
            return 'connected to ' + SERIAL
        with patch.object(upload, 'device_states', return_value={}), \
                patch.object(upload, 'wait_ready', return_value=True), \
                patch.object(upload, 'run', side_effect=fake_run) as run, \
                patch.object(upload.time, 'sleep'):
            upload.connect_wireless(SERIAL)
        self.assertEqual([('disconnect', SERIAL), ('connect', SERIAL)],
                         [call.args for call in run.call_args_list])

    def test_does_not_restart_server_used_by_other_devices(self):
        with patch.object(upload, 'device_states', return_value={SERIAL: 'offline', 'other-usb': 'device'}), \
                patch.object(upload, 'wait_ready', side_effect=[False, True]), \
                patch.object(upload, 'run', return_value='ok') as run, \
                patch.object(upload.time, 'sleep'):
            upload.connect_wireless(SERIAL)
        self.assertNotIn(('kill-server',), [c.args for c in run.call_args_list])
        self.assertTrue(all(c.args[-1] == SERIAL for c in run.call_args_list))

    def test_ready_connection_is_left_alone(self):
        with patch.object(upload, 'device_states', return_value={SERIAL: 'device'}), \
                patch.object(upload, 'wait_ready', return_value=True), \
                patch.object(upload, 'run') as run:
            upload.connect_wireless(SERIAL)
        run.assert_not_called()

    def test_device_state_requires_working_shell(self):
        with patch.object(upload, 'device_states', return_value={SERIAL: 'device'}), \
                patch.object(upload, 'run', return_value='wrong response'), \
                patch.object(upload.time, 'sleep'), \
                patch.object(upload.time, 'monotonic', side_effect=[0, 0, 2]):
            self.assertFalse(upload.wait_ready(SERIAL, seconds=1))

    def test_unauthorized_does_not_silently_continue(self):
        with patch.object(upload, 'device_states', return_value={SERIAL: 'unauthorized'}), \
                self.assertRaisesRegex(ValueError, '未授权'):
            upload.wait_ready(SERIAL)

    def test_identical_installed_apk_is_not_reinstalled(self):
        with patch.object(upload, 'installed_matches', return_value=True), \
                patch.object(upload, 'live_run') as live:
            upload.install_apk(SERIAL, DIGEST)
        live.assert_not_called()

    def test_transfer_timeout_never_runs_install(self):
        with patch.object(upload, 'installed_matches', return_value=False), \
                patch.object(upload, 'remote_sha256', return_value=None), \
                patch.object(upload, 'live_run', side_effect=subprocess.TimeoutExpired('push', 600)) as live, \
                self.assertRaisesRegex(ValueError, '未执行安装'):
            upload.install_apk(SERIAL, DIGEST)
        self.assertEqual(1, live.call_count)
        self.assertEqual('push', live.call_args.args[2])

    def test_wrong_staged_checksum_blocks_install(self):
        with patch.object(upload, 'installed_matches', return_value=False), \
                patch.object(upload, 'remote_sha256', side_effect=[None, 'b' * 64]), \
                patch.object(upload, 'live_run', return_value='1 file pushed') as live, \
                self.assertRaisesRegex(ValueError, '校验不一致'):
            upload.install_apk(SERIAL, DIGEST)
        self.assertEqual(1, live.call_count)

    def test_install_timeout_only_counts_success_if_exact_apk_matches(self):
        for installed in (True, False, None):
            with self.subTest(installed=installed), \
                    patch.object(upload, 'installed_matches', side_effect=[False, installed]), \
                    patch.object(upload, 'remote_sha256', return_value=DIGEST), \
                    patch.object(upload, 'live_run', side_effect=subprocess.TimeoutExpired('pm install', 300)), \
                    patch.object(upload, 'run') as run:
                if installed is True:
                    upload.install_apk(SERIAL, DIGEST)
                    self.assertEqual('rm', run.call_args.args[3])
                else:
                    with self.assertRaisesRegex(ValueError, '未能确认安装成功'):
                        upload.install_apk(SERIAL, DIGEST)
                    run.assert_not_called()

    def test_pm_failure_never_becomes_success(self):
        with patch.object(upload, 'installed_matches', return_value=False), \
                patch.object(upload, 'remote_sha256', return_value=DIGEST), \
                patch.object(upload, 'live_run', return_value='Failure [INSTALL_FAILED_UPDATE_INCOMPATIBLE]'), \
                self.assertRaisesRegex(ValueError, 'INSTALL_FAILED_UPDATE_INCOMPATIBLE'):
            upload.install_apk(SERIAL, DIGEST)

    def test_push_then_install_success_and_scoped_cleanup(self):
        with patch.object(upload, 'installed_matches', side_effect=[False, True]), \
                patch.object(upload, 'remote_sha256', side_effect=[None, DIGEST]), \
                patch.object(upload, 'live_run', side_effect=['1 file pushed', 'Success\r\n']) as live, \
                patch.object(upload, 'run', return_value='package:/data/app/rc/base.apk') as run:
            upload.install_apk(SERIAL, DIGEST)
        self.assertEqual('push', live.call_args_list[0].args[2])
        self.assertEqual(('shell', 'pm', 'install', '-r'), live.call_args_list[1].args[2:6])
        self.assertTrue(all(c.args[:2] == ('-s', SERIAL) for c in live.call_args_list + run.call_args_list))
        self.assertEqual(f'/data/local/tmp/ftc-{DIGEST}.apk', run.call_args.args[-1])

    def test_package_presence_alone_does_not_confirm_identity(self):
        with patch.object(upload, 'run', return_value='package:/data/app/rc/base.apk'), \
                patch.object(upload, 'remote_sha256', return_value=None):
            self.assertIsNone(upload.installed_matches(SERIAL, DIGEST))

    def test_check_mode_never_calls_adb(self):
        with patch.object(upload, 'run', side_effect=AssertionError('must not run adb')), \
                patch.object(upload, 'live_run', side_effect=AssertionError('must not run adb')), \
                patch.object(sys, 'argv', ['upload-offline.py', '--check']):
            upload.main()
        self.assertIn('未连接、未上传', self.output.getvalue())

    def test_live_output_exit_status_and_timeout_use_fake_process_only(self):
        with tempfile.TemporaryDirectory() as folder:
            fake = pathlib.Path(folder) / 'fake-adb'
            fake.write_text('#!' + sys.executable + '\n'
                            'import sys, time\n'
                            'if sys.argv[1] == "slow": time.sleep(10)\n'
                            'print("[100%] Fake progress", flush=True)\n'
                            'if sys.argv[1] == "fail": sys.exit(2)\n')
            fake.chmod(0o755)
            with patch.object(upload, 'ADB', fake):
                self.assertIn('Fake progress', upload.live_run('ok', timeout=2, label='fake'))
                with self.assertRaises(subprocess.CalledProcessError):
                    upload.live_run('fail', timeout=2, label='fake')
                with self.assertRaises(subprocess.TimeoutExpired):
                    upload.live_run('slow', timeout=0.15, label='fake')


if __name__ == '__main__':
    unittest.main(verbosity=2)
