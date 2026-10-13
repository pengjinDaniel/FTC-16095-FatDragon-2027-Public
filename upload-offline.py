#!/usr/bin/env python3
"""User-run offline ADB installation. Never starts a robot OpMode."""
import argparse
import hashlib
import ipaddress
import os
import pathlib
import pty
import re
import select
import shlex
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent
ADB = pathlib.Path.home() / 'Developer/Android-FRC/android-sdk/platform-tools/adb'
APK = ROOT / 'TeamCode/build/outputs/apk/debug/TeamCode-debug.apk'
PACKAGE = 'com.qualcomm.ftcrobotcontroller'
STAGE = '本地文件检查'


def stage(message):
    global STAGE
    STAGE = message
    print('\n' + message, flush=True)


def run(*args, timeout=30):
    result = subprocess.run([str(ADB), *args], text=True, capture_output=True,
                            timeout=timeout, check=True)
    return result.stdout.strip()


def live_run(*args, timeout, label):
    """A local PTY preserves adb push's progress display; always reap the client."""
    master, slave = pty.openpty()
    process = None
    chunks = []
    started = time.monotonic()
    next_notice = started + 10
    try:
        process = subprocess.Popen([str(ADB), *args], stdin=subprocess.DEVNULL,
                                   stdout=slave, stderr=slave)
        os.close(slave)
        slave = None
        while True:
            now = time.monotonic()
            if now - started >= timeout:
                raise subprocess.TimeoutExpired(process.args, timeout,
                                                output=''.join(chunks))
            if now >= next_notice:
                print(f'\n[{label}] 已用时 {int(now - started)} 秒，仍在等待完成。', flush=True)
                next_notice = now + 10
            readable, _, _ = select.select([master], [], [], 0.25)
            if readable:
                try:
                    data = os.read(master, 8192)
                except OSError as exc:
                    # macOS/Linux report PTY EOF differently.
                    if exc.errno != 5:
                        raise
                    data = b''
                if not data:
                    if process.poll() is not None:
                        break
                    time.sleep(0.05)
                    continue
                output = data.decode('utf-8', errors='replace')
                chunks.append(output)
                print(output, end='', flush=True)
            elif process.poll() is not None:
                break
        result = ''.join(chunks).strip()
        if process.returncode:
            raise subprocess.CalledProcessError(process.returncode, process.args,
                                                output=result, stderr=result)
        return result
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        os.close(master)
        if slave is not None:
            os.close(slave)


def device_states():
    return {parts[0]: parts[1] for line in run('devices', timeout=5).splitlines()[1:]
            if len(parts := line.split()) >= 2}


def wait_ready(serial, seconds=20):
    deadline = time.monotonic() + seconds
    previous = None
    while time.monotonic() < deadline:
        try:
            state = device_states().get(serial, '未连接')
            if state != previous:
                print('设备状态：', state, flush=True)
                previous = state
            if state == 'unauthorized':
                raise ValueError('设备未授权；请在机器人端允许 USB 调试。没有开始上传。')
            if state == 'device':
                if run('-s', serial, 'shell', 'echo', 'FTC_LINK_OK', timeout=5) == 'FTC_LINK_OK':
                    return True
        except subprocess.SubprocessError:
            pass
        time.sleep(1)
    return False


def connect_wireless(serial):
    stage('1/4：连接机器人')
    if device_states().get(serial) == 'device' and wait_ready(serial, seconds=3):
        return
    for attempt in range(2):
        if attempt:
            # Do not interrupt unrelated phones, emulators or Android Studio targets.
            others = set(device_states()) - {serial}
            if not others:
                print('连接仍无响应，重启电脑端 ADB 服务后重试（不重启机器人）。', flush=True)
                run('kill-server', timeout=8)
                run('start-server', timeout=10)
            else:
                print('另有设备连接，只重连本次目标：', serial, flush=True)
        print(f'连接尝试 {attempt + 1}/2：{serial}', flush=True)
        try:
            print(run('disconnect', serial, timeout=5), flush=True)
        except subprocess.SubprocessError as exc:
            detail = getattr(exc, 'stderr', None) or str(exc)
            if 'no such device' in detail:
                print('没有旧连接需要清理，直接建立新连接。', flush=True)
            else:
                print('旧连接清理未完成，仍继续尝试连接：', detail, flush=True)
        time.sleep(1)
        # Cleanup is best-effort: a missing/stale transport must not skip connect.
        try:
            print(run('connect', serial, timeout=12), flush=True)
        except subprocess.SubprocessError as exc:
            print('连接命令未完成：', getattr(exc, 'stderr', None) or str(exc), flush=True)
        if wait_ready(serial):
            return
    raise ValueError('机器人仍未响应，尚未开始上传。确认电脑连的是机器人 Wi-Fi、IP 正确；'
                     '停止机器人后可重启 Control Hub 再试，或用 USB 选择方式 2。')


def remote_sha256(serial, path):
    # Older Android builds may not expose this tool. Absence is not a hash match.
    for command in ('sha256sum', 'toybox sha256sum'):
        try:
            output = run('-s', serial, 'shell', command + ' ' + shlex.quote(path), timeout=15)
            match = re.match(r'^([0-9a-fA-F]{64})\s', output)
            if match:
                return match.group(1).lower()
        except subprocess.SubprocessError:
            pass
    return None


def installed_matches(serial, digest):
    try:
        output = run('-s', serial, 'shell', 'pm', 'path', PACKAGE, timeout=10)
        paths = [line[len('package:'):] for line in output.splitlines()
                 if line.startswith('package:')]
        if len(paths) != 1:
            return None
        installed_digest = remote_sha256(serial, paths[0])
        return None if installed_digest is None else installed_digest == digest
    except subprocess.SubprocessError:
        return None


def install_apk(serial, digest):
    stage('2/4：检查并传输 APK（显示百分比，最多等待 10 分钟）')
    print('先核对上次是否已经装好同一份 APK……', flush=True)
    if installed_matches(serial, digest) is True:
        print('机器人已经安装了这份 APK，文件 SHA256 一致，无需重复上传。')
        return
    remote = f'/data/local/tmp/ftc-{digest}.apk'
    if remote_sha256(serial, remote) == digest:
        print('机器人上已有完整安装文件，跳过传输。', flush=True)
    else:
        try:
            live_run('-s', serial, 'push', str(APK), remote, timeout=600, label='传输文件')
        except subprocess.SubprocessError as exc:
            raise ValueError('APK 传输未完成，未执行安装。请重试；若无线持续卡住，可改用 USB。') from exc
        staged_digest = remote_sha256(serial, remote)
        if staged_digest is not None:
            if staged_digest != digest:
                raise ValueError('机器人收到的 APK 校验不一致，已停止，未执行安装。')
        else:
            size = run('-s', serial, 'shell', 'wc -c < ' + shlex.quote(remote), timeout=10)
            if not size.strip().isdigit() or int(size) != APK.stat().st_size:
                raise ValueError('机器人收到的 APK 大小不符，已停止，未执行安装。')
            print('设备不支持 SHA256 校验；传输成功，文件大小核对通过。', flush=True)
    stage('3/4：在机器人上安装（保留应用数据，最多等待 5 分钟）')
    try:
        output = live_run('-s', serial, 'shell', 'pm', 'install', '-r', remote,
                          timeout=300, label='安装应用')
    except subprocess.SubprocessError as exc:
        print('\n安装连接中断或超时，正在核实实际安装结果……', flush=True)
        if installed_matches(serial, digest) is not True:
            detail = getattr(exc, 'stderr', None) or getattr(exc, 'output', None) or str(exc)
            raise ValueError('未能确认安装成功；机器人端可能仍在安装。稍等后重新运行本工具，'
                             '会先检查已装版本。详情：' + str(detail)) from exc
        print('已确认安装完成：机器人已装 APK 的 SHA256 与本机一致。', flush=True)
    else:
        if 'Success' not in [line.strip() for line in output.splitlines()]:
            raise ValueError('机器人没有报告安装成功：' + output)
        stage('4/4：核对安装结果')
        installed = run('-s', serial, 'shell', 'pm', 'path', PACKAGE, timeout=15)
        if not installed.startswith('package:'):
            raise ValueError('安装命令已报告 Success，但未查到 Robot Controller App，请重新运行核对。')
        matched = installed_matches(serial, digest)
        if matched is False:
            raise ValueError('安装后 APK 校验仍不一致，不能确认更新成功。请重新运行核对。')
        if matched is None:
            print('安装器已报告 Success；设备不支持或未完成已装 APK 的额外校验。', flush=True)
    try:
        # Remove only this tool's staging copy after confirmed success.
        run('-s', serial, 'shell', 'rm', '-f', remote, timeout=5)
    except subprocess.SubprocessError:
        print('安装已完成；临时安装文件暂未清理，不影响使用。', flush=True)


def ready_devices():
    listing = run('devices', '-l')
    print(listing)
    return [line.split()[0] for line in listing.splitlines()[1:]
            if len(line.split()) >= 2 and line.split()[1] == 'device']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check local files only; do not connect or install')
    args = parser.parse_args()
    if not ADB.is_file() or not APK.is_file():
        raise ValueError('缺少 ADB 或 APK；请先在联网时完成环境配置和编译。')
    print('上传工具 v2.1：自动重连 / 分步传输与安装')
    print('当前工程：', ROOT.name)
    print('APK：', APK)
    digest = hashlib.sha256(APK.read_bytes()).hexdigest()
    print('SHA256：', digest)
    if args.check:
        print('本地文件检查通过；未连接、未上传。')
        return
    print('\n请先停止机器人程序。此次更新 Robot Controller App，保留应用数据。')
    print('1：无线连接 Control Hub（电脑先连机器人 Wi-Fi）')
    print('2：USB 连接 Robot Controller / Control Hub')
    mode = input('连接方式 [1]：').strip() or '1'
    if mode == '1':
        address = input('Control Hub IP [192.168.43.1]，也可填 IP:端口：').strip() or '192.168.43.1'
        host, sep, port = address.partition(':')
        ipaddress.IPv4Address(host)
        port = int(port) if sep else 5555
        if not 1 <= port <= 65535:
            raise ValueError('端口必须在 1–65535 之间。')
        serial = f'{host}:{port}'
        connect_wireless(serial)
    elif mode == '2':
        stage('1/4：选择 USB 设备')
        devices = ready_devices()
        if not devices:
            raise ValueError('没有已授权设备。检查 USB 数据线，并在设备上允许 USB 调试。')
        if len(devices) == 1:
            serial = devices[0]
        else:
            serial = input('输入目标 Robot Controller / Control Hub 的完整设备编号：').strip()
            if serial not in devices:
                raise ValueError('设备编号不在已授权列表中。')
        if not wait_ready(serial, seconds=8):
            raise ValueError('所选设备没有响应，未开始上传。请检查数据线和 USB 调试授权。')
    else:
        raise ValueError('请选择 1 或 2。')
    model = run('-s', serial, 'shell', 'getprop', 'ro.product.model')
    print(f'\n目标设备：{serial}（{model}）')
    print('确认这是机器人端 Robot Controller / Control Hub，不能是驾驶端 Driver Station。')
    if input('输入 y 上传，其他输入取消：').strip().lower() != 'y':
        print('已取消，未上传。')
        return
    install_apk(serial, digest)
    print('\n上传完成。回到控制手机查看程序列表。没有启动任何机器人程序。')
    print('程序入口：Blue / Red。松开 R2：校准成功后等 150ms，失败持续尝试；按住 R2 暂停校准。')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, EOFError, subprocess.SubprocessError) as exc:
        detail = getattr(exc, 'stderr', None) or str(exc)
        print(f'\n未完成（{STAGE}）：{detail}', file=sys.stderr)
        if 'INSTALL_FAILED_UPDATE_INCOMPATIBLE' in detail or 'signatures do not match' in detail:
            print('安装器报告签名不匹配；请保留配置再处理，本工具不会卸载应用或清除数据。', file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print(f'\n已中止（{STAGE}）。若在安装阶段，机器人端可能仍在安装；下次运行会先核对版本。', file=sys.stderr)
        sys.exit(130)
