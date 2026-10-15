#!/usr/bin/env python3
"""Real local AF_UNIX/HMAC/SQLite subprocess trial, absolutely no PX4 output."""
import hashlib
import hmac
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def exercise(executable: str) -> None:
    with tempfile.TemporaryDirectory(prefix='dyx_preflight_') as temp:
        root = Path(temp)
        os.chmod(root, 0o700)
        secret = os.urandom(32)
        keyfile = root / 'secret.key'
        keyfile.write_bytes(secret)
        os.chmod(keyfile, 0o600)
        db = root / 'replay.sqlite'
        sock = root / 'probe.sock'
        env = dict(os.environ, DYX_HMAC_KEY_FILE=str(keyfile), DYX_REPLAY_DB=str(db))
        session = 'operator_01'
        nonce = 'abcdef0123456789'

        def request(seq: int, good: bool = True) -> bytes:
            content = f'GET\n/health\n{session}\n{seq}\n{nonce}'.encode()
            signature = hmac.new(secret, content, hashlib.sha256).hexdigest()
            if not good:
                signature = '0' * 64
            return (f'GET /health HTTP/1.1\r\nHost: localhost\r\n'
                    f'X-DYX-Session: {session}\r\nX-DYX-Sequence: {seq}\r\n'
                    f'X-DYX-Nonce: {nonce}\r\n'
                    f'Authorization: DYX-HMAC {signature}\r\n\r\n').encode()

        def boot() -> subprocess.Popen:
            proc = subprocess.Popen([executable, '--serve-unix', str(sock)],
                env=env, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE)
            for _ in range(200):
                if sock.exists():
                    return proc
                if proc.poll() is not None:
                    raise AssertionError(f'server died: {proc.stderr.read().decode()}')
                time.sleep(0.02)
            raise AssertionError('socket did not appear')

        def send(payload: bytes) -> bytes:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as cli:
                cli.settimeout(2)
                cli.connect(str(sock))
                cli.sendall(payload)
                chunks = []
                while True:
                    data = cli.recv(4096)
                    if not data:
                        break
                    chunks.append(data)
            return b''.join(chunks)

        def stop(proc: subprocess.Popen) -> None:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                raise
            assert proc.returncode == 0
            assert not sock.exists()

        p = boot()
        try:
            first = send(request(1))
            assert first.startswith(b'HTTP/1.1 200 OK')
            assert b'"motion_enabled":false' in first
            assert b'"physical_stop_confirmed":false' in first
            assert send(request(1)).startswith(b'HTTP/1.1 409 Conflict')
            assert send(request(2, good=False)).startswith(b'HTTP/1.1 401 Unauthorized')
            assert send(request(2)).startswith(b'HTTP/1.1 200 OK')
            # A second server sharing the same SQLite owner must fail to start.
            other_dir = root / 'other'
            other_dir.mkdir(mode=0o700)
            other = subprocess.run([executable, '--serve-unix', str(other_dir/'probe.sock')],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            assert other.returncode != 0
        finally:
            stop(p)
        p = boot()
        try:
            assert send(request(1)).startswith(b'HTTP/1.1 409 Conflict')
            assert send(request(2)).startswith(b'HTTP/1.1 409 Conflict')
            assert send(request(3)).startswith(b'HTTP/1.1 200 OK')
        finally:
            stop(p)
        # Test that an invalid key permission prevents any socket startup.
        os.chmod(keyfile, 0o644)
        bad = subprocess.run([executable, '--serve-unix', str(sock)],env=env,
             stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        assert bad.returncode != 0 and not sock.exists()


if __name__ == '__main__':
    exercise(sys.argv[1])
    print('H5Z Unix/HMAC/SQLite/restart subprocess test PASS')
