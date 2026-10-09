#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_wasd_pty.py - test amr_wasd bez czlowieka przy klawiaturze.

Uruchamia podane polecenie w pseudoterminalu (pty) i "trzyma" klawisze przez zadany czas
(terminal autopowtarza znaki, tak jak przy trzymaniu klawisza), potem puszcza i naciska X.

Uzycie:
  python3 tools/test_wasd_pty.py --keys w=1.5,d=1.0 -- ./amr_wasd --ip 127.0.0.1 --port 5003 \\
      --type 100 --set 2.1=@@vx@@ --set 2.2=@@wz@@ --stop-type 102 --speed 250 --turn 300
"""
import argparse
import os
import pty
import select
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--keys', default='w=1.5',
                    help='sekwencja: klawisz=sekundy, np. "w=1.5,d=1.0" (klawisz trzymany tyle sekund)')
    ap.add_argument('--pause', type=float, default=0.8, help='pauza po puszczeniu klawiszy (s)')
    ap.add_argument('cmd', nargs=argparse.REMAINDER, help='-- polecenie do uruchomienia')
    args = ap.parse_args()
    cmd = args.cmd[1:] if args.cmd and args.cmd[0] == '--' else args.cmd
    if not cmd:
        print('podaj polecenie po "--"', file=sys.stderr)
        return 2

    seq = []
    for part in args.keys.split(','):
        k, _, s = part.partition('=')
        seq.append((k, float(s or 1.0)))

    pid, fd = pty.fork()
    if pid == 0:  # dziecko
        os.execvp(cmd[0], cmd)
        os._exit(127)

    out = bytearray()

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                try:
                    data = os.read(fd, 8192)
                except OSError:
                    return False
                if not data:
                    return False
                out.extend(data)
        return True

    for key, dur in seq:
        print('[test] trzymam "%s" przez %.2f s' % (key, dur))
        end = time.time() + dur
        while time.time() < end:
            os.write(fd, key.encode())
            if not pump(0.04):
                break
    print('[test] puszczam klawisze (pauza %.2f s)' % args.pause)
    pump(args.pause)
    print('[test] wysylam "x" (wyjscie)')
    os.write(fd, b'x')
    pump(1.0)
    try:
        os.close(fd)
    except OSError:
        pass
    try:
        os.waitpid(pid, 0)
    except ChildProcessError:
        pass

    text = out.decode(errors='replace')
    # HUD uzywa \r - zamien na czytelne linie
    for line in text.replace('\r', '\n').split('\n'):
        line = line.strip('\x1b[K')
        if line.strip():
            print(line)


if __name__ == '__main__':
    sys.exit(main())
