#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_capture.py - rozklada zapis ramek (amr_proxy --save / panel_hook saveCapture)
na grupy o tym samym "ksztalcie" pol i pokazuje, co sie w nich zmienia w czasie.

Do czego sluzy: gdy nie znamy znaczenia pol, grupujemy ramki po zestawie obecnych pol
(field-ow) i patrzymy, ktore wartosci zmieniaja sie razem z tym, co robisz w panelu.

Uzycie:
  python3 tools/analyze_capture.py cap/panel_paste.log
  python3 tools/analyze_capture.py cap/panel_paste.log --timeline 3      # pelna os czasu grupy 3
  python3 tools/analyze_capture.py cap/panel_paste.log --dir UP
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_server import parse_msg  # mini-dekoder protobuf (ten sam, co w atrapie)


def as_i64(v):
    return v - (1 << 64) if v >= (1 << 63) else v


def flatten(msg, prefix=''):
    """Zamienia komunikat w slownik sciezka->wartosc (sciezki typu '6.3')."""
    out = {}
    counts = {}
    try:
        fields = parse_msg(msg)
    except Exception:
        return {'(nie protobuf)': msg.hex()}
    for (num, wire, val) in fields:
        key = str(num)
        counts[key] = counts.get(key, 0) + 1
        path = prefix + key + (('[%d]' % (counts[key] - 1)) if counts[key] > 1 else '')
        if wire == 2:
            sub = None
            if val:
                try:
                    sub = parse_msg(val)
                except Exception:
                    sub = None
            if sub:
                out.update(flatten(val, path + '.'))
            else:
                try:
                    s = val.decode('utf-8')
                    out[path] = '"%s"' % s if s.isprintable() else 'h:' + val.hex()
                except Exception:
                    out[path] = 'h:' + val.hex()
        elif wire == 0:
            out[path] = '%d (int64 %d)' % (val, as_i64(val)) if val >= (1 << 63) else str(val)
        else:
            out[path] = str(val)
    return out


def norm_path(p):
    return p.split('[')[0]


def read_frames(path):
    frames = []
    for line in open(path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        tok = line.split()
        if len(tok) < 3:
            continue
        try:
            t = float(tok[0].lstrip('+'))
            body = bytes.fromhex(tok[-1])
        except ValueError:
            continue
        frames.append({'t': t, 'dir': tok[-2], 'body': body, 'seq': tok[0]})
    return frames


def fmt_time(t):
    return '%.3f' % t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('file')
    ap.add_argument('--dir', default=None, help='filtruj kierunek (UP/DN)')
    ap.add_argument('--timeline', type=int, default=None, help='pelna os czasu grupy nr N')
    ap.add_argument('--min-count', type=int, default=1, help='pokaz grupy z co najmniej N ramkami')
    args = ap.parse_args()

    frames = read_frames(args.file)
    if args.dir:
        frames = [f for f in frames if f['dir'] == args.dir]
    if not frames:
        print('brak ramek w pliku')
        return 1

    flat = []
    for f in frames:
        fl = flatten(f['body'])
        sig = tuple(sorted(set(norm_path(p) for p in fl)))
        f['flat'] = fl
        f['sig'] = sig

    # grupowanie po (kierunek, ksztalt pol)
    groups = {}
    for i, f in enumerate(frames):
        groups.setdefault((f['dir'], f['sig']), []).append(i)

    order = sorted(groups.items(), key=lambda kv: min(kv[1]))

    print('== %s: %d ramek, t = %s .. %s s ==' % (args.file, len(frames), fmt_time(frames[0]['t']),
                                                  fmt_time(frames[-1]['t'])))
    for gi, ((d, sig), idxs) in enumerate(order):
        if len(idxs) < args.min_count:
            continue
        gh = [frames[i] for i in idxs]
        print('\nGRUPA %d  %s  pola: %s   [%d ramek]   t=%s..%s' %
              (gi, d, ','.join(sig), len(gh), fmt_time(gh[0]['t']), fmt_time(gh[-1]['t'])))
        if args.timeline == gi:
            for h in gh:
                pairs = ' '.join('%s=%s' % (k, v) for k, v in h['flat'].items())
                print('   %s  %s' % (fmt_time(h['t']), pairs))
            continue
        # statystyki per sciezka + ciagle serie
        allp = sorted(set(norm_path(p) for h in gh for p in h['flat']))
        for p in allp:
            if p == '2':
                print('   %-6s = %s..%s (numer sekwencji, rosnie o 1)' %
                      (p, gh[0]['flat'].get('2'), gh[-1]['flat'].get('2')))
                continue
            vals = []
            for h in gh:
                v = None
                for k in h['flat']:
                    if norm_path(k) == p:
                        v = h['flat'][k]
                vals.append(v)
            uniq = []
            for v in vals:
                if v not in uniq:
                    uniq.append(v)
            if len(uniq) == 1:
                print('   %-6s = %s (stale)' % (p, uniq[0]))
            else:
                shown = ', '.join('%s x%d' % (v, vals.count(v)) for v in uniq[:6])
                print('   %-6s : %s%s' % (p, shown, ' ...' if len(uniq) > 6 else ''))
                # serie: kolejne ramki z ta sama wartoscia
                runs = []
                cur_v, start, cnt = vals[0], gh[0]['t'], 1
                for k in range(1, len(gh)):
                    if vals[k] == cur_v:
                        cnt += 1
                    else:
                        runs.append((cur_v, start, gh[k - 1]['t'], cnt))
                        cur_v, start, cnt = vals[k], gh[k]['t'], 1
                runs.append((cur_v, start, gh[-1]['t'], cnt))
                if len(runs) > 1:
                    for (v, t0, t1, n) in runs:
                        print('           seria %-12s t=%s..%s  (%d ramek, %.2f s)' %
                              (v, fmt_time(t0), fmt_time(t1), n, t1 - t0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
