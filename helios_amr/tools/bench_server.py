#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_server.py - ATRAPA bazy Helios (Matrix OS) do testow bez robota.

Nie odtwarza prawdziwej bazy! Udaje ja tylko na tyle, zeby dalo sie przetestowac narzedzia
(amr_state_cli, amr_probe, amr_proxy, amr_cmd) i przecwiczyc caly lancuch:

  * przyjmuje WebSocket na porcie (domyslnie 5003),
  * przyjmuje logowanie (dowolne konto/haslo), przydziela sesje i wysyla push stanu,
  * odpowiada potwierdzeniem na kazde zadanie, wiec widac, ze ramka doszla,
  * ma WLASNY, wymyslony protokol ruchu (tylko na potrzeby lawki):
        type 100, pole 2 = { 1: vx [mm/s], 2: wz [mrad/s] }   -> jog (wygasa po 0.5 s bez powtorzen)
        type 101, pole 2 = { 1: x [mm], 2: y [mm], 3: kat [mrad] }  -> jedz do punktu
        type 102                                              -> STOP
    Prawdziwa baza ma oczywiscie inne kody - te sluza tylko do sprawdzenia narzedzi.

Przyklady:
  python3 tools/bench_server.py --port 5003 --auto-mission
  python3 tools/bench_server.py --port 5003 --log-frames
"""
import argparse
import base64
import hashlib
import math
import signal
import socket
import struct
import sys
import threading
import time

# ----------------------------------------------------------------- protobuf (mini)

def enc_varint(v):
    v &= (1 << 64) - 1
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)


def field_varint(num, v):
    return enc_varint(num << 3) + enc_varint(v)


def field_bytes(num, b):
    return enc_varint((num << 3) | 2) + enc_varint(len(b)) + b


def field_fixed32(num, v):
    return enc_varint((num << 3) | 5) + struct.pack('<I', v & 0xFFFFFFFF)


def field_fixed64(num, v):
    return enc_varint((num << 3) | 1) + struct.pack('<Q', v & ((1 << 64) - 1))


def read_varint(b, i):
    out = 0
    shift = 0
    while True:
        c = b[i]
        i += 1
        out |= (c & 0x7F) << shift
        if not (c & 0x80):
            return out, i
        shift += 7


def parse_msg(b):
    fields = []
    i = 0
    while i < len(b):
        key, i = read_varint(b, i)
        num, wire = key >> 3, key & 7
        if wire == 0:
            v, i = read_varint(b, i)
        elif wire == 1:
            v = struct.unpack_from('<Q', b, i)[0]
            i += 8
        elif wire == 2:
            n, i = read_varint(b, i)
            v = b[i:i + n]
            i += n
        elif wire == 5:
            v = struct.unpack_from('<I', b, i)[0]
            i += 4
        else:
            raise ValueError('wire=%d' % wire)
        fields.append((num, wire, v))
    return fields


def as_i64(v):
    """protobuf int64 = dwa uzupelnienia; varint >= 2^63 to liczba ujemna"""
    if isinstance(v, int) and v >= (1 << 63):
        return v - (1 << 64)
    return v


def get_field(fields, num, wire=None):
    for n, w, v in fields:
        if n == num and (wire is None or w == wire):
            return v
    return None


def show_msg(b, indent=0, limit=64):
    try:
        fs = parse_msg(b)
    except Exception:
        return ' ' * indent + '(nie protobuf)'
    lines = []
    for n, w, v in fs[:limit]:
        if w == 0:
            lines.append('%s#%d varint %d' % (' ' * indent, n, v))
        elif w == 2:
            try:
                sub = parse_msg(v)
                if sub:
                    lines.append('%s#%d msg (%d B)' % (' ' * indent, n, len(v)))
                    lines.append(show_msg(v, indent + 2, limit))
                    continue
            except Exception:
                pass
            try:
                lines.append('%s#%d str "%s"' % (' ' * indent, n, v.decode('utf-8')))
            except Exception:
                lines.append('%s#%d bytes %s' % (' ' * indent, n, v[:16].hex()))
        else:
            lines.append('%s#%d wire%d %d' % (' ' * indent, n, w, v))
    return '\n'.join(lines)


# ----------------------------------------------------------------- symulacja bazy

class Sim:
    """Bardzo prosta symulacja: punkt na plaszczyznie, jazda po prostej."""

    def __init__(self):
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = -22.0
        self.yaw = -math.pi / 2       # -1570 mrad, jak w Waszym logu
        self.phase = 2                # 2 spoczynek, 10 start, 7 jazda
        self.dist_cm = 0
        self.route = []               # (sx, sy, ex, ey) w mm
        self.vx = 0.0                 # mm/s
        self.wz_mrad = 0.0            # mrad/s
        self.vel_until = 0.0          # deadman dla joga
        self.target = None
        self.phase_deadline = 0.0
        self.map = 'map1'
        self.mission_at = 0.0

    # --- komendy lawki
    def cmd_jog(self, vx, wz_mrad):
        with self.lock:
            self.target = None
            self.vx = vx
            self.wz_mrad = wz_mrad
            self.vel_until = time.time() + 0.5

    def cmd_stop(self):
        with self.lock:
            self.target = None
            self.vx = self.wz_mrad = 0.0
            self.vel_until = 0.0
            self.phase = 2
            self.dist_cm = 0
            self.route = []

    def cmd_goto(self, x, y, yaw_mrad):
        with self.lock:
            self.target = (float(x), float(y), yaw_mrad / 1000.0)
            self.phase = 10
            self.phase_deadline = time.time() + 0.6
            self.route = [(self.x, self.y, float(x), float(y))]

    def start_mission(self):
        with self.lock:
            self.x, self.y, self.yaw = 0.0, -22.0, -math.pi / 2
            self.cmd_goto_locked(0.0, -1532.0, -1570.0)

    def cmd_goto_locked(self, x, y, yaw_mrad):
        self.target = (float(x), float(y), yaw_mrad / 1000.0)
        self.phase = 10
        self.phase_deadline = time.time() + 0.6
        self.route = [(self.x, self.y, float(x), float(y))]

    def step(self, dt):
        with self.lock:
            now = time.time()
            if self.target and now >= self.phase_deadline:
                tx, ty, tyaw = self.target
                dx, dy = tx - self.x, ty - self.y
                dist = math.hypot(dx, dy)
                if dist < 5.0:
                    self.x, self.y, self.yaw = tx, ty, tyaw
                    self.target = None
                    self.phase = 2
                    self.dist_cm = 0
                    self.route = []
                else:
                    want = math.atan2(dy, dx)
                    err = math.atan2(math.sin(want - self.yaw), math.cos(want - self.yaw))
                    self.phase = 7
                    if abs(err) > 0.05:
                        self.yaw += max(-0.6, min(0.6, err)) * dt
                    else:
                        v = 200.0  # mm/s
                        step = min(v * dt, dist)
                        self.x += math.cos(self.yaw) * step
                        self.y += math.sin(self.yaw) * step
                    self.dist_cm = int(round(math.hypot(tx - self.x, ty - self.y) / 10.0))
            elif self.target is None:
                if now < self.vel_until:
                    self.phase = 7
                    v = self.vx  # mm/s
                    self.x += math.cos(self.yaw) * v * dt
                    self.y += math.sin(self.yaw) * v * dt
                    self.yaw += (self.wz_mrad / 1000.0) * dt
                elif self.vx or self.wz_mrad:
                    self.vx = self.wz_mrad = 0.0
                    self.phase = 2
            if self.phase == 10 and now >= self.phase_deadline and not self.target:
                self.phase = 2

    # --- serializacja stanu
    def state_payload(self):
        with self.lock:
            x, y, yaw = self.x, self.y, self.yaw
            phase, dist, route, mapname = self.phase, self.dist_cm, list(self.route), self.map
        pose = (field_varint(2, int(round(x))) + field_varint(3, int(round(y))) +
                field_varint(7, int(round(yaw * 1000))))
        task = field_varint(8, int(dist))
        for (sx, sy, ex, ey) in route:
            seg = (field_varint(2, int(sx)) + field_varint(3, int(sy)) +
                   field_varint(4, int(ex)) + field_varint(5, int(ey)))
            task += field_bytes(12, seg)
        st = (field_varint(2, int(phase)) + field_bytes(4, pose) + field_bytes(7, task) +
              field_bytes(10, mapname.encode()))
        return field_varint(1, 2) + field_bytes(3, st)


def envelope(kind, seq, session, content_field, content):
    return (field_varint(1, kind) + field_fixed32(2, seq) + field_fixed64(3, session) +
            field_bytes(content_field, content))


def framed(body):
    return struct.pack('>I', len(body)) + body


# ----------------------------------------------------------------- WebSocket (serwer)

def ws_accept(key):
    return base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()


def ws_send(sock, data):
    n = len(data)
    if n < 126:
        hdr = bytes([0x82, n])
    elif n < 65536:
        hdr = bytes([0x82, 126]) + struct.pack('>H', n)
    else:
        hdr = bytes([0x82, 127]) + struct.pack('>Q', n)
    sock.sendall(hdr + data)


class Conn:
    def __init__(self, sock, addr, sim, opts):
        self.sock = sock
        self.addr = addr
        self.sim = sim
        self.opts = opts
        self.session = 0x5A000000 + (int(time.time()) & 0xFFFF)
        self.seq = 100
        self.alive = True
        self.lock = threading.Lock()

    def log(self, msg):
        print('[bench %s] %s' % (self.addr, msg))
        sys.stdout.flush()

    def send_body(self, body):
        with self.lock:
            ws_send(self.sock, framed(body))

    def run(self):
        try:
            if not self.handshake():
                return
            self.log('polaczono (atrapa bazy)')
            t = threading.Thread(target=self.pusher, daemon=True)
            t.start()
            self.reader()
        except Exception as e:
            self.log('blad: %r' % (e,))
        finally:
            self.alive = False
            try:
                self.sock.close()
            except Exception:
                pass
            self.log('rozlaczono')

    def handshake(self):
        buf = b''
        while b'\r\n\r\n' not in buf:
            d = self.sock.recv(4096)
            if not d:
                return False
            buf += d
        head, _, rest = buf.partition(b'\r\n\r\n')
        key = ''
        for line in head.decode(errors='replace').split('\r\n'):
            if line.lower().startswith('sec-websocket-key:'):
                key = line.split(':', 1)[1].strip()
        if not key:
            return False
        resp = ('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                'Sec-WebSocket-Accept: %s\r\n\r\n' % ws_accept(key))
        self.sock.sendall(resp.encode())
        self.appbuf = rest
        return True

    def reader(self):
        while self.alive:
            d = self.sock.recv(8192)
            if not d:
                return
            payloads = self.ws_parse(d)
            for p in payloads:
                self.appbuf += p
            while len(self.appbuf) >= 4:
                n = struct.unpack('>I', self.appbuf[:4])[0]
                if len(self.appbuf) < 4 + n:
                    break
                body = self.appbuf[4:4 + n]
                self.appbuf = self.appbuf[4 + n:]
                self.handle(body)

    def ws_parse(self, data):
        out = []
        self.wsbuf = getattr(self, 'wsbuf', b'') + data
        while True:
            b = self.wsbuf
            if len(b) < 2:
                break
            op = b[0] & 0x0F
            masked = b[1] & 0x80
            ln = b[1] & 0x7F
            off = 2
            if ln == 126:
                if len(b) < 4:
                    break
                ln = struct.unpack('>H', b[2:4])[0]
                off = 4
            elif ln == 127:
                if len(b) < 10:
                    break
                ln = struct.unpack('>Q', b[2:10])[0]
                off = 10
            if len(b) < off + (4 if masked else 0) + ln:
                break
            mask = b[off:off + 4] if masked else b''
            if masked:
                off += 4
            pay = bytearray(b[off:off + ln])
            if masked:
                for i in range(len(pay)):
                    pay[i] ^= mask[i & 3]
            self.wsbuf = b[off + ln:]
            if op in (0x0, 0x1, 0x2):
                out.append(bytes(pay))
            elif op == 0x8:
                self.alive = False
                return out
            elif op == 0x9:
                # pong
                self.sock.sendall(bytes([0x8A, len(pay)]) + bytes(pay))
        return out

    def pusher(self):
        while self.alive:
            try:
                payload = self.sim.state_payload()
                self.seq += 1
                self.send_body(envelope(2, self.seq, self.session, 5, payload))
            except Exception:
                return
            time.sleep(0.2)

    def handle(self, body):
        try:
            fs = parse_msg(body)
        except Exception:
            self.log('ramka niezdekodowana: %s' % body[:32].hex())
            return
        kind = get_field(fs, 1, 0) or 0
        seq = get_field(fs, 2, 5) or 0
        inner = get_field(fs, 4, 2) or get_field(fs, 5, 2)
        # kanal 6: tresc = {2: opkod, 3/4/...: parametry} - tak panel wysyla jog (op=16)
        op = None
        op_fields = []
        ch6 = get_field(fs, 6, 2)
        if ch6:
            try:
                op_fields = parse_msg(ch6)
                op = get_field(op_fields, 2, 0)
            except Exception:
                pass
        type_ = None
        payload = b''
        if inner:
            try:
                ifs = parse_msg(inner)
                type_ = get_field(ifs, 1, 0)
                extra = [f for f in ifs if f[0] != 1]
                if extra:
                    payload = b''.join(
                        field_varint(n, v) if w == 0 else
                        field_bytes(n, v) if w == 2 else
                        field_fixed32(n, v) if w == 5 else field_fixed64(n, v)
                        for (n, w, v) in extra)
            except Exception:
                pass
        self.log('<<< kind=%s seq=%s typ=%s op=%s%s' % (kind, seq, type_,
                 ('%d' % op) if op is not None else '-',
                 (' (%d B tresci)' % len(payload)) if payload else ''))
        if self.opts.log_frames:
            self.log('    ' + show_msg(body, 4).replace('\n', '\n    '))

        def unwrap_params(payload):
            # Nasze narzedzia wysylaja tresc zadania jako {1: typ, 2: {parametry}} - wyciagamy pole 2.
            try:
                fs2 = parse_msg(payload)
            except Exception:
                return b''
            for (n, w, v) in fs2:
                if n == 2 and w == 2:
                    return v
            return b''

        if type_ == 1 and seq == 1:
            # logowanie -> przydziel sesje i wyslij stan startowy w tresci odpowiedzi
            ack_payload = self.sim.state_payload()
            self.send_body(envelope(1, 1, self.session, 5, ack_payload))
            self.log('>>> logowanie OK, sesja = 0x%x' % self.session)
            return

        params = unwrap_params(payload)

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        if op == 16:  # JOG - dokladnie jak panel (pole 3 = jazda, pole 4 = obrot)
            v1 = clamp(as_i64(get_field(op_fields, 3, 0) or 0), -3000, 3000)
            v2 = clamp(as_i64(get_field(op_fields, 4, 0) or 0), -3000, 3000)
            self.sim.cmd_jog(v1, v2)
            self.log('>>> JOG(op16) vx=%d wz=%d' % (v1, v2))
        elif op in (6, 7, 111):
            self.log('>>> OP%d (przejecie/oddanie sterowania - atrapa ignoruje)' % op)
        elif op == 32:
            self.log('>>> OP32 (zadanie - atrapa ignoruje)')
        elif op is not None:
            self.log('>>> OP%d (nieznane)' % op)

        if type_ == 100:  # jog (protokol LAWKI)
            try:
                pf = parse_msg(params)
                v1 = clamp(as_i64(get_field(pf, 1, 0) or 0), -2000, 2000)
                v2 = clamp(as_i64(get_field(pf, 2, 0) or 0), -2000, 2000)
                self.sim.cmd_jog(v1, v2)
                self.log('>>> JOG vx=%d mm/s, wz=%d mrad/s' % (v1, v2))
            except Exception:
                self.log('>>> JOG: nie umiem odczytac parametrow %s' % params.hex())
        elif type_ == 101:  # goto (protokol LAWKI)
            try:
                pf = parse_msg(params)
                x = clamp(as_i64(get_field(pf, 1, 0) or 0), -100000, 100000)
                y = clamp(as_i64(get_field(pf, 2, 0) or 0), -100000, 100000)
                yaw = clamp(as_i64(get_field(pf, 3, 0) or 0), -4000, 4000)
                self.sim.cmd_goto(x, y, yaw)
                self.log('>>> GOTO x=%s y=%s kat=%s mrad' % (x, y, yaw))
            except Exception:
                self.log('>>> GOTO: nie umiem odczytac parametrow %s' % params.hex())
        elif type_ == 102:
            self.sim.cmd_stop()
            self.log('>>> STOP')
        # potwierdzenie dla kazdego zadania
        ack = field_varint(1, op if op is not None else (type_ if type_ is not None else 0))
        self.send_body(envelope(1, seq, self.session, 5, ack))


# ----------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description='Atrapa bazy Helios (Matrix OS) - tylko testy!')
    ap.add_argument('--port', type=int, default=5003)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--auto-mission', action='store_true',
                    help='po 3 s od startu odgrywaj misje 1->3 (fazy 2->10->7->2, dystans 151->1 cm)')
    ap.add_argument('--log-frames', action='store_true', help='drukuj pelne drzewo odebranych ramek')
    opts = ap.parse_args()

    sim = Sim()

    def physics():
        last = time.time()
        while True:
            now = time.time()
            sim.step(min(0.1, now - last))
            last = now
            time.sleep(0.02)

    threading.Thread(target=physics, daemon=True).start()

    if opts.auto_mission:
        def mission():
            time.sleep(3.0)
            sim.start_mission()
            print('[bench] start misji 1 -> 3 (y: -22 mm -> -1532 mm)')
        threading.Thread(target=mission, daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((opts.host, opts.port))
    srv.listen(4)
    print('bench_server (ATRAPA bazy, nie prawdziwa!) na %s:%d' % (opts.host, opts.port))
    print('Wlasny protokol ruchu: type 100 jog {1:vx mm/s, 2:wz mrad/s}, 101 goto, 102 stop')
    sys.stdout.flush()

    def bye(*_):
        print('\n[bench] koniec')
        sys.exit(0)

    signal.signal(signal.SIGINT, bye)
    while True:
        try:
            s, addr = srv.accept()
        except OSError:
            break
        threading.Thread(target=Conn(s, addr, sim, opts).run, daemon=True).start()


if __name__ == '__main__':
    main()
