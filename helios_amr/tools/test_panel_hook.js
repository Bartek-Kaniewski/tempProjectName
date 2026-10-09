// tools/test_panel_hook.js - test haczyka panel_hook.js bez przegladarki (node).
// Sprawdza:
//   A) haczyk zalozony PO utworzeniu polaczenia - i tak lapie komendy wychodzace (UP),
//   B) haczyk zalozony PRZED polaczeniem - lapie tez ramki przychodzace (DN),
//   C) ramke rozbita na kilka wiadomosci WebSocket skleja w calosc.
// Uruchomienie:  node tools/test_panel_hook.js
const fs = require('fs');
const { performance } = require('perf_hooks');

class StubWS {                        // metody na prototypie - jak w prawdziwym WebSocket
  constructor(url) { this.url = url; this.l = {}; }
  addEventListener(t, fn) { (this.l[t] = this.l[t] || []).push(fn); }
  send(d) { this.sent = (this.sent || []).concat([d]); }
  push(u8) { (this.l.message || []).forEach((fn) => fn({ data: u8 })); }
}

const logged = [];
const say = (...a) => process.stdout.write(a.join(' ') + '\n');   // prawdziwy stdout
global.console = { log: (...a) => logged.push(a.join(' ')) };
global.performance = performance;
global.WebSocket = StubWS;
global.Blob = class { constructor(p) { this.text = p.join(''); } };
global.URL = { createObjectURL: () => 'blob:x', revokeObjectURL: () => {} };
global.document = { createElement: () => ({ click() {}, set href(v) {}, set download(v) {} }) };

// polaczenie utworzone PRZED zalozeniem haczyka (tak jest w praktyce - wklejasz w konsoli)
const istniejace = new global.WebSocket('ws://192.168.71.50:5002/');

eval(fs.readFileSync(__dirname + '/panel_hook.js', 'utf8'));

const body = Buffer.from('0802156600000019092f005a0000000022090864120508b4011000', 'hex');
const frame = Buffer.concat([Buffer.from([0, 0, 0, body.length]), body]);
const u8 = new Uint8Array(frame);

// A) komenda z ISTNIEJACEGO polaczenia
istniejace.send(u8.subarray(0, u8.length));

// B) nowe polaczenie po zalozonym haczyku - ramka przychodzaca rozbita na 3 kawalki
const nowe = new global.WebSocket('ws://192.168.71.50:5002/');
nowe.push(u8.subarray(0, 1));
nowe.push(u8.subarray(1, 5));
nowe.push(u8.subarray(5));

// C) komenda z nowego polaczenia - ma dac dokladnie jedna linie UP
nowe.send(u8.subarray(0, u8.length));

global.saveCapture();

const expect = body.toString('hex');
const up = logged.filter((l) => l.includes(' UP '));
const dn = logged.filter((l) => l.includes(' DN '));
const ok = up.length === 2 && dn.length === 1 &&
           up.every((l) => l.endsWith(expect)) && dn[0].endsWith(expect);
say('UP (komendy):', up.length, '(oczekiwane 2: istniejace + nowe polaczenie)');
say('DN (stan):', dn.length, '(oczekiwana 1: z nowego polaczenia)');
say('przyklad:', up[0]);
say(ok ? 'OK: haczyk dziala (takze na juz otwartym polaczeniu)' : 'BLAD: ramki sie nie zgadzaja');
process.exit(ok ? 0 : 1);
