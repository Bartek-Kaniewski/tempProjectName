// tools/panel_hook.js - przechwytywanie komend panelu Matrix BEZ proxy (bez roota, bez instalacji).
//
// Jak uzywac (dokladnie te kroki):
//   1. Otworz panel: http://192.168.71.50/#/homePage  (zaloguj sie, upewnij sie, ze dziala)
//   2. F12 -> zakladka Console -> wklej CALA zawartosc tego pliku -> Enter.
//      Ma pojawic sie: "haczyk zalozony".
//      (Nie trzeba przeladowywac strony - haczyk lapie TAKZE polaczenie, ktore juz dziala.)
//   3. W panelu wykonaj po JEDNEJ rzeczy naraz i zrob male przerwy:
//        - "przod" ok. 1 s i pusc,
//        - obrot w lewo ok. 1 s,
//        - "jedz do stacji 3" (jesli panel to ma),
//        - STOP / anuluj zadanie.
//   4. W konsoli wywolaj:  saveCapture()
//      -> pobierze sie plik panel_capture.log
//   5. Ten plik wrzuc do katalogu cap/ i uruchom (u siebie, na komputerze - nie na robocie):
//        ./amr_cmd --from-capture cap/panel_capture.log --type 17 --nth 1 --dry-run
//      albo po prostu przyslij mi ten plik - rozloze go na czynniki pierwsze.
//
// Format zapisu: linie "+<czas> UP|DN <hex>" - hex bez 4-bajtowego prefiksu dlugosci, czyli
// dokladnie to, co przyjmuje --frame / --dump-hex / --from-capture.
(() => {
  const S = globalThis;
  if (S.__amrHook) {
    console.log('haczyk juz zalozony. Wpisz saveCapture(), zeby zapisac zebrane ramki.');
    return;
  }
  S.__amrHook = true;

  const frames = [];          // zebrane linie
  const t0 = performance.now();
  const bufs = new WeakMap(); // bufor sklejania ramek per gniazdo
  const seen = new WeakSet(); // zeby nie podpinac nasluchu dwa razy

  const hex = (u8) => {
    let s = '';
    for (const b of u8) s += b.toString(16).padStart(2, '0');
    return s;
  };
  const ts = () => ((performance.now() - t0) / 1000).toFixed(3);

  // sklada kompletne ramki aplikacji (u32 BE dlugosc + protobuf) z fragmentow WebSocket
  const feed = (ws, dir, chunk) => {
    let buf = bufs.get(ws);
    if (!buf || !buf[dir]) buf = Object.assign({}, buf, {[dir]: new Uint8Array(0)});
    let b = new Uint8Array(buf[dir].length + chunk.length);
    b.set(buf[dir]);
    b.set(chunk, buf[dir].length);
    while (b.length >= 4) {
      const n = ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]) >>> 0;
      if (b.length < 4 + n) break;
      const line = '+' + ts() + ' ' + dir + ' ' + hex(b.subarray(4, 4 + n));
      frames.push(line);
      console.log(line);
      b = b.subarray(4 + n);
    }
    buf[dir] = b;
    bufs.set(ws, buf);
  };

  // --- 1) komendy wychodzace (UP): patch na prototypie => dziala tez dla polaczenia,
  //        ktore juz istnieje (bez przeladowania strony!)
  const origSend = S.WebSocket.prototype.send;
  S.WebSocket.prototype.send = function (data) {
    try {
      if (data instanceof ArrayBuffer) feed(this, 'UP', new Uint8Array(data));
      else if (ArrayBuffer.isView(data)) feed(this, 'UP', new Uint8Array(data.buffer, data.byteOffset, data.byteLength));
    } catch (e) {}
    return origSend.call(this, data);
  };

  // --- 2) ramki przychodzace (DN): dla polaczen tworzonych PO zalozeniu haczyka
  const attachIncoming = (ws) => {
    if (seen.has(ws)) return;
    seen.add(ws);
    ws.addEventListener('message', async (ev) => {
      const d = ev.data;
      try {
        if (d instanceof ArrayBuffer) feed(ws, 'DN', new Uint8Array(d));
        else if (typeof Blob !== 'undefined' && d instanceof Blob) feed(ws, 'DN', new Uint8Array(await d.arrayBuffer()));
        else if (ArrayBuffer.isView(d)) feed(ws, 'DN', new Uint8Array(d.buffer, d.byteOffset, d.byteLength));
      } catch (e) {}
    });
  };
  const Orig = S.WebSocket;
  const Patched = function (...args) {
    const ws = new Orig(...args);
    attachIncoming(ws);
    return ws;
  };
  Patched.prototype = Orig.prototype;
  Object.setPrototypeOf(Patched, Orig);
  S.WebSocket = Patched;

  S.__amrFrames = frames;
  S.saveCapture = () => {
    const text = '# zapis z panelu Matrix (haczyk JS), format jak amr_proxy --save\n' +
                 frames.join('\n') + '\n';
    const blob = new Blob([text], {type: 'text/plain'});
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'panel_capture.log';
    a.click();
    URL.revokeObjectURL(a.href);
    console.log('zapisano panel_capture.log (' + frames.length + ' ramek). ' +
                'Jesli plik sie nie pobral, wpisz: copy(__amrFrames.join("\\n"))');
  };

  console.log('haczyk zalozony (' + frames.length + ' ramek). ' +
              'Teraz pojezdzij w panelu, potem wywolaj: saveCapture()');
})();
