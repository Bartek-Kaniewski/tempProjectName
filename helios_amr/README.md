# Helios – baza jezdna (Matrix OS): odczyt i sterowanie z C++

Zestaw narzędzi do **bazy jezdnej robota Helios** (ROKAE). Baza działa na własnym systemie
(Standard Robots „Matrix OS") i gada po WebSocket na porcie **5002** — to **nie** jest xCore ani
Robot Assist, więc SDK od ROKAE tu nie sięga. Wasz `amr_matrix.hpp` (odczyt) został rozwinięty tak,
żeby dało się **wysyłać komendy** — w tym odtwarzać komendy ruchu przechwycone z panelu Matrix.

> Uwaga: samego *kodowania* komend ruchu bazy nie ma w dokumentacji. Dlatego podstawowym trybem
> pracy jest: **przechwyć ruch panelu (`amr_proxy`) → wskaż ramkę → odtwórz ją z kodu (`amr_cmd`)**.
> Format da się przy tym zmieniać polami (`--set`), czyli np. zmienić prędkość w komendzie „przód".

---

## 1. Co jest w projekcie

```
helios_amr/
├── include/amr_matrix.hpp     # rdzeń protokołu: WebSocket + protobuf + klient + warstwa wysyłki
├── src/amr_state_cli.cpp      # (Wasz) tylko odczyt: pozycja/faza/trasa — + --full, --segments
├── src/amr_probe.cpp          # podglądacz ruchu + próbnik: dekoduje ramki, pokazuje różnice, wysyła
├── src/amr_proxy.cpp          # MITM: panel → ten program → baza; loguje i zapisuje ruch do odtworzenia
├── src/amr_cmd.cpp            # „ster": odtwarza ramki (pętla + ramka stopu), mierzy przemieszczenie
├── src/amr_wasd.cpp           # ręczne prowadzenie bazy z klawiatury (W/S/A/D, hold-to-move)
├── src/amr_selftest.cpp       # testy rdzenia protokołu (bez robota)
├── tools/bench_server.py      # ATRAPA bazy do testów narzędzi bez robota (własny protokół ruchu)
├── baza.sh                    # SKROT: wasd / przod / tyl / lewo / prawo / stop / stan / podglad
├── tools/panel_hook.js        # haczyk do konsoli DevTools: zapis ramek panelu BEZ proxy
├── tools/analyze_capture.py   # rozkłada zapis na grupy ramek i pokazuje, co się zmienia
├── tools/test_wasd_pty.py     # test amr_wasd bez człowieka (sterowanie przez pty)
├── tools/test_panel_hook.js   # test haczyka (node, bez przeglądarki)
├── cap/panel_paste.log        # ZRZUT Z PRAWDZIWEJ BAZY: komendy panelu (jog itd.)
├── cap/panel_*.hex            # pojedyncze ramki z tego zrzutu, gotowe dla --frame / --pre-frame
├── cap/bench.log              # przykład zapisu z atrapy (do ćwiczeń bez robota)
└── Makefile
```

Budowanie i testy:

```bash
make            # buduje wszystkie narzędzia (g++ -std=c++17, bez zewnętrznych bibliotek)
make test       # ./amr_selftest - testy parsera/koperty/patchowania/stanu
```

---

## 2. Co wiemy o protokole (stan na dziś)

### Transport
* `ws://<IP>:5002/` — WebSocket binarny, nagłówek `Origin: http://<IP>`.
* **Ramka aplikacji** = `u32 big-endian (długość)` + protobuf. Jedna ramka aplikacji może być
  rozbita na kilka wiadomości WebSocket (nasz klient to składa).

### Koperta (protobuf)
| pole | typ | znaczenie |
|---|---|---|
| 1 | varint | rodzaj: `0` = żądanie z panelu, `1` = odpowiedź, `2` = push (m.in. strumień stanu) |
| 2 | fixed32 | numer sekwencji |
| 3 | fixed64 | identyfikator sesji (baza nadaje go po zalogowaniu) |
| 4 | len | treść żądania — `{1: kod zadania, …}` |
| 5 | len | treść odpowiedzi/pushu — `{1: kod, …}` |

* **Logowanie** (pierwsza ramka): pole 4 → `{1: 0, 2: {1: login, 2: MD5(hasło) w hex, 3: 30}}`.
  Klient admina ma `21232f297a57a5a743894a0e4a801fc3` (MD5 z „admin").
  Inne hasło: `echo -n 'hasło' | md5sum`.
* Po odpowiedzi na logowanie (seq 1) panel wysyła trzy zapytania: **typ 1, 17, 4** (nasze narzędzia
  robią to samo — `bootstrap_requests`). Co dokładnie znaczą, ustalimy przez `amr_probe`.

### Push typu 2 = stan bazy (to, o co pytałeś)
| pole | co to | pewność |
|---|---|---|
| `3.2` | faza zadania: **2 = spoczynek, 10 = start, 7 = jazda** | wysoka (Wasze obserwacje) |
| `3.4.2` | **x** w mm | pewne |
| `3.4.3` | **y** w mm | pewne |
| `3.4.7` | **kąt** w 1/1000 rad (mrad), np. −1570 = −π/2 | pewne |
| `3.7.8` | **pozostały dystans** — 1 jednostka = 10 mm (1 cm) | bardzo prawdopodobne |
| `3.7.12[n]` | odcinki trasy `(sx, sy, ex, ey)` w mm | prawdopodobne (kolejność pól z 2,3,4,5) |
| `3.10` | nazwa mapy | pewne |

**Skąd wniosek o jednostce dystansu:** stacja 1 ma y = −22 mm, stacja 3 y = −1532 mm, czyli trasa
1→3 ma dokładnie **1510 mm**. Twoje odczyty 151 → 1 dają 150 jednostek na 1500 mm, więc
**1 jednostka = 10 mm = 1 cm** (nie „prawdopodobnie cm z dokładnością do 1", tylko 1 cm wprost).
Zapas 151→1 na trasie 1510 mm mieści się w tym z błędem 10 mm.

Pozostałe znane kody: `type=2` to właśnie push stanu, `type=0` to treść logowania,
`type=18` to **heartbeat** (panel wysyła go co ~3 s).

### Kanał komend panelu = pole 6 (to jest to, czego brakowało)

Z przechwyconego ruchu prawdziwego panelu (`cap/panel_paste.log`) wynika, że **komendy ruchu lecą
innym kanałem niż zapytania**: nie polem 4 (`0x22`), a **polem 6 (`0x32`)**, o rodzaju `kind=2`.
Treść pola 6 = `{2: opkod, …parametry…}`:

| opkod | treść | znaczenie (**potwierdzone przez użytkownika** na przebiegu z panelu) |
|---|---|---|
| **16** | `{2:16, 3: v1, 4: v2, 7: 0}` | **strumień jogu ~5 Hz**: pole 3 = **jazda, + = przód** (w zrzucie 330), pole 4 = **obrót, + = w lewo, − = w prawo** (w zrzucie +266 / −266) |
| 6 | `{2:6, 7:0}` | „przejmij sterowanie" — raz przed jazdą (razem z op=111) |
| 111 | `{2:111, 7:0}` | leci zawsze w parze z op=6 (dokładnie 9 ms po nim) |
| 7 | `{2:7, 7:0}` | „oddaj sterowanie" — po zakończeniu jazdy |
| 32 | `{2:32, 7:0, 9:{3:2, 4:2, 5:03, 11:1}}` | **zlecenie zadania** („jedź do..."); baza potwierdza po ~8,5 s odpowiedzią `type=11`, `{14:{1:1, 2:0}}` |

Jednostki: pole 3 ≈ **mm/s**, pole 4 ≈ **mrad/s** (266 ≈ 15°/s) — wartości zgrubne, do
dokładnego ustalenia pierwszym przejazdem (procedura kalibracji w rozdziale 4). Przy 330 przez
2,7 s wyszło „~0,5 m", a nie 0,9 m, więc albo baza sama rampuje rozruch, albo skala nie jest 1:1.

Role opkodów (ustalone z zachowania panelu i jazdy próbnej): `op=6`+`op=111` = przejmij
sterowanie (raz, na początku), `op=16` = strumień jogu, **`op=7` = oddaj sterowanie — po nim baza
przestaje słuchać jogu, dopóki nie dostanie znowu `op=6`+`op=111`**.

Ważne obserwacje z tego zrzutu:
* **martwy człowiek (deadman):** panel **nie wysyła żadnej ramki „stop"** — strumień jogu po prostu
  się urywa (14 ramek → cisza) i baza sama się zatrzymuje. Nasze narzędzia i tak wysyłają na koniec
  jawną ramkę z zerowymi prędkościami + op=7 (bezpieczniej). *Nie wiadomo jeszcze, czy nasza sesja
  dostanie ten sam przywilej „samo-stop" — dlatego zawsze wysyłamy jawne zera.*
* jog leci ~5 Hz (co ~0,2 s), a **heartbeat (typ 18) wplata się między ramki jogu** (sekwencje rosną
  wspólnym licznikiem).
* prędkości z jogu: 330 i ±266 → przy założeniu, że pole 3 to mm/s, a pole 4 to mrad/s
  (15°/s), robot w 2,7 s przejechałby ~0,9 m. **Do potwierdzenia pomiarem.**

Jednostki i przypisanie pól (które jest „przód", które „obrót", znaki) są jeszcze hipotezą —
potwierdzamy je jedną krótką jazdą (patrz rozdział niżej).

---

## 3. Jak rozgryźć komendy ruchu (procedura krok po kroku)

Panel Matrix to **strona webowa** (`http://192.168.71.50/#/homePage`), a przeglądarka łączy się z bazą
bezpośrednio: `ws://192.168.71.50:5002`. Komendy ruchu przechwytujemy więc **po stronie komputera,
na którym masz otwarty panel** — nie trzeba nic zmieniać w bazie ani wyłączać panelu.

### Krok 1. Wybierz sposób przechwycenia

**Droga A (najprostsza — bez roota, bez instalowania czegokolwiek): haczyk w konsoli przeglądarki**

1. Otwórz panel i zaloguj się (upewnij się, że działa, i że stoisz z dala od robota).
2. F12 → Console → wklej **całą** zawartość `tools/panel_hook.js` → Enter.
   Ma pojawić się `haczyk założony`. **Nie przeładowuj strony** — haczyk łapie także
   połączenie, które już działa (to była pułapka: po F5 kod z konsoli znika).
3. W panelu wykonaj **jedną akcję naraz**, z przerwami: „przód” ~1 s i puść, obrót w lewo ~1 s,
   „jedź do stacji 3”, STOP/anuluj.
4. W konsoli wpisz `saveCapture()` — pobierze się plik `panel_capture.log`.

**Droga B (pełny, dwukierunkowy zapis przez proxy; Linux + sudo)**

```bash
# proxy jako root, żeby nie łapało samego siebie:
sudo ./amr_proxy --listen 127.0.0.1:5002 --remote 192.168.71.50:5002 --save cap/panel.log

# przekieruj na proxy tylko ruch swojego użytkownika (czyli przeglądarki z panelem):
sudo iptables -t nat -A OUTPUT -p tcp -d 192.168.71.50 --dport 5002 \
     -m owner --uid-owner $UID -j REDIRECT --to-ports 5002

# ...po zakończeniu zdejmij regułę:
sudo iptables -t nat -D OUTPUT -p tcp -d 192.168.71.50 --dport 5002 \
     -m owner --uid-owner $UID -j REDIRECT --to-ports 5002
```

Proxy nic nie zmienia w ruchu (przepisuje bajty 1:1), a na konsoli od razu pokazuje ramki z panelu
**w całości** i pod ścieżkami pól widać tylko **różnice** przy powtórzeniach.

### Krok 2. Znajdź komendę ruchu

W pliku (albo na konsoli proxy) szukasz ramek `UP`, które pojawiły się dopiero przy wciskaniu
przycisków. Podsumowanie z proxy (Ctrl-C) wypisuje tabelkę `UP kind type -> liczba` — nowe typy to
właśnie komendy ruchu.

```bash
./amr_probe --dump-hex <HEX_Z_ZAPISU>            # co jest w środku (offline)
./amr_cmd --from-capture cap/panel_capture.log --type 17 --nth 1 --dry-run   # obejrzyj ramkę
```

### Krok 3. Ustal znaczenie pól i przetestuj (na robocie, małe wartości)

Zrobione dla Twojej bazy: **`6.3` = jazda (mm/s, + przód), `6.4` = obrót (mrad/s, + w lewo)**,
a cała procedura pierwszego testu jest w rozdziale 4. Sposób ogólny — gdyby kiedyś doszła inna
komenda: analogicznie podstaw `--from-capture` + `--set` i patrz na drukowane przemieszczenie.

### Krok 4. Dopisz to do swojego kodu

Komendy z panelu są w kanale `pole 6`, o rodzaju `kind=2` — wysyła się je tak samo jak każde inne
ciało (koperta bez zmian). W C++ wystarczy złożyć zadanie i podmienić pola:

```cpp
amr::MatrixClient client(opt);            // opt.host = "192.168.71.50"
client.connect(err);
client.pollAny(100, nullptr, nullptr);    // czekamy na sesję

// zadanie jogu, op=16: pole 6 = {2:16, 3:vx, 4:wz, 7:0}
std::vector<amr::Node> op;
op.push_back(amr::varintNode(2, 16));     // opkod
op.push_back(amr::varintNode(3, vx));     // jazda, mm/s (+ przod)
op.push_back(amr::varintNode(4, wz));     // obrot, mrad/s (+ lewo)
op.push_back(amr::varintNode(7, 0));
std::vector<amr::Node> body;
body.push_back(amr::varintNode(1, 2));                     // kind=2 (komenda)
body.push_back(amr::stringNode(6, amr::serializeTree(op))); // pole 6 = kanal komend
client.sendBody(amr::serializeTree(body));

// przed jazda raz: op=6 (przejmij) i op=111; po jezdzie: op=7 (oddaj)
```

Zamiast wpisywać wartości na sztywno, można wczytać zapisaną ramkę i podmienić pola —
dokładnie to robi `--from-capture` + `--set` (funkcja `readCaptureSelect` + `applyPatch`).

## 4. Sterowanie baza — na co dzień `./baza.sh`

**STATUS: pierwszy ruch z naszego kodu wykonany na prawdziwej bazie (7.10.2026).** Robot ruszył
dokładnie tak, jak miał: komenda 50 → ~50 mm/s, komenda 100 → ~103 mm/s w fazie stałej
(czyli **jednostka to mm/s 1:1**; różnica w przebiegu to tylko rampa rozruchu bazy). Polecenia
op=6/op=111 przed jazdą i zera + op=7 po jeździe zostały przyjęte bez protestu.

Wygodna nakładka na całą wiedzę z tego projektu:

```bash
cd helios_amr
./baza.sh                      # wypisze pomoc
./baza.sh przod 150 2          # jedz do przodu 150 mm/s przez 2 s (ok. 30 cm)
./baza.sh tyl 100 1            # do tylu 100 mm/s przez 1 s (ok. 10 cm)
./baza.sh lewo 200 1           # obrot w lewo 200 mrad/s przez 1 s (ok. 11,5 st.)
./baza.sh prawo 200 1          # obrot w prawo
./baza.sh wasd                 # prowadzenie z klawiatury: W/S/A/D, SPACJA=stop, X=wyjscie
./baza.sh stop                 # natychmiast: zera + oddanie sterowania
./baza.sh stan 5               # podglad stanu bazy przez 5 s
./baza.sh podglad lewo 100 2   # NIC nie wysyla, tylko pokazuje, co by poszlo (--dry-run)
```

* Każda komenda **sama** dokłada ramki `op=6`/`op=111` przed jazdą i `zera + op=7` po jeździe,
  więc nie musisz o tym pamiętać.
* **`wasd` trzyma sterowanie przez całą sesję** i oddaje je (`op=7`) dopiero przy wyjściu;
  w trakcie puszczenia klawisza lecą same zera. (Panel robi dokładnie to samo — jeden `op=7`
  na końcu sesji. Wcześniejsza wersja oddawała sterowanie po każdym puszczeniu klawisza i baza
  przestawała reagować — patrz rozdział o `amr_wasd`.)
* Wartości liczbowe to **mm/s** (jazda) i **mrad/s** (obrót; 1000 mrad/s = 57,3 st./s).
  Bezpieczne widełki na halę: jazda 100–300, obrót 100–300.
* `wasd` domyślnie: 150 mm/s i 200 mrad/s; można zmienić: `./baza.sh wasd --speed 250 --turn 150`.
* `podglad` działa też bez robota (nic nie łączy) — tym sprawdzisz każdą komendę przed wysłaniem.
* Nadpisać IP/port (np. na atrapę): `AMR_IP=127.0.0.1 AMR_PORT=5003 ./baza.sh przod 100 1`.

Warunki przed jazdą: **wolna przestrzeń ≥ 1 m, fizyczny STOP w ręce, karta panelu Matrix zamknięta**
(dwie sesje naraz nie powinny sterować bazą).

### Co skrypt robi pod spodem (gołe komendy)

```bash
# podglad:
./amr_cmd --ip 192.168.71.50 \
  --pre-frame @cap/panel_op6_enable.hex --pre-frame @cap/panel_op111.hex \
  --from-capture cap/panel_paste.log --type 16 --nth 1 \
  --set 6.3=50 --set 6.4=0 \
  --stop-frame @cap/panel_stop_zero.hex --stop-frame @cap/panel_op7_release.hex --dry-run

# ruch: to samo bez --dry-run; wasd:
./amr_wasd --ip 192.168.71.50 \
  --pre-frame @cap/panel_op6_enable.hex --pre-frame @cap/panel_op111.hex \
  --from-capture cap/panel_paste.log --type 16 --nth 1 \
  --set 6.3=@@vx@@ --set 6.4=@@wz@@ \
  --stop-frame @cap/panel_stop_zero.hex --stop-frame @cap/panel_op7_release.hex
```

Pola są potwierdzone: **`6.3` = jazda, + = przód; `6.4` = obrót, + = w lewo, − = w prawo**.
Program sam mierzy i drukuje **dx, dy, dkąt** z rzeczywistego stanu bazy.

**Kalibracja — zrobiona (wynik):** komenda 50 → 95 mm w 2 s, komenda 100 → 188 mm w 2 s;
w fazie stałej dokładnie 50 i 103 mm/s. **1 jednostka = 1 mm/s, zależność liniowa.** Różnica
w dystansie to rampa rozruchu bazy (~0,2 s), więc krótkie ruchy wypadają odrobinę krótsze niż
„prędkość × czas". **Obrót czeka na potwierdzenie:** `./baza.sh lewo 200 1` powinien dać
`dkat ≈ +0,2 rad` (≈ 11,5°) — jeśli wyjdzie inna liczba, poprawimy `--turn` w `wasd`.

Jak czytać wynik pierwszego testu:

* pojechał do przodu → wszystko się zgadza;
* **nie ruszył się** → baza odrzuca komendy z obcej sesji; wtedy: (a) spróbuj z otwartą kartą panelu,
  (b) spróbuj bez ramek wstępnych, (c) uruchom z `--echo-state` i wyślij mi log.

**Do sprawdzenia na hali (po kolei, małymi krokami):**

1. **Obrót:** `./baza.sh lewo 200 1`, potem `./baza.sh prawo 200 1` — czy `dkat` ma właściwy znak
   i wartość (oczekiwane ±0,2 rad).
2. **Obie osie naraz** (tego nie było w zrzucie z panelu — jog łączył osie pojedynczo):
   `./amr_wasd` i jazda z zakręcaniem. Jeśli baza przyjmie sumę `6.3`+`6.4` — skręcanie działa.
3. **Zasięg:** przytrzymaj W w `wasd` i sprawdź, że po puszczeniu robot staje w ~0,35 s (`--hold-ms`).

## 5. Narzędzia — użycie

### `amr_state_cli` — tylko odczyt (Wasze, rozszerzone)
```bash
./amr_state_cli --ip 192.168.71.50                    # linia przy każdej zmianie stanu
./amr_state_cli --seconds 30 --all --segments --full   # każdy push + odcinki trasy + pełne drzewo
```

### `amr_probe` — podglądacz i próbnik
Pokazuje każdą ramkę z bazy (koperta + drzewo pól), dla powtarzanych typów drukuje **tylko różnice**,
a dla pushu stanu od razu x/y/kąt/dystans/faza. Potrafi też wysłać zadanie:

```bash
./amr_probe --ip 192.168.71.50                          # nasłuch
./amr_probe --type 17 --seconds 5                        # wyślij zadanie typu 17 i patrz, co wróci
./amr_probe --type 100 --set 2.1=200 --repeat 40 --rate 20   # pętla (przykład protokołu ATROPY)
./amr_probe --interactive                                # komendy: type 17 3.1=1 | hex <HEX> | dump <HEX> | q
./amr_probe --dump-hex 08021565...                       # dekodowanie offline (np. z cap.log)
```
Konwencja `--set`: **ścieżka w całym body** (razem z kopertą), typy: `v` varint (domyślnie, też
ujemne), `u`, `z` zigzag, `s` tekst, `f` float32, `d` float64. Przykłady: `6.3=150`, `1=17`,
`10:s=mapa1`, `6.1:z=-5`.

### `tools/analyze_capture.py` — rozkładanie zrzutu na grupy

```bash
python3 tools/analyze_capture.py cap/panel_paste.log                 # grupy + serie wartości
python3 tools/analyze_capture.py cap/panel_paste.log --timeline 2    # pełna oś czasu grupy nr 2
```
Grupuje ramki po zestawie obecnych pól (niezależnie od wartości), a potem pokazuje, co w każdej
grupie się zmienia i w jakich przedziałach czasu — dzięki temu z jednego zrzutu widać np.
„pole 3 = 330 przez 2,7 s, potem pole 4 = 266, potem pole 4 = -266".

### `amr_proxy` — przechwytywanie ruchu panelu
```bash
./amr_proxy --listen 0.0.0.0:5002 --remote 192.168.71.50:5002 --save cap/krok1.log
```
* ramki z panelu (`UP`) zawsze w całości + różnice względem poprzedniej tego samego typu,
* ramki z bazy (`DN`): push stanu skrócony do jednej zmieniającej się linijki, reszta z nagłówkami
  (`--down-full` = pełne drzewo),
* `--save` zapisuje plik w formacie `+<czas> UP|DN <hex>` (+ linie `#` z opisem) — **bezpośrednio
  czytelny dla `amr_cmd --from-capture`**,
* Ctrl-C kończy i drukuje tabelę liczby ramek per typ.

### `amr_cmd` — sterowanie
```bash
# 1) pojedyncza ramka (np. „jedź do stacji”): wyślij raz, przytrzymaj --hold, potem ramka stopu
./amr_cmd --ip 192.168.71.50 --frame <HEX> --seconds 0 --stop-frame <HEX_STOPU>

# 2) pętla (jog): 20 ramek, 20 Hz + STOP
./amr_cmd --ip 192.168.71.50 --frame <HEX> --repeat 20 --rate 20 --stop-frame <HEX_STOPU>

# 3) z zapisu proxy: pierwsza ramka typu 17 z kierunku UP
./amr_cmd --ip 192.168.71.50 --from-capture cap/krok1.log --type 17 --nth 1 --repeat 1 --seconds 0 --stop-frame <HEX>

# 4) test bez wysyłania: pokaże ramki i różnice po --set
./amr_cmd --frame <HEX> --set 4.2.2=120 --dry-run
```
Dodatkowo (od czasu zdobycia zrzutu z panelu):
* `--pre-frame HEX|@plik` — ramki wysyłane **raz przed** ruchem (np. `op=6` „przejmij sterowanie"
  i `op=111`); można podać kilka,
* `--stop-frame HEX|@plik` — **można podać kilka** (np. najpierw zerowe prędkości, potem `op=7`
  „oddaj sterowanie"); każda leci 2×. W `amr_cmd` to poprawne, bo każda komenda to jedna,
  zamknięta sesja ruchu. **W `amr_wasd` oddanie sterowania ma osobną opcję `--release-frame`** —
  inaczej baza przestałaby reagować po pierwszym puszczeniu klawisza,
* `--from-capture` rozumie już także kanał komend: `--type 16` = jog, `--type 6/7/111/32` = ramki
  sterowania z panelu (`op=` w nagłówku ramki).

Zabezpieczenia: 3-sekundowe odliczanie (pomijane `--yes`), ograniczenie `--rate` do 50 Hz, limit
czasu w pętli, ramka stopu na koniec i przy Ctrl-C, podsumowanie przemieszczenia.
**Ramka stopu jest bardzo zalecana** — przy niektórych komendach baza może jechać dalej po
zerwaniu połączenia (to samo robi panel: wysyła „puszczenie" przycisku).

### `amr_wasd` — prowadzenie bazy z klawiatury (W/S/A/D)

```bash
# na atrapie (własny protokół atrapy):
./amr_wasd --ip 127.0.0.1 --port 5003 --type 100 --set 2.1=@@vx@@ --set 2.2=@@wz@@ --stop-type 102

# najprosciej: ./baza.sh wasd

# albo recznie (prawdziwa baza): ramka jogu z przechwyconego panelu (kanal 6, op=16)
./amr_wasd --ip 192.168.71.50 \
           --pre-frame @cap/panel_op6_enable.hex --pre-frame @cap/panel_op111.hex \
           --from-capture cap/panel_paste.log --type 16 --nth 1 \
           --set 6.3=@@vx@@ --set 6.4=@@wz@@ \
           --stop-frame @cap/panel_stop_zero.hex \
           --release-frame @cap/panel_op7_release.hex \
           --speed 150 --turn 200
```

* **W/S** = przód/tył, **A/D** = obrót, **SPACJA** = natychmiastowy stop, **X** = wyjście,
  strzałki też działają.
* Terminal nie przekazuje „puszczenia" klawisza, więc działa okno `--hold-ms` (domyślnie 350 ms):
  każde wciśnięcie (w tym autopowtarzanie przy trzymaniu) je odświeża; gdy nic nie przychodzi,
  prędkość spada do zera i leci **ramka stopu**. *Trzymasz = jedzie, puszczasz = staje.*
* Kolejka bezpieczeństwa: `--accel` (łagodne narastanie), `--max-seconds`, stop przy Ctrl-C
  i przy zamknięciu połączenia, `--dry-run` do testu bez wysyłania.
* **`--stop-frame` = chwilowy stop** (puszczenie klawisza, SPACJA) — ma zawierać tylko
  wyzerowanie prędkości. **`--release-frame` = oddanie sterowania** (`op=7`, można kilka) —
  leci wyłącznie przy wyjściu z programu.
* **`--no-rearm`** wyłącza automatyczne powtarzanie ramek `--pre-frame` przed każdym nowym ruchem
  (domyślnie włączone: po każdej pauzie `amr_wasd` przejmuje sterowanie jeszcze raz, więc nawet
  gdyby baza je w międzyczasie zabrała, kolejny ruch zadziała).
* **`--verbose`** pokazuje ramki przychodzące z bazy (odpowiedzi, nie push stanu) — przydatne,
  gdy trzeba zobaczyć, czy baza odrzuca nasze komendy.

> **Dlaczego wcześniej „działało przez chwilę":** `wasd` wysyłał po każdym puszczeniu klawisza
> ramkę `op=7`, czyli *oddawał sterowanie*. Wyglądało to jak awaria bazy, a to my sami odbieraliśmy
> sobie prawo do jazdy. Panel Matrix wysyła `op=7` tylko raz, na końcu sesji — i tak teraz robi
> `baza.sh wasd` (`--stop-frame` = zera, `--release-frame` = `op=7` przy wyjściu).
* Ścieżki `--set`: przy `--type` względem treści zadania (`2.1`), przy `--frame`/`--from-capture`
  względem całego body (`6.3`) — tak samo jak w `amr_cmd`.
* HUD na żywo: aktualne vx/wz oraz x, y, kąt i faza bazy.

Test bez człowieka (przydatny, gdy chcesz sprawdzić łańcuch przed wejściem na halę):

```bash
python3 tools/test_wasd_pty.py --keys w=1.5,d=1.0 -- ./amr_wasd --ip 127.0.0.1 --port 5003 \
    --type 100 --set 2.1=@@vx@@ --set 2.2=@@wz@@ --stop-type 102
```

---

## 6. Ławka testowa (bez robota)

`tools/bench_server.py` to atrapa bazy: przyjmuje logowanie, przydziela sesję, wysyła push stanu
co 200 ms, potwierdza każde zadanie i ma **własny, wymyślony protokół ruchu** (tylko do testów):

| typ | treść (pole 2) | znaczenie |
|---|---|---|
| 100 | `{1: vx mm/s, 2: wz mrad/s}` | jog (wygasa po 0,5 s bez powtórzeń) |
| 101 | `{1: x mm, 2: y mm, 3: kąt mrad}` | jedź do punktu |
| 102 | — | STOP |

```bash
python3 tools/bench_server.py --port 5003 --auto-mission   # + odgrywa trasę 1→3 (y: -22 → -1532 mm)
```

**Przetestowane u mnie na tej atrapie (cały łańcuch działa):**
* `amr_state_cli` czyta stan i odcinki trasy: faza 2 → 10 → 7 → 2, dystans 151 → 1 cm, odcinek
  `(0, -22) -> (0, -1532) mm` — czyli atrapa odtwarza dokładnie Waszą obserwację;
* `amr_cmd --type 100 --set 2.1=200 --set 2.2=0 --seconds 2 --rate 20 --stop-type 102`
  → wysłano 40 ramek + STOP, przemieszczenie **391 mm** (≈ 200 mm/s) — komenda działa;
* klient → `amr_proxy` → atrapa: jog z obrotem (`--set 2.1=250 --set 2.2=150`)
  → 358 mm i 12,3° obrotu, a w `cap/bench.log` (przykład) wylądowały ramki gotowe do odtworzenia;
* **odtworzenie z zapisu**: `--from-capture cap/bench.log --type 100 --nth 1 --repeat 20`
  → znowu ruch (236 mm, 8,2°) — to jest dokładnie obieg, który będzie potrzebny na prawdziwej bazie;
* **`amr_wasd` (WASD)**: trzymanie „w" 1,2 s przy 250 mm/s → ~390 mm jazdy; „a" → +0,45 rad obrotu;
  puszczenie → ramka stopu (faza 2); ten sam efekt z ramki z pliku (`--frame @cap/jog_type100.hex`)
  i z zapisu (`--from-capture`);
* **haczyk przeglądarkowy** (`tools/panel_hook.js`): test `node tools/test_panel_hook.js` potwierdza,
  że ramki są poprawnie sklejane z fragmentów WebSocket i wychodzą w formacie `+<czas> UP|DN <hex>`.

---

## 7. Bezpieczeństwo (przeczytaj przed pierwszym ruchem)

**Sprawdzone na prawdziwej bazie (7.10.2026):** op=6/op=111 przed jazdą i zera + op=7 po jeździe
są przyjmowane, a komenda prędkości działa 1:1 (mm/s). **Nie wiemy jeszcze**, czy nasza sesja
dostaje od bazy „martwego człowieka" (panel ma: urwany strumień jogu = baza staje) — dlatego
`baza.sh` ZAWSZE wysyła jawne ramki zatrzymania (zera + op=7) na koniec każdego ruchu, na puszczenie
klawisza, na SPACJĘ i przy Ctrl-C.

* Robot ma ~190 kg i 1,7 m — **fizyczny STOP w zasięgu ręki**, przestrzeń wokół wolna.
* Pierwsze próby: **mała prędkość i krótki czas** (np. 1 ramka, `--seconds 0`, `--hold 0.3`).
* Zawsze podawaj **ramkę stopu** (`--stop-frame` / `--stop-type`), jeśli tylko udało się ją złapać.
* Miej otwartą drogę ucieczki od bazy; nie stawaj między bazą a ścianą.
* Jeśli baza odmówi wykonania komendy (np. brak lokalizacji), zobaczysz to w fazie/dystansie —
  ale serwer wykonuje własne sprawdzenia bezpieczeństwa, nasz program ich nie omija.
* Panel Matrix i nasz program mogą ze sobą konkurować o sesję — do testów ruchu **zamknij panel**,
  a proxy używaj w osobnych przebiegach.

---

## 8. Co dalej (propozycje)

1. ~~Złapać realne komendy panelu~~ **zrobione** (`cap/panel_paste.log`, kanał 6, jog = op 16).
2. ~~Pierwszy ruch z naszego kodu~~ **zrobione** (7.10.2026, 1:1 mm/s, ramki op=6/111/7 przyjęte).
2b. Potwierdzić obrót (`./baza.sh lewo 200 1` → oczekiwane `dkat ≈ +0,2 rad`) i jazdę z zakręcaniem.
2a. Rozgryźć `op=32` (komenda zadania, odpowiedź bazy type=11) — to prawdopodobnie „jedź do stacji".
   Przydałby się zrzut z etykietami: co dokładnie kliknąłeś przy każdej ramce.
3. Node ROS 2: publikacja stanu (`nav_msgs/Odometry`, `tf`) i subskrypcja `/cmd_vel`
   → nadbudowa nad `amr_matrix.hpp` (kilkadziesiąt linii, gdy znamy komendy ruchu).
4. Sekwencje: kolejka „jedź do stacji N, poczekaj na fazę 2" z użyciem `dist`/`phase`.
5. Potwierdzić w jednym przebiegu: jednostkę dystansu (`3.7.8`), kolejność pól w `3.7.12`,
   znaczenie typów 1/17/4 i to, czy baza przyjmuje dwie sesje równolegle.

---

## 9. Ściągawka z opcji

| narzędzie | najważniejsze opcje |
|---|---|
| `amr_state_cli` | `--ip --port --user --pass-md5 --seconds --all --segments --full` |
| `amr_probe` | `--type N --set --repeat --rate --hex --interactive --dump-hex --dry-run --all` |
| `amr_proxy` | `--listen H:P --remote H:P --save PLIK --down-full --no-up-full` |
| `baza.sh` | `wasd \| przod v s \| tyl v s \| lewo w s \| prawo w s \| stop \| stan s \| podglad ...` — najprostsze wejscie |
| `amr_cmd` | `--frame HEX\|@plik --type N --set --from-capture --dir --nth --seconds --repeat --rate --hold --pre-frame (wielokrotny) --stop-frame (wielokrotny) --stop-type --dry-run --yes --echo-state` |
| `amr_wasd` | `--frame --from-capture --type --set a.b=@@vx@@ --set a.b=@@wz@@ --pre-frame --stop-frame --release-frame --no-rearm --verbose --stop-type --speed --turn --rate --accel --hold-ms --max-seconds --dry-run` |

Domyślne dane bazy: `192.168.71.50:5002`, użytkownik `admin`, MD5 hasła `admin`.

Kanał komend w pigułce: `kind=2`, pole 6, `{2: opkod}` — jog `op=16` (`{3: jazda, 4: obrót}`);
sterowanie: `op=6`/`op=111` przed, `op=7` po. Heartbeat: `kind=0`, pole 4, `{1: 18}` co 3 s.
