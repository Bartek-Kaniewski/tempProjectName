# Protokół serwera bazy Helios (Matrix OS)
### Co można przechwycić i co można wysłać — opis interfejsu sieciowego

Dokument opisuje **wyłącznie serwer** (bazę jezdną) i jego interfejs sieciowy: co wysyła,
co przyjmuje, jak to zdjąć z sieci i jak zbudować własną komendę. Nie opisuje żadnego
konkretnego klienta ani narzędzia.

Oznaczenia statusu:

| Znaczenie |
|---|
| ✅ potwierdzone bezpośrednio na robocie |
| 🔎 ustalone z obserwacji ruchu panelu |
| ❓ hipoteza / nieustalone |

Stan wiedzy: październik 2026.

---

## 1. Punkt dostępu

| Element | Wartość |
|---|---|
| Adres usługi | `ws://192.168.71.50:5002` (WebSocket, ramki **binarne**) |
| Panel webowy (klient oryginalny) | `http://192.168.71.50/#/homePage` |
| Konto domyślne | login `admin`, hasło `admin` (przesyłane jako MD5 w hex: `21232f297a57a5a743894a0e4a801fc3`) |
| Szyfrowanie | brak (zwykłe `ws://`, bez TLS) — ✅ |
| Inne porty | nie zaobserwowano (5002 obsługuje całą komunikację) |

Baza jest **osobnym systemem** od ramion robota: ma własny serwer, panel i protokół.
Interfejs opisany niżej dotyczy wyłącznie bazy jezdnej.

---

## 2. Warstwa transportowa

### 2.1 Ramkowanie

Każda wiadomość aplikacji, w obie strony, ma postać:

```
[ 4 bajty: długość N, u32 big-endian ][ N bajtów: komunikat protobuf ]
```

Część protobufowa (bez prefiksu) to dalej **„treść"** — w tej postaci najwygodniej zapisywać
przechwycone ramki.

### 2.2 Kodowanie

| Typ drutu | Znaczenie |
|---|---|
| 0 | varint |
| 1 | fixed64 (8 B, little-endian) |
| 2 | długość + bajty (zagnieżdżony komunikat / tekst) |
| 5 | fixed32 (4 B, little-endian) |

Uwagi praktyczne:
- wartości liczbowe są **całkowite** (floatów nie zaobserwowano);
- **liczby ujemne** (np. −266) zapisane jako int64 zajmują 10 bajtów — wyglądają jak ogromna
  liczba (`18446744073709551350`). Zanim uznasz to za błąd, sprawdź interpretację ze znakiem.

---

## 3. Koperta

Każda ramka, niezależnie od treści, ma wspólną „kopertę":

| Pole | Typ | Nazwa | Znaczenie |
|---|---|---|---|
| 1 | varint | `kind` | **0** = żądanie, **1** = odpowiedź, **2** = push/komenda |
| 2 | fixed32 | `seq` | numer sekwencji; klient nadaje własne, serwer odsyła je w odpowiedzi |
| 3 | fixed64 | `session` | identyfikator sesji nadany przez serwer (**0** przed zalogowaniem) |
| 4 | bytes | – | treść żądania/pushu (podpole 1 = typ komunikatu) |
| 5 | bytes | – | treść odpowiedzi |
| 6 | bytes | – | **kanał komend** (podpole 2 = opkod) — patrz rozdz. 6 |

Przykładowy opis ramki (skrót używany w dalszej części dokumentu):
`kind=2 seq=19 sess=0x1a11642c431 op=16 len=27`

Reguła: **`seq` i `session` nadpisuje klient**. Budując własną ramkę z przechwyconej, podmienia się
tylko te dwa pola; reszta zostaje bez zmian.

---

## 4. Uwierzytelnianie i utrzymanie sesji

### 4.1 Logowanie

Pierwsza ramka od klienta: `kind=0`, `seq=1`, `session=0`, treść w polu 4:

```
pole 4 = { 1: 0, 2: { 1: "<login>", 2: "<md5-hasła-hex>", 3: 30 } }
```

| Ścieżka | Znaczenie |
|---|---|
| `4.1` | kod żądania: **0** = logowanie |
| `4.2.1` | login |
| `4.2.2` | MD5 hasła (32 znaki hex) |
| `4.2.3` | `30` — nieustalone ❓ (stałe w panelu) |

Odpowiedź serwera niesie **sesję** (pole 3 koperty). Od tego momentu każda wysyłana ramka musi
ją zawierać — bez sesji serwer nie przyjmuje niczego.

### 4.2 Po zalogowaniu

Obserwowany klient (panel) wysyła kolejno trzy żądania o typach: **1**, **17**, **4** 🔎
(znaczenie każdego nieustalone ❓; prawdopodobnie mapa/konfiguracja/stan).

### 4.3 Podtrzymanie sesji

| Aspekt | Wartość |
|---|---|
| żądanie | `kind=0`, pole 4 = `{ 1: 18 }` |
| częstotliwość | co ~3 s 🔎 |

### 4.4 Znane kody żądań (klient → serwer)

| Typ | Znaczenie | Status |
|---|---|---|
| 0 | logowanie | ✅ |
| 1 | nieustalone (bootstrap) | 🔎 ❓ |
| 4 | nieustalone (bootstrap) | 🔎 ❓ |
| 17 | nieustalone (bootstrap) | 🔎 ❓ |
| 18 | heartbeat | 🔎 |

---

## 5. Co serwer nadaje (do przechwycenia)

### 5.1 Push stanu (typ 2)

Serwer sam, cyklicznie, wysyła stan bazy jako treść w polu 5 (kind=1/2). Trafia też do odpowiedzi
na logowanie. Znane pola:

| Ścieżka | Jednostka | Znaczenie | Status |
|---|---|---|---|
| `3.2` | – | faza: **2** = spoczynek, **7** = jazda, **10** = start | ✅ |
| `3.4.2` | mm | pozycja x | ✅ |
| `3.4.3` | mm | pozycja y | ✅ |
| `3.4.7` | mrad | kąt (yaw) | ✅ |
| `3.7.8` | 10 mm | pozostały dystans do celu (1 jednostka = 1 cm) | 🔎 |
| `3.7.12[n]` | mm | odcinek trasy: (sx, sy, ex, ey) | 🔎 ❓ (kolejność pól) |
| `3.10` | tekst | nazwa mapy (np. `Showroom`) | ✅ |

### 5.2 Odpowiedzi i potwierdzenia

| Ramka | Znaczenie | Status |
|---|---|---|
| `kind=1`, `5.1 = 11` | odpowiedź na zlecenie zadania (rozdz. 6.4), po ~8,5 s | 🔎 |
| `kind=1`, `5.14.1=1`, `5.14.2=0` | dodatkowe pola tej odpowiedzi — nieustalone | ❓ |

---

## 6. Co można wysłać

### 6.1 Kanał komend (pole 6)

Komendy sterujące wysyła się ramkami `kind=2`, z treścią w **polu 6**:

```
pole 6 = { 2: <opkod>, 7: 0, <parametry…> }
```

| Opkod | Nazwa | Parametry | Status |
|---|---|---|---|
| **16** | jog (strumień prędkości) | pole 3 = jazda, pole 4 = obrót | ✅ |
| **6** | przejmij sterowanie | pole 7 = 0 | ✅ (skutek) |
| **111** | nieznane, wysyłane razem z `op=6` | pole 7 = 0 | 🔎 ❓ |
| **7** | oddaj sterowanie | pole 7 = 0 | ✅ (skutek) |
| **32** | zlecenie zadania (jazda do celu) | `9: {3:2, 4:2, 5:03, 11:1}` | 🔎 ❓ (adresowanie) |

### 6.2 Jog (`op=16`)

| Ścieżka | Jednostka | Znaczenie |
|---|---|---|
| `6.3` | mm/s | jazda: **+ przód**, − tył |
| `6.4` | mrad/s | obrót: **+ w lewo**, − w prawo (1000 ≈ 57,3°/s) |
| `6.7` | – | `0` (stałe we wszystkich zaobserwowanych ramkach) |

Kalibracja: wartość **1:1** — komenda `100` daje ~103 mm/s w fazie ustalonej ✅.
Baza ma własną **rampę rozruchu ~0,2 s**, więc krótkie ruchy wypadają nieco krótsze niż
„prędkość × czas".

Obserwowana częstotliwość strumienia: **~5 Hz** (panel) 🔎; wyższe częstotliwości nie były
testowane.

### 6.3 Kolejność ramek (ważne)

```
op=6 + op=111   →  jog (op=16) w pętli  →  jog z zerami  →  op=7
   przejęcie           jazda                zatrzymanie      oddanie
```

| Zasada | Skutek |
|---|---|
| `op=6`+`op=111` **przed** jazdą | bez tego jog jest ignorowany |
| **`op=7` kończy prawo do jazdy** | po `op=7` kolejne jogi nie działają, dopóki nie wyśle się znowu `op=6`+`op=111` |
| serwer nie wysyła własnej ramki „stop" przy przerwaniu strumienia | urwanie jogu samo w sobie wygląda na zatrzymanie (deadman) — 🔎, nie wiadomo, czy dotyczy każdej sesji ❓ |
| brak jawnego stopu | warto wysyłać jog z zerami przed zakończeniem (bezpieczniej niż polegać na deadmanie) |

### 6.4 Zadanie (`op=32`)

```
pole 6 = { 2: 32, 7: 0, 9: { 3: 2, 4: 2, 5: 0x03, 11: 1 } }
```

Po ~8,5 s serwer odpowiada ramką `kind=1`, `5.1 = 11`. Znaczenie pól `9.3/9.4/9.5/9.11`
(wskazanie celu/trasy) — **nieustalone** ❓.

### 6.5 Przykładowe ramki (hex = treść bez prefiksu długości)

| Co | hex |
|---|---|
| jog: przód 330 mm/s | `080215130000001931c44216a10100003209101018ca0220003800` |
| jog: obrót w lewo 266 mrad/s | `0802151f0000001931c44216a1010000320910101800208a023800` |
| jog: obrót w prawo 266 mrad/s | `080215450000001931c44216a101000032111010180020f6fdffffffffffffff013800` |
| jog: zera (zatrzymanie) | `0802150000000019000000000000000032081010180020003800` |
| przejmij sterowanie (`op=6`) | `08021500000000190000000000000000320410063800` |
| `op=111` (para z `op=6`) | `080215000000001900000000000000003204106f3800` |
| oddaj sterowanie (`op=7`) | `08021500000000190000000000000000320410073800` |
| zlecenie zadania (`op=32`) | `0802156b0000001931c44216a1010000320f102038004a09180220022a01035801` |
| odpowiedź na zadanie (`kind=1`) | `0801156b0000001931c44216a10100002a08080b720408011000` |

Uwaga: w ramkach do wysłania pola `seq` (pole 2) i `session` (pole 3) ustawia się na własne
wartości — w powyższych przykładach są zerowe.

---

## 7. Jak przechwycić ruch

Cel: zobaczyć, **co dokładnie** wysyła oryginalny klient (panel), żeby potem powtórzyć to samo.
Trzy podejścia, od najprostszego:

### 7.1 Konsola przeglądarki (bez uprawnień administratora)

1. Otwórz panel i zaloguj się.
2. DevTools (F12) → **Network** → filtr **WS** → wybierz połączenie do portu 5002 →
   zakładka **Messages**: widać wszystkie ramki w obie strony (binarne, w hex).
3. Dodatkowo z konsoli można podpiąć się do gniazda (np. nadpisując `WebSocket.prototype.send`),
   żeby zbierać ramki z etykietami czasu i zapisać je do pliku.

Co daje: komendy wychodzące **oraz** przychodzące, z czasem. Ograniczenia: trzeba wkleić kod
w konsoli i nie przeładowywać strony; nasłuch samych odpowiedzi wymaga świeżego połączenia.

### 7.2 Proxy (pełny, dwukierunkowy zapis)

Wstaw pośrednika między panel a serwer (panel kierujesz na swój komputer, pośrednik przekazuje
dalej). Zapis dostajesz po obu stronach, bez modyfikowania panelu.
Wymaga uprawnień administratora i ostrożności, żeby pośrednik nie złapał własnego ruchu
(np. przekierowanie tylko dla procesu przeglądarki).

### 7.3 Własny klient

Znając kopertę i logowanie, można po prostu **wysyłać własne ramki** i słuchać odpowiedzi.
Najprostsze do rozpoznania protokołu: wysłać przechwyconą komendę **bajt w bajt** (podmieniając
tylko `seq` i `session`), potem stopniowo zmieniać pojedyncze pola.

### 7.4 Co dokładnie zapisywać

- **treść bez prefiksu długości** (4-bajtowy nagłówek długości odrzucamy) — inaczej ramki się nie
  dopasują przy odtwarzaniu;
- kierunek (**wysyłane** vs **odbierane**) i czas;
- **co robiono w danym momencie** (dziennik działań) — bez tego zrzut jest bezużyteczny;
- wartości `seq` i `session` są techniczne — przy analizie można je pominąć.

---

## 8. Bezpieczeństwo i ograniczenia wynikające z protokołu

1. **`op=7` jest nieodwracalne** dla danej chwili — kończy prawo do jogu. Nie wysyłać go między
   ruchami.
2. **Zawsze kończyć jogiem z zerami** — nie polegać na tym, że serwer sam zatrzyma bazę.
3. **Jedna sesja sterująca**: nie wiadomo, jak serwer rozstrzyga sterowanie równoczesne
   (panel + klient) ❓ — bezpiecznie zakładać konflikt.
4. **Rampa rozruchu** bazy (~0,2 s) — krótkie ruchy są krótsze niż wynika to z iloczynu
   prędkości i czasu.
5. `session` i `seq`: `session` nadaje serwer i jest wymagana w każdej ramce; `seq` może się
   powtarzać — nie jest to klucz sesji.
6. Brak TLS: cały ruch (w tym hash hasła) idzie otwartym tekstem w sieci lokalnej.

---

## 9. Czego serwer jeszcze przed nami skrywa

| Element | Status | Jak ustalić |
|---|---|---|
| `op=111` — znaczenie | ❓ | wysłać jog bez tego opkodu i porównać zachowanie |
| `op=32` — adresowanie celu (pola `9.3/9.4/9.5/9.11`) | ❓ | przechwycić zlecenie kilku różnych stacji z opisem, co klikał operator |
| Deadman: czy urwanie jogu zawsze zatrzymuje bazę | 🔎 | przerwać strumień w połowie i obserwować |
| Typy żądań 1 / 4 / 17 | ❓ | porównać odpowiedzi z wysyłką i bez |
| Kolejność pól odcinka trasy (`3.7.12[n]`) | ❓ | porównać trasę widoczną w panelu z ramkami stanu |
| Jednostka pozostałego dystansu (`3.7.8`) | 🔎 | skorelować z faktycznie przejechanym dystansem |
| Limity przyspieszenia i prędkości bazy | ❓ | seria prób z narastającymi wartościami |
| Znaczenie `4.2.3 = 30` w logowaniu | ❓ | próby logowania z inną wartością |

---

*Opis interfejsu sieciowego bazy jezdnej Helios (system Matrix OS). Wszystkie ustalenia dotyczą
wyłącznie warstwy komunikacji: adresacji, koperty, sesji, kanału komend i danych stanu.*
