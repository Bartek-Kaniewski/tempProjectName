// amr_cmd - "ster" do bazy Helios (Matrix OS): wysyla komendy ruchu i mierzy efekt.
//
// Idea: komend ruchu bazy NIE znamy z dokumentacji, ale znamy je z ruchu panelu Matrix.
// Ten program potrafi odtworzyc przechwycona ramke panelu (podmieniajac tylko numer sekwencji
// i sesje) oraz zmienic w niej pojedyncze pola (np. predkosc). Dzieki temu "przod" z panelu
// staje sie komenda z naszego programu, bez zgadywania formatu.
//
// Trzy tryby:
//  1) --frame <hex|@plik>            odtworz jedna ramke (np. "jedz do stacji 3")
//  2) --frame ... --repeat N|--seconds S --rate H   wysylaj w petli (np. jog przytrzymany)
//  3) --type N --set a.b=v           zbuduj zadanie od zera (do eksperymentow)
// Dodatkowo --stop-frame / --stop-type: ramka, ktora leci na koniec (odpowiednik puszczenia
// przycisku / STOP w panelu). Bez tego przy niektorych komendach baza moze jechac dalej!
//
// Budowanie: make
//
// Przyklady:
//   # 1. Zobacz, co jest w przechwyconej ramce i co sie zmieni po nadpisaniach (nic nie wysyla):
//   ./amr_cmd --frame @cap/krok1.log.hex --set 4.2.2=150 --dry-run
//
//   # 2. Jedz do przodu przez 2 s (ramka jog z przechwycenia, 20 Hz, na koniec ramka stopu):
//   ./amr_cmd --ip 192.168.71.50 --frame <hex> --rate 20 --seconds 2 --stop-frame <hex>
//
//   # 3. Wybierz ramke z zapisu proxy (linia UP, typ 17, pierwsze wystapienie):
//   ./amr_cmd --from-capture cap/krok1.log --type 17 --nth 1 --dry-run
//
// Wynik: po zakonczeniu drukuje przemieszczenie (dx, dy, dkat) - od razu widac, czy zadzialalo.
#include <poll.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "amr_matrix.hpp"

namespace {

volatile sig_atomic_t g_stop = 0;
void onSigint(int) { g_stop = 1; }

struct Args {
  amr::MatrixClient::Options opt;
  int type = -1;
  std::string frame;
  std::vector<std::string> pre_frames;    // ramki wstepne (np. "przejmij sterowanie" op=6, op=111)
  int stop_type = -1;
  std::vector<std::string> stop_frames;
  std::string capture;
  std::string capture_dir = "UP";
  int capture_type = -1;
  int nth = 1;
  double seconds = 0;
  int repeat = 0;
  double rate = 20;
  double hold = 1.0;
  bool seconds_set = false;
  bool dry_run = false;
  bool yes = false;
  bool echo_state = false;
  std::vector<std::string> sets;
  std::vector<std::string> stop_sets;
};

void usage() {
  std::printf(
      "amr_cmd %s - wysylanie komend ruchu do bazy Helios (Matrix OS)\n"
      "\n"
      "Ramki wstepne (opcjonalne, wysylane raz przed komenda):\n"
      "  --pre-frame HEX|@plik  np. 'przejmij sterowanie' z panelu (op=6) i op=111; mozna powtarzac\n"
      "\n"
      "Zrodlo komendy (jedno z):\n"
      "  --frame HEX | --frame @plik     gotowa ramka (hex calego body) - np. z amr_proxy --save\n"
      "  --type N [--set a.b=v ...]      zadanie zbudowane od zera (--set: sciezka wzgledna do tresci)\n"
      "  --from-capture PLIK [--type N] [--nth K]   wybierz ramke z zapisu proxy (linie \"UP\")\n"
      "\n"
      "Sposob wysylki:\n"
      "  --seconds S        wysylaj przez S sekund (domyslnie 2; 0 = pojedyncza ramka)\n"
      "  --repeat N         wyslij dokladnie N razy (nadrzedne nad --seconds)\n"
      "  --rate H           czestotliwosc w Hz (domyslnie 20)\n"
      "  --hold S           gdy pojedyncza ramka: ile sekund czekac przed ramka stopu (domyslnie 1)\n"
      "  --stop-frame HEX|@plik / --stop-type M [--stop-set ...]   ramka zakonczenia (STOP/puszczenie)\n"
      "\n"
      "Pozostale:\n"
      "  --ip / --port / --user / --pass-md5   dane bazy (domyslnie jak w panelu: 192.168.71.50:5002)\n"
      "  --echo-state       drukuj stan bazy co odczyt\n"
      "  --dry-run          nic nie wysyla; pokazuje ramki i roznice po --set\n"
      "  --yes              pomin 3-sekundowe odliczanie przed ruchem\n"
      "\n"
      "BEZPIECZENSTWO: robot ruszy. Zapewnij wolna przestrzen, trzymaj fizyczny STOP w zasiegu reki,\n"
      "pierwsze proby rob z mala predkoscia i krotkim czasem.\n",
      amr::kVersion);
}

// plik = albo czysty hex, albo zapis z amr_proxy (linie "+t DIR HEX" i komentarze "#")
bool frameFromFile(const std::string& path, std::string& hex_out, std::string& err) {
  std::ifstream f(path);
  if (!f) {
    err = "nie moge otworzyc " + path;
    return false;
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    if (line[0] == '+') continue;  // to zapis proxy - uzyj --from-capture
    hex_out += line;  // czysty hex, moze byc w wielu liniach
  }
  if (hex_out.empty()) {
    err = "brak heksu w pliku " + path;
    return false;
  }
  return true;
}

double nowSec() {
  static const auto t0 = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

std::string stamp() {
  char buf[32];
  std::snprintf(buf, sizeof buf, "[+%7.2fs]", nowSec());
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (arg == "--ip") a.opt.host = next();
    else if (arg == "--port") a.opt.port = std::atoi(next().c_str());
    else if (arg == "--user") a.opt.user = next();
    else if (arg == "--pass-md5") a.opt.pass_md5 = next();
    else if (arg == "--type") a.type = std::atoi(next().c_str());
    else if (arg == "--frame") a.frame = next();
    else if (arg == "--stop-type") a.stop_type = std::atoi(next().c_str());
    else if (arg == "--stop-frame") a.stop_frames.push_back(next());
    else if (arg == "--pre-frame") a.pre_frames.push_back(next());
    else if (arg == "--stop-set") a.stop_sets.push_back(next());
    else if (arg == "--from-capture") a.capture = next();
    else if (arg == "--dir") a.capture_dir = next();
    else if (arg == "--nth") a.nth = std::atoi(next().c_str());
    else if (arg == "--set") a.sets.push_back(next());
    else if (arg == "--seconds") { a.seconds = std::atof(next().c_str()); a.seconds_set = true; }
    else if (arg == "--repeat") a.repeat = std::atoi(next().c_str());
    else if (arg == "--rate") a.rate = std::atof(next().c_str());
    else if (arg == "--hold") a.hold = std::atof(next().c_str());
    else if (arg == "--dry-run") a.dry_run = true;
    else if (arg == "--yes") a.yes = true;
    else if (arg == "--echo-state") a.echo_state = true;
    else if (arg == "--help" || arg == "-h") { usage(); return 0; }
    else {
      std::cerr << "Nieznany argument: " << arg << "\n";
      usage();
      return 2;
    }
  }
  if (a.capture.size()) a.capture_type = a.type;
  if (a.rate <= 0) a.rate = 20;
  if (a.rate > 50) {
    std::cerr << "Uwaga: --rate " << a.rate << " Hz za duzo, przycinam do 50 Hz\n";
    a.rate = 50;
  }
  if (!a.seconds_set && a.repeat == 0) a.seconds = 2;  // domyslnie 2 s jazdy

  const bool has_source = (a.type >= 0) || !a.frame.empty() || !a.capture.empty();
  if (!has_source) {
    usage();
    return 2;
  }

  // --- zrodlo komendy
  std::string frame_hex = a.frame;
  std::string frame_info;
  if (!a.frame.empty() && a.frame[0] == '@') {
    std::string err;
    frame_hex.clear();  // nazwa pliku nie moze wejsc do heksu
    if (!frameFromFile(a.frame.substr(1), frame_hex, err)) {
      std::cerr << err << "\n";
      return 2;
    }
  }
  if (!a.capture.empty()) {
    std::string err, info;
    if (!amr::readCaptureSelect(a.capture, a.capture_dir, a.capture_type, a.nth, frame_hex, info, err)) {
      std::cerr << err << "\n";
      return 2;
    }
    frame_info = info;
    std::cout << "Z przechwycenia: " << info << "\n";
  }

  // --- budowa ramek
  std::string jog_body, jog_template;
  if (!frame_hex.empty()) {
    if (!amr::hexDecode(frame_hex, jog_template)) {
      std::cerr << "Zly hex w ramce\n";
      return 2;
    }
  } else {
    std::vector<amr::Node> inner;
    inner.push_back(amr::varintNode(1, a.type));
    std::vector<amr::Node> extra;
    for (const std::string& s : a.sets) {
      std::string err;
      if (!amr::applyPatch(extra, s, err)) {
        std::cerr << "Blad w --set " << s << ": " << err << "\n";
        return 2;
      }
    }
    for (amr::Node& n : extra) inner.push_back(std::move(n));
    jog_template = amr::encodeEnvelope(0, 1, 0, 4, amr::serializeTree(inner));
  }

  // nadpisania pol w ramce (--set) - na kopii drzewa
  std::vector<amr::Node> jog_tree;
  if (!amr::parseTree(jog_template, jog_tree)) {
    std::cerr << "Nie umiem zdekodowac ramki komendy\n";
    return 2;
  }
  for (const std::string& s : a.sets) {
    if (frame_hex.empty()) break;  // dla --type pola juz wpisane wyzej
    std::string err;
    if (!amr::applyPatch(jog_tree, s, err)) {
      std::cerr << "Blad w --set " << s << ": " << err << "\n";
      return 2;
    }
  }
  jog_template = amr::serializeTree(jog_tree);

  // --- ramki wstepne (np. przejecie sterowania) - te same mechanizmy co przy stopie
  std::vector<std::string> pre_templates;
  for (const std::string& pf : a.pre_frames) {
    std::string hex = pf;
    if (!pf.empty() && pf[0] == '@') {
      std::string err;
      hex.clear();
      if (!frameFromFile(pf.substr(1), hex, err)) {
        std::cerr << err << "\n";
        return 2;
      }
    }
    std::string body;
    if (!amr::hexDecode(hex, body)) {
      std::cerr << "Zly hex w --pre-frame\n";
      return 2;
    }
    pre_templates.push_back(body);
  }

  // --- ramki stopu (moze byc kilka: najpierw wylacz ruch, potem np. oddaj sterowanie)
  std::vector<std::string> stop_templates;
  bool have_stop = false;
  for (const std::string& sf : a.stop_frames) {
    std::string hex = sf;
    if (!sf.empty() && sf[0] == '@') {
      std::string err;
      hex.clear();
      if (!frameFromFile(sf.substr(1), hex, err)) {
        std::cerr << err << "\n";
        return 2;
      }
    }
    std::string body;
    if (!amr::hexDecode(hex, body)) {
      std::cerr << "Zly hex w --stop-frame\n";
      return 2;
    }
    stop_templates.push_back(body);
    have_stop = true;
  }
  if (a.stop_type >= 0) {
    std::vector<amr::Node> inner;
    inner.push_back(amr::varintNode(1, a.stop_type));
    std::vector<amr::Node> extra;
    for (const std::string& s : a.stop_sets) {
      std::string err;
      if (!amr::applyPatch(extra, s, err)) {
        std::cerr << "Blad w --stop-set " << s << ": " << err << "\n";
        return 2;
      }
    }
    for (amr::Node& n : extra) inner.push_back(std::move(n));
    stop_templates.push_back(amr::encodeEnvelope(0, 1, 0, 4, amr::serializeTree(inner)));
    have_stop = true;
  }

  const bool loop_mode = (a.repeat > 1) || (a.repeat == 0 && a.seconds > 0);
  const size_t frames_total =
      a.repeat > 1 ? static_cast<size_t>(a.repeat) : static_cast<size_t>(std::ceil(a.seconds * a.rate));

  // --- podglad tego, co poleci
  std::cout << "== ramka ruchu (szablon) ==\n" << amr::describeFrame(jog_template) << "\n"
            << amr::dumpBody(jog_template);
  if (!frame_info.empty()) std::cout << "(zrodlo: przechwycenie, " << frame_info << ")\n";
  for (const std::string& pt : pre_templates) {
    std::cout << "== ramka wstepna ==\n" << amr::describeFrame(pt) << "\n" << amr::dumpBody(pt);
  }
  if (have_stop) {
    for (const std::string& stp : stop_templates) {
      std::cout << "== ramka stopu ==\n" << amr::describeFrame(stp) << "\n" << amr::dumpBody(stp);
    }
  } else {
    std::cout << "== ramka stopu: BRAK (--stop-frame/--stop-type nie podano) ==\n";
  }

  if (a.dry_run) {
    for (const std::string& pt : pre_templates) {
      std::cout << "[dry-run] najpierw poszlaby ramka wstepna: " << amr::describeFrame(pt) << "\n";
    }
    std::cout << "\n[dry-run] tryb: " << (loop_mode ? "petla" : "pojedyncza ramka") << ", "
              << frames_total << " x " << a.rate << " Hz"
              << (loop_mode ? "" : " + stop po " + std::to_string(a.hold) + " s") << "\n";
    std::cout << "[dry-run] nic nie wyslano.\n";
    return 0;
  }

  amr::MatrixClient client(a.opt);
  std::string err;
  if (!client.connect(err)) {
    std::cerr << "Blad polaczenia z " << a.opt.host << ":" << a.opt.port << ": " << err << "\n";
    return 1;
  }
  std::cout << "\nPolaczono z " << a.opt.host << ":" << a.opt.port << ", login wyslany\n";

  amr::State first_state, last_state;
  bool have_first = false;
  const auto on_frame = [&](const std::string&) {};
  const auto on_state = [&](const amr::State& s) {
    if (!have_first) {
      first_state = s;
      have_first = true;
    }
    last_state = s;
    if (a.echo_state) std::printf("%s stan: %s\n", stamp().c_str(), amr::stateLine(s).c_str());
  };

  // czekamy na sesje (potwierdzenie logowania)
  for (int i = 0; i < 40 && !g_stop && !client.logged_in(); ++i) {
    client.pollAny(100, on_frame, on_state);
  }
  if (!client.logged_in()) {
    std::cerr << "Uwaga: baza nie przyslala sesji w 4 s. Moze byc zajeta panelem? Proboje dalej.\n";
  } else {
    std::cout << "Sesja OK.\n";
  }

  signal(SIGINT, onSigint);
  signal(SIGTERM, onSigint);

  if (!a.yes && !g_stop) {
    std::cout << "Ruch za 3 s - Ctrl-C anuluje. ";
    for (int i = 3; i > 0 && !g_stop; --i) {
      std::cout << i << "..." << std::flush;
      const double end = nowSec() + 1.0;
      while (nowSec() < end && !g_stop) {
        client.pollAny(50, on_frame, on_state);
      }
    }
    std::cout << "\n";
  }
  if (g_stop) {
    std::cout << "Anulowano przed wyslaniem czegokolwiek.\n";
    client.close();
    return 0;
  }

  auto send_stop = [&]() {
    if (!have_stop) return;
    for (const std::string& stp : stop_templates) {
      for (int i = 0; i < 2 && !g_stop; ++i) {
        std::string eff;
        if (client.sendReplay(stp, &eff)) {
          std::printf("%s wyslano ramke stopu: %s\n", stamp().c_str(), amr::describeFrame(eff).c_str());
        }
        const double end = nowSec() + 0.05;
        while (nowSec() < end) client.pollAny(20, on_frame, on_state);
      }
    }
  };

  for (const std::string& pt : pre_templates) {
    std::string eff;
    if (client.sendReplay(pt, &eff)) {
      std::printf("%s wyslano ramke wstepna: %s\n", stamp().c_str(), amr::describeFrame(eff).c_str());
    }
    const double end = nowSec() + 0.15;
    while (nowSec() < end && !g_stop) client.pollAny(20, on_frame, on_state);
  }
  if (g_stop) {
    std::cout << "Przerwano przed ruchem (po ramkach wstepnych).\n";
    client.close();
    return 0;
  }

  size_t sent = 0;
  const double t_start = nowSec();
  const double interval = 1.0 / a.rate;
  double next_at = t_start;
  while (!g_stop) {
    if (sent >= frames_total) break;
    if (loop_mode) {
      const double el = nowSec() - t_start;
      if (a.repeat > 1) {
        if (el > (static_cast<double>(a.repeat) / a.rate) + 2.0) break;  // bezpiecznik
      } else if (el >= a.seconds) {
        break;
      }
    }
    if (nowSec() < next_at) {
      client.pollAny(static_cast<int>(std::min(20.0, (next_at - nowSec()) * 1000)), on_frame, on_state);
      continue;
    }
    std::string eff;
    if (!client.sendReplay(jog_template, &eff)) {
      std::cerr << "Blad wysylki\n";
      break;
    }
    ++sent;
    if (a.repeat > 1 && sent >= frames_total) break;
    next_at += interval;
    if (!loop_mode) break;
    if (sent % static_cast<size_t>(std::max(1.0, a.rate)) == 0) {
      std::printf("%s wyslano %zu ramek, stan: %s\n", stamp().c_str(), sent,
                  have_first ? amr::stateLine(last_state).c_str() : "(brak)");
    }
  }
  if (!loop_mode) {
    // pojedyncza komenda: przytrzymaj i pusc
    const double end = nowSec() + (g_stop ? 0.0 : a.hold);
    while (nowSec() < end && !g_stop) client.pollAny(50, on_frame, on_state);
  }
  send_stop();

  // jeszcze chwile czytamy stan po ruchu
  const double t_end = nowSec() + 0.5;
  while (nowSec() < t_end && !g_stop) client.pollAny(50, on_frame, on_state);

  std::printf("\nPodsumowanie: wyslano %zu ramek ruchu%s\n", sent, have_stop ? " + ramka stopu" : "");
  if (have_first) {
    const double dx = last_state.x_mm - first_state.x_mm;
    const double dy = last_state.y_mm - first_state.y_mm;
    double dyaw = last_state.yaw_rad - first_state.yaw_rad;
    while (dyaw > M_PI) dyaw -= 2 * M_PI;
    while (dyaw < -M_PI) dyaw += 2 * M_PI;
    std::printf("Przemieszczenie: dx=%+.1f mm, dy=%+.1f mm, dkat=%+.4f rad (%.1f stopni), droga=%.1f mm\n",
                dx, dy, dyaw, dyaw * 180.0 / M_PI, std::hypot(dx, dy));
  } else {
    std::printf("(brak odczytow stanu - nie wiem, czy sie ruszyl)\n");
  }
  client.close();
  return 0;
}
