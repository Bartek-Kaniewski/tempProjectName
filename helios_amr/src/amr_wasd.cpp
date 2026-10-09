// amr_wasd - reczne prowadzenie bazy Helios z klawiatury ("hold to move").
//
// Idea: bierzesz JEDNA ramke jogu (przechwycona z panelu, albo zbudowana --type) i podstawiasz
// w niej pola predkosci placeholderami @@vx@@ / @@wz@@. Program wysyla te ramke w petli z
// czestotliwoscia --rate, wpisujac aktualne predkosci; po puszczeniu klawiszy wysyla ramke stopu.
//
// Klawisze:
//   W / strzalka w gore   - jazda do przodu
//   S / strzalka w dol    - jazda do tylu
//   A / strzalka w lewo   - obrot w lewo
//   D / strzalka w prawo  - obrot w prawo
//   SPACJA                - natychmiast stop (zeruje predkosc i wysyla ramke stopu)
//   X albo Ctrl-C         - wyjscie
//
// Terminal nie przekazuje "puszczenia" klawisza, dlatego dziala to tak: kazde wcisniecie
// (w tym autopowtarzanie, gdy trzymasz klawisz) odswieza okno --hold-ms. Gdy w oknie nic nie
// przyjdzie, predkosc spada do zera, a program wysyla ramke stopu. Czyli: trzymasz = jedzie,
// puszczasz = staje (albo stukaj klawiszem, zeby dojechac malymi kroczkami).
//
// Budowanie: make
//
// Przyklady:
//   # 1) na atrapie (protokol LAWKI): type 100, pole 2 = {1: vx mm/s, 2: wz mrad/s}
//   ./amr_wasd --ip 127.0.0.1 --port 5003 --type 100 --set 2.1=@@vx@@ --set 2.2=@@wz@@ --stop-type 102
//
//   # 2) na prawdziwej bazie: ramka jogu z przechwycenia panelu + ramka puszczenia
//   ./amr_wasd --ip 192.168.71.50 --frame 0800150b00000019... --set 4.2.1=@@vx@@ --set 4.2.2=@@wz@@ --stop-frame 0800150c... --speed 200 --turn 250
//
//   # 3) najpierw na sucho (nic nie wysyla, pokazuje ramki i predkosci):
//   ./amr_wasd --type 100 --set 2.1=@@vx@@ --set 2.2=@@wz@@ --stop-type 102 --dry-run
#include <poll.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
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
  std::vector<std::string> sets;   // moga zawierac @@vx@@ / @@wz@@
  int stop_type = -1;
  std::vector<std::string> stop_frames;   // moze byc kilka (zera + oddanie sterowania)
  std::vector<std::string> pre_frames;
  std::vector<std::string> release_frames;   // na WYJSCIU (op=7 oddanie sterowania)
  bool rearm = true;                         // przed kazdym nowym ruchem powtorz op=6/op=111
  bool verbose = false;
  bool arc = true;                           // luki: dwa klawisze naraz = jazda po luku
  std::string capture;
  std::string capture_dir = "UP";
  int capture_type = -1;
  int nth = 1;
  double speed = 300;   // mm/s
  double turn = 400;    // mrad/s
  double rate = 20;     // Hz
  double accel = 900;   // mm/s^2 (narastanie/opadanie predkosci)
  double hold_ms = 350; // okno "trzymania" klawisza (autopowtarzanie terminala)
  double max_seconds = 0;
  bool dry_run = false;
};

void usage() {
  std::printf(
      "amr_wasd %s - prowadzenie bazy z klawiatury (W/S/A/D, SPACJA = stop, X = wyjscie)\n"
      "\n"
      "Zrodlo ramki ruchu:\n"
      "  --frame HEX|@plik       przechwycona ramka jogu (z amr_proxy --save)\n"
      "  --from-capture PLIK [--type N] [--nth K]   wybierz ramke jogu z zapisu proxy (linie UP)\n"
      "  --type N                ramka budowana od zera (do testow / atrapy)\n"
      "  --set a.b=@@vx@@        pole predkosci liniowej (mm/s)  [wymagane]\n"
      "  --set a.b=@@wz@@        pole predkosci obrotu (mrad/s) [wymagane]\n"
      "                          sciezka: przy --type wzgledem tresci zadania (2.1),\n"
      "                          przy --frame wzgledem calego body (4.2.1)\n"
      "\n"
      "Przed jazda (opcjonalnie):\n"
      "  --pre-frame HEX|@plik  np. 'przejmij sterowanie' z panelu (op=6) i op=111; mozna powtarzac\n"
      "\n"
      "Zatrzymanie:\n"
      "  --stop-frame HEX|@plik  ramka STOP po puszczeniu klawiszy; mozna podac kilka\n"
      "                          (ma byc TYLKO wyzerowanie predkosci, bez oddawania sterowania)\n"
      "  --release-frame HEX|@plik  ramka wysylana przy WYJSCIU z programu (np. op=7 oddanie\n"
      "                          sterowania); mozna podac kilka. Nie jest wysylana w trakcie jazdy.\n"
      "  --no-rearm              nie powtarzaj ramek wstepnych przed kazdym nowym ruchem\n"
      "  --verbose               pokaz ramki przychodzace z bazy (odpowiedzi, nie push stanu)\n"
      "  --arc / --no-arc        luki: trzymanie np. W+A jedzie i skreca NARAZ (domyslnie wlaczone).\n"
      "                          Gdy jazda drga przy trzymaniu: --hold-ms 600 albo 'xset r rate 250 30'.\n"
      "                          --no-arc = stara zasada: liczy sie klawisz, ktory sie powtarza.\n"
      "  --stop-type N           ramka stopu budowana od zera (przyklad: 102 na atrapie)\n"
      "\n"
      "Parametry ruchu:\n"
      "  --speed MM_S      predkosc jazdy (domyslnie 300 mm/s)\n"
      "  --turn MRAD_S     predkosc obrotu (domyslnie 400 mrad/s)\n"
      "  --rate HZ         czestotliwosc wysylki (domyslnie 20)\n"
      "  --accel MM_S2     lagodne narastanie predkosci (domyslnie 900)\n"
      "  --hold-ms MS      okno podtrzymania klawisza (domyslnie 350)\n"
      "  --max-seconds S   awaryjny limit czasu jazdy (0 = bez limitu)\n"
      "  --dry-run         nic nie wysyla (test), pokazuje co by poszlo\n"
      "\n"
      "BEZPIECZENSTWO: to jest sterowanie ruchem. Trzymaj fizyczny STOP, miej wolna przestrzen,\n"
      "zacznij od malych --speed i --turn.\n",
      amr::kVersion);
}

struct RawTty {
  termios old{};
  bool active = false;
  bool enable() {
    if (!isatty(STDIN_FILENO)) return false;
    if (tcgetattr(STDIN_FILENO, &old) != 0) return false;
    termios t = old;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &t) != 0) return false;
    active = true;
    return true;
  }
  ~RawTty() {
    if (active) tcsetattr(STDIN_FILENO, TCSANOW, &old);
  }
};

double nowSec() {
  static const auto t0 = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

bool frameFromFile(const std::string& path, std::string& hex_out, std::string& err) {
  std::ifstream f(path);
  if (!f) {
    err = "nie moge otworzyc " + path;
    return false;
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    if (line[0] == '+') continue;  // zapis proxy - uzyj amr_cmd --from-capture
    hex_out += line;
  }
  if (hex_out.empty()) {
    err = "brak heksu w " + path;
    return false;
  }
  return true;
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
    else if (arg == "--set") a.sets.push_back(next());
    else if (arg == "--stop-type") a.stop_type = std::atoi(next().c_str());
    else if (arg == "--stop-frame") a.stop_frames.push_back(next());
    else if (arg == "--pre-frame") a.pre_frames.push_back(next());
    else if (arg == "--release-frame") a.release_frames.push_back(next());
    else if (arg == "--no-rearm") a.rearm = false;
    else if (arg == "--verbose") a.verbose = true;
    else if (arg == "--arc") a.arc = true;
    else if (arg == "--no-arc") a.arc = false;
    else if (arg == "--from-capture") a.capture = next();
    else if (arg == "--dir") a.capture_dir = next();
    else if (arg == "--nth") a.nth = std::atoi(next().c_str());
    else if (arg == "--speed") a.speed = std::atof(next().c_str());
    else if (arg == "--turn") a.turn = std::atof(next().c_str());
    else if (arg == "--rate") a.rate = std::atof(next().c_str());
    else if (arg == "--accel") a.accel = std::atof(next().c_str());
    else if (arg == "--hold-ms") a.hold_ms = std::atof(next().c_str());
    else if (arg == "--max-seconds") a.max_seconds = std::atof(next().c_str());
    else if (arg == "--dry-run") a.dry_run = true;
    else if (arg == "--help" || arg == "-h") { usage(); return 0; }
    else {
      std::cerr << "Nieznany argument: " << arg << "\n";
      usage();
      return 2;
    }
  }
  if (a.capture.size() && a.type >= 0) a.capture_type = a.type;
  if ((a.type < 0 && a.frame.empty() && a.capture.empty()) || a.sets.empty()) {
    std::cerr << "Podaj ramke (--frame / --from-capture / --type) i pola predkosci (--set a.b=@@vx@@, --set a.b=@@wz@@)\n\n";
    usage();
    return 2;
  }
  if (a.rate <= 0 || a.rate > 50) a.rate = 20;
  if (a.hold_ms < 80) a.hold_ms = 80;

  // --- szablon ramki ruchu
  std::string jog_template;
  if (!a.capture.empty()) {
    std::string info, err2;
    if (!amr::readCaptureSelect(a.capture, a.capture_dir, a.capture_type, a.nth, a.frame, info, err2)) {
      std::cerr << err2 << "\n";
      return 2;
    }
    std::cout << "Ramka z przechwycenia: " << info << "\n";
  }
  if (!a.frame.empty()) {
    std::string hex = a.frame;
    if (a.frame[0] == '@') {
      hex.clear();
      std::string err;
      if (!frameFromFile(a.frame.substr(1), hex, err)) {
        std::cerr << err << "\n";
        return 2;
      }
    }
    if (!amr::hexDecode(hex, jog_template)) {
      std::cerr << "Zly hex w --frame\n";
      return 2;
    }
  } else {
    std::vector<amr::Node> inner;
    inner.push_back(amr::varintNode(1, a.type));
    jog_template = amr::encodeEnvelope(0, 1, 0, 4, amr::serializeTree(inner));
  }

  // --- ramki oddania sterowania (tylko na wyjsciu)
  std::vector<std::string> release_templates;
  for (const std::string& rf : a.release_frames) {
    std::string hex = rf;
    if (!rf.empty() && rf[0] == '@') {
      std::string err;
      hex.clear();
      if (!frameFromFile(rf.substr(1), hex, err)) {
        std::cerr << err << "\n";
        return 2;
      }
    }
    std::string body;
    if (!amr::hexDecode(hex, body)) {
      std::cerr << "Zly hex w --release-frame\n";
      return 2;
    }
    release_templates.push_back(body);
  }

  // --- ramki chwilowego stopu (puszczenie klawisza / SPACJA)
  std::vector<std::string> stop_templates;
  const bool have_stop = (a.stop_type >= 0) || !a.stop_frames.empty();
  if (a.stop_type >= 0) {
    std::vector<amr::Node> inner;
    inner.push_back(amr::varintNode(1, a.stop_type));
    stop_templates.push_back(amr::encodeEnvelope(0, 1, 0, 4, amr::serializeTree(inner)));
  } else if (!a.stop_frames.empty()) {
    for (const std::string& sf : a.stop_frames) {
      std::string hex = sf;
      if (!sf.empty() && sf[0] == '@') {
        hex.clear();
        std::string err;
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
    }
  }

  // --- renderowanie ramki z podstawionymi predkosciami
  // Tryb --type: sciezki --set sa wzgledem TRESCI zadania (tak jak w amr_cmd), np. "2.1=@@vx@@".
  // Tryb --frame: sciezki sa wzgledem CALEGO body (razem z koperta), np. "4.2.1=@@vx@@".
  auto replace_all = [](std::string& s, const std::string& what, const std::string& with) {
    size_t p = 0;
    while ((p = s.find(what, p)) != std::string::npos) {
      s.replace(p, what.size(), with);
      p += with.size();
    }
  };
  auto render = [&](int vx, int wz, std::string& err) -> std::string {
    if (a.type >= 0 && a.capture.empty()) {
      std::vector<amr::Node> inner;
      inner.push_back(amr::varintNode(1, a.type));
      for (const std::string& spec0 : a.sets) {
        std::string spec = spec0;
        replace_all(spec, "@@vx@@", std::to_string(vx));
        replace_all(spec, "@@wz@@", std::to_string(wz));
        if (!amr::applyPatch(inner, spec, err)) return {};
      }
      return amr::encodeEnvelope(0, 1, 0, 4, amr::serializeTree(inner));
    }
    std::vector<amr::Node> tree;
    if (!amr::parseTree(jog_template, tree)) {
      err = "nie umiem zdekodowac szablonu";
      return {};
    }
    for (const std::string& spec0 : a.sets) {
      std::string spec = spec0;
      replace_all(spec, "@@vx@@", std::to_string(vx));
      replace_all(spec, "@@wz@@", std::to_string(wz));
      if (!amr::applyPatch(tree, spec, err)) return {};
    }
    return amr::serializeTree(tree);
  };

  // --- podglad konfiguracji
  {
    std::string err;
    const std::string f0 = render(0, 0, err);
    if (f0.empty()) {
      std::cerr << "Blad szablonu: " << err << "\n";
      return 2;
    }
    std::cout << "== ramka ruchu (predkosci 0) ==\n" << amr::dumpBody(f0);
    if (have_stop) {
      for (const std::string& stp : stop_templates) {
        std::cout << "== ramka stopu ==\n" << amr::dumpBody(stp);
      }
    } else {
      std::cout << "== brak ramki stopu (--stop-frame/--stop-type) - na koniec pojdzie ramka z zerem ==\n";
    }
    std::cout << "== ustawienia: speed=" << a.speed << " mm/s, turn=" << a.turn << " mrad/s, rate="
              << a.rate << " Hz, accel=" << a.accel << " mm/s2, hold=" << a.hold_ms << " ms ==\n";
  }

  amr::MatrixClient client(a.opt);
  std::string err;
  if (!a.dry_run) {
    if (!client.connect(err)) {
      std::cerr << "Blad polaczenia z " << a.opt.host << ":" << a.opt.port << ": " << err << "\n";
      return 1;
    }
    for (int i = 0; i < 40 && !client.logged_in(); ++i) {
      client.pollAny(100, nullptr, nullptr);
    }
    if (!client.logged_in()) {
      std::cerr << "Uwaga: brak sesji po 4 s (baza zajeta panelem?). Probuje dalej.\n";
    }
  }

  std::vector<std::string> pre_templates;
  for (const std::string& pf : a.pre_frames) {
    std::string hex = pf;
    if (!pf.empty() && pf[0] == '@') {
      std::string err2;
      hex.clear();
      if (!frameFromFile(pf.substr(1), hex, err2)) {
        std::cerr << err2 << "\n";
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

  // wysylka ramek wstepnych (op=6 "przejmij sterowanie" itd.)
  auto send_pre = [&](const char* why) {
    for (const std::string& pt : pre_templates) {
      std::string eff;
      if (client.sendReplay(pt, &eff)) {
        std::cout << "\r\033[K" << why << ": " << amr::describeFrame(eff) << "\n";
        std::fflush(stdout);
      }
      const double end = nowSec() + 0.12;
      while (nowSec() < end) client.pollAny(20, nullptr, nullptr);
    }
  };
  send_pre("ramka wstepna");

  RawTty tty;
  const bool raw = tty.enable();
  std::cout << "\nSterowanie: W/S = przod/tyl, A/D = obrot, SPACJA = stop, X = wyjscie"
            << (raw ? "" : "  (stdin to nie terminal - czytam bajty bez trybu raw)") << "\n";
  std::cout << "Trzymaj klawisz = jedzie, pusc = staje. Fizyczny STOP w zasiegu reki!\n";
  std::cout << (a.arc ? "Tryb lukowy WLACZONY: np. W + A jedzie i skreca naraz (--no-arc wylacza).\n"
                      : "Tryb lukowy wylaczony (--arc wlacza): liczy sie ostatnio powtarzany klawisz.\n")
            << "\n";
  std::fflush(stdout);

  signal(SIGINT, onSigint);
  signal(SIGTERM, onSigint);

  double dl_fwd = 0, dl_back = 0, dl_left = 0, dl_right = 0;  // terminy waznosci "wcisnietego" klawisza
  // tryb lukowy (--arc): zestaw klawiszy zyje razem, dopoki cokolwiek sie powtarza
  double burst_until = 0;   // koniec "zestawu" - kazde powtorzenie dowolnego klawisza go przedluza
  int lat_motion = 0;       // zapamietany kierunek jazdy: +1 przod, -1 tyl, 0 brak
  int lat_turn = 0;         // zapamietany kierunek obrotu: +1 lewo, -1 prawo, 0 brak
  double vx = 0, wz = 0;                                       // aktualne (wygladzone) predkosci
  double next_tick = nowSec();
  double t_start = nowSec();
  double next_hud = 0;
  std::string esc_buf;
  amr::State state;
  bool have_state = false;
  long sent = 0, stops = 0, bursts = 0;
  bool was_moving = false;
  std::string last_rendered;

  auto on_state = [&](const amr::State& s) {
    state = s;
    have_state = true;
  };
  auto on_frame = [&](const std::string& body) {
    if (!a.verbose) return;
    const amr::FrameInfo fi = amr::frameInfo(body);
    if (fi.ok && fi.kind == 2) return;  // push stanu - pomijamy, za duzo
    std::cout << "\r\033[K[recv] " << amr::describeFrame(body) << "\n";
    std::fflush(stdout);
  };

  while (!g_stop) {
    const double now = nowSec();
    if (a.max_seconds > 0 && (now - t_start) > a.max_seconds) {
      std::cout << "\nLimit --max-seconds osiagniety - koncze.\n";
      break;
    }

    // --- wejscie
    pollfd fds[2];
    int nfds = 0;
    if (client.nativeFd() >= 0) fds[nfds++] = pollfd{client.nativeFd(), POLLIN, 0};
    fds[nfds++] = pollfd{STDIN_FILENO, POLLIN, 0};
    const int wait_ms = static_cast<int>(std::max(1.0, std::min(20.0, (next_tick - now) * 1000)));
    if (::poll(fds, static_cast<nfds_t>(nfds), wait_ms) > 0) {
      int idx = 0;
      if (client.nativeFd() >= 0) {
        if (fds[0].revents & (POLLIN | POLLHUP | POLLERR)) {
          if (client.pollAny(0, on_frame, on_state) == amr::Recv::Closed) {
            std::cerr << "\nPolaczenie zamkniete przez baze\n";
            g_stop = 1;
          }
        }
        idx = 1;
      }
      if (fds[idx].revents & POLLIN) {
        char buf[64];
        const ssize_t n = ::read(STDIN_FILENO, buf, sizeof buf);
        for (ssize_t i = 0; i < n; ++i) {
          const char c = buf[i];
          const double until = now + a.hold_ms / 1000.0;
          // wcisniecie klawisza kierunkowego 'w'/'s'/'a'/'d'
          auto press_dir = [&](char k) {
            if (!a.arc) {  // stara zasada: kazdy klawisz ma wlasny termin waznosci
              if (k == 'w') dl_fwd = until;
              else if (k == 's') dl_back = until;
              else if (k == 'a') dl_left = until;
              else if (k == 'd') dl_right = until;
              return;
            }
            // luki: zapamietujemy kierunek obu osi, a caly zestaw zyje tak dlugo,
            // jak przychodza powtorzenia JAKIEGOKOLWIEK klawisza z zestawu
            if (k == 'w') lat_motion = +1;
            else if (k == 's') lat_motion = -1;
            else if (k == 'a') lat_turn = +1;
            else if (k == 'd') lat_turn = -1;
            burst_until = until;
          };
          if (c == 0x1B) {  // moze byc strzalka: ESC [ A/B/C/D
            esc_buf = "\x1b";
            continue;
          }
          if (!esc_buf.empty()) {
            esc_buf.push_back(c);
            if (esc_buf.size() >= 3) {
              const char k = esc_buf[2];
              if (k == 'A') press_dir('w');
              else if (k == 'B') press_dir('s');
              else if (k == 'C') press_dir('d');
              else if (k == 'D') press_dir('a');
              esc_buf.clear();
            }
            continue;
          }
          switch (c) {
            case 'w': case 'W': press_dir('w'); break;
            case 's': case 'S': press_dir('s'); break;
            case 'a': case 'A': press_dir('a'); break;
            case 'd': case 'D': press_dir('d'); break;
            case ' ':
              dl_fwd = dl_back = dl_left = dl_right = 0;
              lat_motion = lat_turn = 0;
              burst_until = 0;
              vx = wz = 0;
              break;
            case 'x': case 'X': case 'q': case 'Q': case 0x03:
              g_stop = 1;
              break;
            default: break;
          }
        }
      }
    }

    if (now < next_tick) continue;
    const double dt = std::max(1e-3, std::min(0.2, now - (next_tick - 1.0 / a.rate)));
    next_tick = now + 1.0 / a.rate;

    // --- cel z klawiszy
    double t_vx = 0, t_wz = 0;
    if (a.arc && now >= burst_until && (lat_motion != 0 || lat_turn != 0)) {
      // wszystkie klawisze puszczone (cisza w powtorzeniach) - zestaw sie konczy,
      // wiec nastepny ruch ma startowac "od zera", bez starych klawiszy
      lat_motion = lat_turn = 0;
    }
    if (a.arc) {
      const bool burst = (now < burst_until);
      t_vx = (!burst || lat_motion == 0) ? 0.0 : (lat_motion > 0 ? a.speed : -a.speed);
      t_wz = (!burst || lat_turn == 0) ? 0.0 : (lat_turn > 0 ? a.turn : -a.turn);
    } else {
      t_vx = (now < dl_fwd ? a.speed : 0.0) - (now < dl_back ? a.speed : 0.0);
      t_wz = (now < dl_left ? a.turn : 0.0) - (now < dl_right ? a.turn : 0.0);
    }

    // --- wygladzanie (ograniczone przyspieszenie)
    const double dv = a.accel * dt;
    const double dwz = (a.speed > 0 ? a.accel * (a.turn / a.speed) : a.turn) * dt;
    vx += std::max(-dv, std::min(dv, t_vx - vx));
    wz += std::max(-dwz, std::min(dwz, t_wz - wz));
    if (std::fabs(vx) < 1) vx = 0;
    if (std::fabs(wz) < 1) wz = 0;

    const bool moving = (vx != 0 || wz != 0);

    if (!a.dry_run) {
      if (moving && !was_moving) {
        // nowy ruch: baza mogla stracic grant sterowania - przejmij je znowu
        ++bursts;
        if (bursts > 1 && a.rearm && !pre_templates.empty()) {
          send_pre("re-arm (ponowne przejecie sterowania)");
        }
        std::cout << "\r\033[K[start] vx=" << static_cast<int>(std::lround(vx))
                  << " mm/s wz=" << static_cast<int>(std::lround(wz)) << " mrad/s\n";
        std::fflush(stdout);
      }
      if (moving) {
        std::string e;
        const std::string body = render(static_cast<int>(std::lround(vx)), static_cast<int>(std::lround(wz)), e);
        if (body.empty()) {
          std::cerr << "Blad budowy ramki: " << e << "\n";
          g_stop = 1;
        } else {
          std::string eff;
          if (client.sendReplay(body, &eff)) ++sent;
          last_rendered = eff;
        }
        was_moving = true;
      } else if (was_moving) {
        // wszystkie klawisze puszczone (albo SPACJA) - zatrzymaj,
        // ale NIE oddawaj sterowania (inaczej baza przestaje sluchac kolejnych komend!)
        if (have_stop) {
          for (const std::string& stp : stop_templates) {
            for (int rep = 0; rep < 2; ++rep) {
              std::string eff;
              if (client.sendReplay(stp, &eff)) {
                ++stops;
                last_rendered = eff;
              }
              const double end = nowSec() + 0.05;
              while (nowSec() < end) client.pollAny(10, on_frame, on_state);
            }
          }
        } else {
          std::string e;
          const std::string body = render(0, 0, e);
          std::string eff;
          if (!body.empty() && client.sendReplay(body, &eff)) {
            ++stops;
            last_rendered = eff;
          }
        }
        std::cout << "\r\033[KSTOP (wyslano " << (have_stop ? "ramke stopu" : "ramke z zerem") << ")\n";
        std::fflush(stdout);
        was_moving = false;
      }
    } else {
      if (moving != was_moving) {
        std::string e;
        const std::string body = render(static_cast<int>(std::lround(vx)), static_cast<int>(std::lround(wz)), e);
        std::cout << "[dry-run] " << (moving ? "jedzie" : "stop") << ": " << amr::describeFrame(body) << "\n";
        if (!moving && have_stop) {
          for (const std::string& stp : stop_templates) {
            std::cout << "[dry-run] ramka stopu: " << amr::describeFrame(stp) << "\n";
          }
        }
        was_moving = moving;
      }
    }

    // --- HUD
    if (now >= next_hud) {
      next_hud = now + 0.2;
      char line[256];
      std::snprintf(line, sizeof line,
                    "\r\033[Kvx=%+5d mm/s  wz=%+5d mrad/s  |  x=%8.1f y=%9.1f kat=%+7.4f faza=%-2lld |  %s",
                    static_cast<int>(std::lround(vx)), static_cast<int>(std::lround(wz)), state.x_mm, state.y_mm,
                    state.yaw_rad, static_cast<long long>(state.phase), moving ? "JEDZIE" : "stoi ");
      std::cout << line << std::flush;
    }
  }

  // --- zakonczenie: zawsze probujemy zatrzymac
  if (!a.dry_run) {
    if (have_stop) {
      for (const std::string& stp : stop_templates) {
        for (int i = 0; i < 2; ++i) {
          std::string eff;
          if (client.sendReplay(stp, &eff)) ++stops;
          const double end = nowSec() + 0.05;
          while (nowSec() < end) client.pollAny(20, on_frame, on_state);
        }
      }
    } else {
      std::string e;
      const std::string body = render(0, 0, e);
      std::string eff;
      if (!body.empty()) client.sendReplay(body, &eff);
    }
    // oddanie sterowania (tylko tutaj!)
    for (const std::string& rt : release_templates) {
      for (int i = 0; i < 2; ++i) {
        std::string eff;
        if (client.sendReplay(rt, &eff)) {
          std::cout << "zwolnienie sterowania: " << amr::describeFrame(eff) << "\n";
        }
        const double end = nowSec() + 0.05;
        while (nowSec() < end) client.pollAny(20, on_frame, on_state);
      }
    }
    std::cout << "\n\nKoniec. Wyslano " << sent << " ramek ruchu, " << stops << " ramek zatrzymania, "
              << bursts << " ruchow.\n";
    if (have_state) {
      std::printf("Ostatni stan: %s\n", amr::stateLine(state).c_str());
    }
  } else {
    std::cout << "\n[dry-run] koniec, nic nie wyslano.\n";
  }
  return 0;
}
