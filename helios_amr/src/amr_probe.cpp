// amr_probe - podgladacz i probnik ruchu do bazy Helios (Matrix OS).
//
// Co robi:
//   * pokazuje KAZDA ramke przychodzaca z bazy: koperta (kind/seq/sesja/typ) + drzewo pol,
//   * dla powtarzajacych sie ramek (ten sam kind+typ) pokazuje tylko roznice wzgledem poprzedniej,
//     czyli dokladnie to, czego szukamy: "co sie zmienia, gdy cos sie dzieje",
//   * dla pushu stanu (typ 2) wypisuje od razu x/y/kat/dystans/faza,
//   * umie wyslac zadanie: --type N --set a.b=v (sciezka wzgledna do tresci zadania),
//   * umie wyslac/odtworzyc ramke z przechwycenia: --hex <hex> (podmienia seq i sesje),
//   * tryb interaktywny: --interactive (komendy: type, hex, dump, q).
//
// Budowanie: make            (albo: g++ -std=c++17 -O2 -Iinclude src/amr_probe.cpp -o amr_probe)
//
// Przyklady:
//   ./amr_probe --ip 192.168.71.50                        # tylko podglad
//   ./amr_probe --type 17 --seconds 5                     # wyslij zapytanie typu 17 i patrz, co wroci
//   ./amr_probe --type 100 --set 2.1=200 --set 2.2=0 --repeat 40 --rate 20   # (protokol LAWKI, nie bazy!)
//   ./amr_probe --dump-hex 0800100000000000000000102a... # zdekoduj ramke offline (np. z cap.log)
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "amr_matrix.hpp"

namespace {

struct Args {
  amr::MatrixClient::Options opt;
  double seconds = 0;
  bool all = false;
  bool full = true;       // pelne drzewo dla nowych/zmienionych ramek
  bool diff = true;       // pokazuj roznice dla powtorzen
  bool interactive = false;
  bool dry_run = false;
  int send_type = -1;
  std::string send_hex;
  std::string dump_hex;
  int repeat = 1;
  double rate = 10;
  std::vector<std::string> sets;
};

void usage() {
  std::printf(
      "amr_probe %s - podgladacz ruchu do bazy Helios (Matrix OS)\n"
      "\n"
      "  --ip ADRES          (domyslnie 192.168.71.50)\n"
      "  --port N            (domyslnie 5002)\n"
      "  --user LOGIN        (domyslnie admin)\n"
      "  --pass-md5 HASH     MD5 hasla w hex; policz: echo -n 'haslo' | md5sum\n"
      "  --seconds S         ile sekund nasluchiwac (0 = do Ctrl-C; z --type domyslnie 5 s)\n"
      "  --all               pokazuj kazda ramke w calosci (bez filtrowania powtorzen)\n"
      "  --type N            po zalogowaniu wyslij zadanie o kodzie N\n"
      "  --set a.b=v         pole w tresci zadania (mozna powtarzac); typy: v,u,z,s,f,d\n"
      "  --repeat N --rate H wysylaj zadanie N razy z czestotliwoscia H Hz (domyslnie 1 x 10 Hz)\n"
      "  --hex HEX           wyslij przechwycona ramke (podmienia tylko seq i sesje)\n"
      "  --interactive       polecenia ze stdin: type N [a.b=v ...] | hex HEX | dump HEX | q\n"
      "  --dump-hex HEX      zdekoduj ramke offline i wyjdz (bez laczenia)\n"
      "  --dry-run           pokaz, co zostanie wyslane, ale nie wysylaj\n",
      amr::kVersion);
}

std::string nowStr(double t) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "[+%7.2fs]", t);
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
    else if (arg == "--seconds") a.seconds = std::atof(next().c_str());
    else if (arg == "--all") { a.all = true; a.diff = false; }
    else if (arg == "--type") a.send_type = std::atoi(next().c_str());
    else if (arg == "--set") a.sets.push_back(next());
    else if (arg == "--repeat") a.repeat = std::atoi(next().c_str());
    else if (arg == "--rate") a.rate = std::atof(next().c_str());
    else if (arg == "--hex") a.send_hex = next();
    else if (arg == "--interactive") a.interactive = true;
    else if (arg == "--dump-hex") a.dump_hex = next();
    else if (arg == "--dry-run") a.dry_run = true;
    else if (arg == "--help" || arg == "-h") { usage(); return 0; }
    else {
      std::cerr << "Nieznany argument: " << arg << "\n";
      usage();
      return 2;
    }
  }

  if (!a.dump_hex.empty()) {
    std::string body;
    if (!amr::hexDecode(a.dump_hex, body)) {
      std::cerr << "Zly hex\n";
      return 2;
    }
    std::cout << amr::describeFrame(body) << "\n" << amr::dumpBody(body);
    return 0;
  }

  // --- zbuduj tresc zadania, jesli podano --type
  std::string request_extra;
  if (a.send_type >= 0 || !a.sets.empty()) {
    std::vector<amr::Node> extra;
    for (const std::string& s : a.sets) {
      std::string err;
      if (!amr::applyPatch(extra, s, err)) {
        std::cerr << "Blad w --set " << s << ": " << err << "\n";
        return 2;
      }
    }
    request_extra = amr::serializeTree(extra);
  }
  if (!a.sets.empty() && a.send_type < 0 && a.send_hex.empty()) {
    std::cerr << "Uwaga: --set podano bez --type/--hex, nic nie wysle.\n";
  }
  if (a.seconds == 0 && (a.send_type >= 0 || !a.send_hex.empty())) a.seconds = 5;

  if (a.dry_run) {
    if (!a.send_hex.empty()) {
      std::string body;
      if (!amr::hexDecode(a.send_hex, body)) {
        std::cerr << "Zly hex\n";
        return 2;
      }
      std::cout << "[dry-run] ramka z przechwycenia, wyslana " << a.repeat << " x " << a.rate << " Hz\n"
                << amr::describeFrame(body) << "\n"
                << amr::dumpBody(body);
      return 0;
    }
    std::vector<amr::Node> inner;
    inner.push_back(amr::varintNode(1, a.send_type));
    std::vector<amr::Node> extra;
    amr::parseTree(request_extra, extra);
    for (auto& n : extra) inner.push_back(std::move(n));
    const std::string body = amr::encodeEnvelope(0, 1, 0, 4, amr::serializeTree(inner));
    std::cout << "[dry-run] zadanie type=" << a.send_type << ", wyslane " << a.repeat << " x " << a.rate
              << " Hz\n"
              << amr::describeFrame(body) << "\n"
              << amr::dumpBody(body)
              << "hex: " << amr::hexEncode(body) << "\n";
    return 0;
  }

  amr::MatrixClient client(a.opt);
  std::string err;
  if (!client.connect(err)) {
    std::cerr << "Blad polaczenia z " << a.opt.host << ":" << a.opt.port << ": " << err << "\n";
    return 1;
  }
  std::cout << "Polaczono z " << a.opt.host << ":" << a.opt.port << ", login wyslany (amr_probe "
            << amr::kVersion << ")\n";

  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };

  std::map<std::string, std::string> prev;  // klucz "kind|type" -> poprzednie body
  int sent = 0;
  double next_send_at = 0.5;    // najpierw dajmy sie zalogowac
  bool warned_no_session = false;

  auto on_frame = [&](const std::string& body) {
    const amr::FrameInfo fi = amr::frameInfo(body);
    const std::string key = std::to_string(fi.kind) + "|" + (fi.has_type ? std::to_string(fi.type) : std::string("-"));
    const auto it = prev.find(key);
    const bool is_new = (it == prev.end());
    amr::State st;
    const bool is_state = amr::stateFromBody(body, st);

    if (a.all || is_new) {
      std::cout << nowStr(elapsed()) << " IN  " << amr::describeFrame(body) << "\n";
      std::cout << amr::dumpBody(body);
      if (is_state) std::cout << nowStr(elapsed()) << "     stan: " << amr::stateLine(st) << "\n";
    } else if (a.diff) {
      const std::vector<std::string> d = amr::diffBodies(it->second, body);
      if (!d.empty()) {
        std::cout << nowStr(elapsed()) << " IN  " << amr::describeFrame(body) << "  zmiany:\n";
        for (const std::string& s : d) std::cout << "        " << s << "\n";
        if (is_state) std::cout << nowStr(elapsed()) << "     stan: " << amr::stateLine(st) << "\n";
      }
    }
    prev[key] = body;
  };

  auto on_state = [&](const amr::State&) {};

  auto do_send = [&]() -> bool {
    std::string eff;
    bool ok = false;
    if (!a.send_hex.empty()) {
      std::string body;
      if (!amr::hexDecode(a.send_hex, body)) {
        std::cerr << "Zly hex\n";
        return false;
      }
      ok = client.sendReplay(body, &eff);
    } else if (a.send_type >= 0) {
      ok = client.sendRequest(static_cast<uint32_t>(a.send_type), request_extra, &eff);
    } else {
      return false;
    }
    if (ok) {
      std::cout << nowStr(elapsed()) << " OUT " << amr::describeFrame(eff) << "\n";
      ++sent;
    } else {
      std::cerr << "Nie udalo sie wyslac ramki\n";
    }
    return ok;
  };

  auto handle_command = [&](const std::string& line) {
    // type N [a.b=v ...] | hex HEX | dump HEX | q
    std::vector<std::string> tok;
    size_t p = 0;
    while (p < line.size()) {
      while (p < line.size() && std::isspace(static_cast<unsigned char>(line[p]))) ++p;
      size_t q = p;
      while (q < line.size() && !std::isspace(static_cast<unsigned char>(line[q]))) ++q;
      if (q > p) tok.push_back(line.substr(p, q - p));
      p = q;
    }
    if (tok.empty()) return;
    if (tok[0] == "q" || tok[0] == "quit") {
      a.seconds = 0.0001;  // wyjdz z petli
      return;
    }
    if (tok[0] == "dump") {
      if (tok.size() < 2) { std::cerr << "uzycie: dump <hex>\n"; return; }
      std::string body;
      if (!amr::hexDecode(tok[1], body)) { std::cerr << "zly hex\n"; return; }
      std::cout << amr::describeFrame(body) << "\n" << amr::dumpBody(body);
      return;
    }
    if (tok[0] == "hex") {
      if (tok.size() < 2) { std::cerr << "uzycie: hex <hex>\n"; return; }
      std::string body;
      if (!amr::hexDecode(tok[1], body)) { std::cerr << "zly hex\n"; return; }
      std::string eff;
      if (client.sendReplay(body, &eff))
        std::cout << nowStr(elapsed()) << " OUT " << amr::describeFrame(eff) << "\n";
      return;
    }
    if (tok[0] == "type") {
      if (tok.size() < 2) { std::cerr << "uzycie: type N [a.b=v ...]\n"; return; }
      std::vector<amr::Node> extra;
      for (size_t i = 2; i < tok.size(); ++i) {
        std::string err2;
        if (!amr::applyPatch(extra, tok[i], err2)) {
          std::cerr << "blad w " << tok[i] << ": " << err2 << "\n";
          return;
        }
      }
      std::string eff;
      if (client.sendRequest(static_cast<uint32_t>(std::atoi(tok[1].c_str())), amr::serializeTree(extra), &eff))
        std::cout << nowStr(elapsed()) << " OUT " << amr::describeFrame(eff) << "\n";
      return;
    }
    std::cerr << "nieznane polecenie: " << tok[0] << " (type|hex|dump|q)\n";
  };

  for (;;) {
    const double el = elapsed();
    if (a.seconds > 0 && el >= a.seconds) break;

    pollfd fds[2];
    int nfds = 1;
    fds[0] = pollfd{client.nativeFd(), POLLIN, 0};
    if (a.interactive) {
      fds[1] = pollfd{STDIN_FILENO, POLLIN, 0};
      nfds = 2;
    }
    // jesli czekamy na kolejna wysylke, nie spimy dluzej niz do jej terminu (dokladniejsze tempo)
    int wait_ms = 50;
    if ((a.send_type >= 0 || !a.send_hex.empty()) && sent < a.repeat && client.logged_in()) {
      const double left_ms = (next_send_at - el) * 1000.0;
      wait_ms = static_cast<int>(std::max(1.0, std::min(50.0, left_ms)));
    }
    const int pr = ::poll(fds, static_cast<nfds_t>(nfds), wait_ms);
    if (pr > 0) {
      if (fds[0].revents & (POLLIN | POLLHUP | POLLERR)) {
        const amr::Recv r = client.pollAny(0, on_frame, on_state);
        if (r == amr::Recv::Closed) {
          std::cerr << "Polaczenie zamkniete przez baze\n";
          return 3;
        }
      }
      if (nfds == 2 && (fds[1].revents & POLLIN)) {
        std::string line;
        if (!std::getline(std::cin, line)) {
          a.interactive = false;
        } else {
          handle_command(line);
        }
      }
    }

    // wysylka zadania (raz albo w petli)
    if ((a.send_type >= 0 || !a.send_hex.empty()) && sent < a.repeat) {
      if (client.logged_in() && el >= next_send_at) {
        if (do_send()) {
          next_send_at = el + (a.rate > 0 ? 1.0 / a.rate : 0.1);
        } else {
          break;
        }
      } else if (!client.logged_in() && el > 5 && !warned_no_session) {
        warned_no_session = true;
        std::cerr << "Uwaga: po 5 s baza nie przyslala sesji - moze byc zajeta innym panelem.\n";
      }
    }
  }

  client.close();
  std::cout << "Koniec (wyslano " << sent << " ramek)\n";
  return 0;
}
