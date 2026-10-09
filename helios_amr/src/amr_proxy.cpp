// amr_proxy - podsłuch (MITM) ruchu panel Matrix <-> baza Helios (Matrix OS, TCP 5002).
//
// Panel laczy sie z TYM programem (zamiast z baza), a program przekazuje ruch dalej do bazy,
// rozkladajac go po drodze na ramki aplikacji (u32 BE dlugosc + protobuf) i dekodujac pola.
// Dzieki temu widac DOKLADNIE, jaka ramke panel wysyla, gdy wciskasz "przod", "obrot", "jedz
// do stacji" itd. Ramki zapisane opcja --save mozna potem odtworzyc przez amr_cmd --from-capture.
//
// WAZNE: program nic nie zmienia w ruchu (przepisuje bajty 1:1), ale UMIESZCZENIE go miedzy
// panelem a baza moze chwilowo przerwac komunikacje. Robot nie ruszy sam z siebie, ale miej
// reke na przycisku STOP.
//
// Budowanie: make
// Uruchomienie (przyklad):
//   ./amr_proxy --listen 0.0.0.0:5002 --remote 192.168.71.50:5002 --save cap/krok1.log
// i w panelu Matrix ustaw adres bazy na IP tego komputera (port 5002).
// Jesli panel ma zaszyty adres na sztywno, na maszynie z panelem:
//   iptables -t nat -A OUTPUT -p tcp -d 192.168.71.50 --dport 5002 -j REDIRECT --to-ports 5002
//
// Przyklady uzycia znacznikow w panelu, zeby zlapac komendy ruchu:
//   1) zaloguj sie i poczekaj 5 s (to bedzie szum: logowanie + trzy zapytania + push stanu),
//   2) wcisnij "przod" na ~1 s i pusc, 3) "obrot w lewo" ~1 s, 4) "jedz do stacji 3",
//   5) STOP/anuluj. Potem w logu szukaj linii UP, ktorych nie bylo w punkcie 1.
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "amr_matrix.hpp"

namespace {

volatile sig_atomic_t g_stop = 0;
void onSigint(int) { g_stop = 1; }

constexpr size_t kMaxFrame = 64u << 20;

double nowSec() {
  static const auto start = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::string stamp() {
  char buf[48];
  std::snprintf(buf, sizeof buf, "[+%8.3fs]", nowSec());
  return buf;
}

bool parseHostPort(const std::string& s, std::string& host, int& port) {
  const size_t c = s.rfind(':');
  if (c == std::string::npos) return false;
  host = s.substr(0, c);
  port = std::atoi(s.substr(c + 1).c_str());
  return port > 0;
}

int connectTcp(const std::string& host, int port, std::string& err) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  const std::string ps = std::to_string(port);
  if (getaddrinfo(host.c_str(), ps.c_str(), &hints, &res) != 0 || !res) {
    err = "nie moge rozwiazac " + host;
    return -1;
  }
  const int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (fd < 0) {
    freeaddrinfo(res);
    err = std::strerror(errno);
    return -1;
  }
  timeval tv{};
  tv.tv_sec = 5;
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
  const int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
  if (::connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
    err = std::string("connect: ") + std::strerror(errno);
    freeaddrinfo(res);
    ::close(fd);
    return -1;
  }
  freeaddrinfo(res);
  return fd;
}

// ---- rozkladanie strumienia TCP na: payloady WebSocket -> ramki aplikacji

struct DirParser {
  bool hs_done = false;
  std::string tcpbuf;
  std::string app;

  // Zwraca w `out` kompletne ramki aplikacji (bez 4-bajtowego prefiksu).
  void feed(const char* d, size_t n, std::vector<std::string>& out) {
    tcpbuf.append(d, n);
    if (!hs_done) {
      const size_t pos = tcpbuf.find("\r\n\r\n");
      if (pos == std::string::npos) {
        if (tcpbuf.size() > 65536) tcpbuf.clear();
        return;
      }
      tcpbuf.erase(0, pos + 4);
      hs_done = true;
    }
    size_t off = 0;
    while (true) {
      if (tcpbuf.size() - off < 2) break;
      const uint8_t b0 = static_cast<uint8_t>(tcpbuf[off]);
      const uint8_t b1 = static_cast<uint8_t>(tcpbuf[off + 1]);
      const int op = b0 & 0x0F;
      const bool masked = b1 & 0x80;
      uint64_t len = b1 & 0x7F;
      size_t hdr = 2;
      if (len == 126) {
        if (tcpbuf.size() - off < 4) break;
        len = (static_cast<uint8_t>(tcpbuf[off + 2]) << 8) | static_cast<uint8_t>(tcpbuf[off + 3]);
        hdr = 4;
      } else if (len == 127) {
        if (tcpbuf.size() - off < 10) break;
        len = 0;
        for (int i = 0; i < 8; ++i) len = (len << 8) | static_cast<uint8_t>(tcpbuf[off + 2 + i]);
        hdr = 10;
      }
      const size_t need = hdr + (masked ? 4 : 0) + static_cast<size_t>(len);
      if (tcpbuf.size() - off < need) break;
      uint8_t mask[4] = {0, 0, 0, 0};
      if (masked) std::memcpy(mask, tcpbuf.data() + off + hdr, 4);
      std::string payload(tcpbuf.data() + off + hdr + (masked ? 4 : 0), static_cast<size_t>(len));
      if (masked) {
        for (size_t i = 0; i < payload.size(); ++i)
          payload[i] = static_cast<char>(static_cast<uint8_t>(payload[i]) ^ mask[i & 3]);
      }
      off += need;
      if (op == 0x0 || op == 0x1 || op == 0x2) app += payload;
    }
    tcpbuf.erase(0, off);

    while (app.size() >= 4) {
      const uint32_t n = (static_cast<uint8_t>(app[0]) << 24) | (static_cast<uint8_t>(app[1]) << 16) |
                         (static_cast<uint8_t>(app[2]) << 8) | static_cast<uint8_t>(app[3]);
      if (n > kMaxFrame) {  // rozsynchronizowany strumien - czyscimy
        app.clear();
        break;
      }
      if (app.size() < 4 + static_cast<size_t>(n)) break;
      out.push_back(app.substr(4, n));
      app.erase(0, 4 + static_cast<size_t>(n));
    }
  }
};

struct Options {
  std::string listen_host = "0.0.0.0";
  int listen_port = 5002;
  std::string remote_host = "192.168.71.50";
  int remote_port = 5002;
  std::string save;
  bool up_full = true;    // pelne drzewo dla ramek panel -> baza
  bool down_full = false; // pelne drzewo dla ramek baza -> panel (poza stanem i tak pokazujemy)
  bool quiet_repeats = true;
};

struct Session {
  int cfd = -1;
  int sfd = -1;
  DirParser up, dn;
  std::string peer;
  bool saw_handshake = false;
};

struct Stats {
  std::map<std::string, long> counts;
  std::map<std::string, std::string> prev;
  void add(const std::string& key) { counts[key]++; }
};

Options g_opt;
Stats g_stats;
FILE* g_save = nullptr;
long g_saved = 0;

void saveFrame(const char* dir, const std::string& body) {
  if (!g_save) return;
  std::fprintf(g_save, "+%9.3f %s %s\n", nowSec(), dir, amr::hexEncode(body).c_str());
  std::fprintf(g_save, "#        %s %s\n", dir, amr::describeFrame(body).c_str());
  std::fflush(g_save);
  ++g_saved;
}

void logFrame(int sess, const char* dir, const std::string& body) {
  const amr::FrameInfo fi = amr::frameInfo(body);
  const std::string key = std::string(dir) + " kind=" + std::to_string(fi.kind) +
                          " type=" + (fi.has_type ? std::to_string(fi.type) : std::string("-"));
  g_stats.add(key);
  const bool is_up = (std::strcmp(dir, "UP") == 0);
  const auto it = g_stats.prev.find(key);
  const bool repeat = (it != g_stats.prev.end());

  amr::State st;
  const bool is_state = amr::stateFromBody(body, st);

  if (is_up) {
    // Ramka z panelu: to sa komendy - pokazujemy zawsze w calosci + roznice wzgledem poprzedniej.
    std::printf("%s s%d %s  %s\n", stamp().c_str(), sess, dir, amr::describeFrame(body).c_str());
    if (g_opt.up_full) std::printf("%s", amr::dumpBody(body).c_str());
    if (repeat) {
      const std::vector<std::string> d = amr::diffBodies(it->second, body);
      if (d.empty()) std::printf("          (identyczna z poprzednia tego typu)\n");
      for (const std::string& s : d) std::printf("      zmiana: %s\n", s.c_str());
    }
  } else {
    if (is_state) {
      // Push stanu leci kilka razy na sekunde - pokazujemy tylko zmiany, jednym wierszem.
      amr::State before;
      const bool have_before = repeat && amr::stateFromBody(it->second, before);
      if (!have_before || before != st) {
        std::printf("%s s%d %s  stan: %s\n", stamp().c_str(), sess, dir, amr::stateLine(st).c_str());
      }
    } else {
      std::printf("%s s%d %s  %s\n", stamp().c_str(), sess, dir, amr::describeFrame(body).c_str());
      if (g_opt.down_full) std::printf("%s", amr::dumpBody(body).c_str());
      if (repeat) {
        const std::vector<std::string> d = amr::diffBodies(it->second, body);
        for (const std::string& s : d) std::printf("      zmiana: %s\n", s.c_str());
      }
    }
  }
  g_stats.prev[key] = body;
  saveFrame(dir, body);
}

std::vector<Session> g_sessions;

void closeSession(size_t i) {
  Session& s = g_sessions[i];
  if (s.cfd >= 0) ::close(s.cfd);
  if (s.sfd >= 0) ::close(s.sfd);
  std::printf("%s sesja %s zamknieta\n", stamp().c_str(), s.peer.c_str());
  s.cfd = s.sfd = -1;
}

}  // namespace

int main(int argc, char** argv) {
  std::string listen = "0.0.0.0:5002";
  std::string remote = "192.168.71.50:5002";

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--listen") listen = next();
    else if (a == "--remote") remote = next();
    else if (a == "--save") g_opt.save = next();
    else if (a == "--no-up-full") g_opt.up_full = false;
    else if (a == "--down-full") g_opt.down_full = true;
    else if (a == "--help" || a == "-h") {
      std::printf(
          "amr_proxy %s - podsłuch ruchu panel <-> baza (Matrix OS)\n"
          "\n"
          "  --listen HOST:PORT   gdzie ma nasluchiwac (domyslnie 0.0.0.0:5002)\n"
          "  --remote HOST:PORT   adres bazy (domyslnie 192.168.71.50:5002)\n"
          "  --save PLIK          zapisz wszystkie ramki (gotowe do amr_cmd --from-capture)\n"
          "  --down-full          pokazuj tez pelne drzewo ramek z bazy (poza stanem)\n"
          "  --no-up-full         nie pokazuj pelnego drzewa ramek z panelu (tylko naglowki)\n"
          "\n"
          "Panel ustaw na adres tego komputera, port 5002. Ctrl-C konczy i drukuje podsumowanie.\n",
          amr::kVersion);
      return 0;
    } else {
      std::fprintf(stderr, "Nieznany argument: %s\n", a.c_str());
      return 2;
    }
  }
  if (!parseHostPort(listen, g_opt.listen_host, g_opt.listen_port) ||
      !parseHostPort(remote, g_opt.remote_host, g_opt.remote_port)) {
    std::fprintf(stderr, "Adresy musza byc w postaci host:port\n");
    return 2;
  }

  if (!g_opt.save.empty()) {
    g_save = std::fopen(g_opt.save.c_str(), "ab");
    if (!g_save) {
      std::fprintf(stderr, "Nie moge otworzyc %s do zapisu\n", g_opt.save.c_str());
      return 2;
    }
    std::fprintf(g_save, "# --- amr_proxy %s, start %s ---\n", amr::kVersion, remote.c_str());
    std::fflush(g_save);
  }

  const int lfd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (lfd < 0) {
    std::fprintf(stderr, "socket: %s\n", std::strerror(errno));
    return 1;
  }
  const int one = 1;
  setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(static_cast<uint16_t>(g_opt.listen_port));
  inet_pton(AF_INET, g_opt.listen_host.c_str(), &sa.sin_addr);
  if (::bind(lfd, reinterpret_cast<sockaddr*>(&sa), sizeof sa) != 0 ||
      ::listen(lfd, 8) != 0) {
    std::fprintf(stderr, "bind/listen na %s:%d: %s\n", g_opt.listen_host.c_str(), g_opt.listen_port,
                 std::strerror(errno));
    return 1;
  }

  signal(SIGINT, onSigint);
  signal(SIGTERM, onSigint);
  std::printf("amr_proxy %s: nasluchuje na %s:%d, przekazuje do %s:%d\n", amr::kVersion,
              g_opt.listen_host.c_str(), g_opt.listen_port, g_opt.remote_host.c_str(), g_opt.remote_port);
  std::printf("Ustaw w panelu Matrix adres %s i port %d. Ctrl-C konczy.\n\n", g_opt.listen_host.c_str(),
              g_opt.listen_port);
  std::fflush(stdout);

  while (!g_stop) {
    std::vector<pollfd> fds;
    fds.push_back(pollfd{lfd, POLLIN, 0});
    for (const Session& s : g_sessions) {
      if (s.cfd >= 0) fds.push_back(pollfd{s.cfd, POLLIN, 0});
      if (s.sfd >= 0) fds.push_back(pollfd{s.sfd, POLLIN, 0});
    }
    const int pr = ::poll(fds.data(), fds.size(), 200);
    if (pr < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (pr == 0) continue;

    size_t k = 0;
    if (fds[k++].revents & POLLIN) {
      sockaddr_in ca{};
      socklen_t cl = sizeof ca;
      const int cfd = ::accept(lfd, reinterpret_cast<sockaddr*>(&ca), &cl);
      if (cfd >= 0) {
        char ip[64] = "?";
        inet_ntop(AF_INET, &ca.sin_addr, ip, sizeof ip);
        std::string err;
        const int sfd = connectTcp(g_opt.remote_host, g_opt.remote_port, err);
        if (sfd < 0) {
          std::printf("%s NOWE polaczenie z %s:%d, ale baza nie odpowiada: %s\n", stamp().c_str(), ip,
                      ntohs(ca.sin_port), err.c_str());
          ::close(cfd);
        } else {
          setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
          Session s;
          s.cfd = cfd;
          s.sfd = sfd;
          s.peer = std::string(ip) + ":" + std::to_string(ntohs(ca.sin_port));
          g_sessions.push_back(s);
          std::printf("%s NOWA sesja: %s (-> %s:%d)\n", stamp().c_str(), s.peer.c_str(),
                      g_opt.remote_host.c_str(), g_opt.remote_port);
          std::fflush(stdout);
        }
      }
    }
    for (size_t i = 0; i < g_sessions.size(); ++i) {
      Session& s = g_sessions[i];
      if (s.cfd < 0) continue;
      const short rc = (k < fds.size()) ? fds[k++].revents : 0;
      const short rs = (k < fds.size()) ? fds[k++].revents : 0;
      bool dead = false;
      if (rc & POLLIN) {
        char buf[8192];
        const ssize_t n = ::recv(s.cfd, buf, sizeof buf, 0);
        if (n <= 0) {
          dead = true;
        } else {
          std::vector<std::string> frames;
          s.up.feed(buf, static_cast<size_t>(n), frames);  // handshake obsluzy sam parser
          s.saw_handshake = s.saw_handshake || s.up.hs_done;
          for (const std::string& b : frames) logFrame(static_cast<int>(i), "UP", b);
          size_t off = 0;
          while (off < static_cast<size_t>(n) && s.sfd >= 0) {
            const ssize_t w = ::send(s.sfd, buf + off, static_cast<size_t>(n) - off, MSG_NOSIGNAL);
            if (w <= 0) { dead = true; break; }
            off += static_cast<size_t>(w);
          }
        }
      }
      if (!dead && (rc & (POLLHUP | POLLERR))) dead = true;
      if (!dead && s.sfd >= 0 && (rs & POLLIN)) {
        char buf[8192];
        const ssize_t n = ::recv(s.sfd, buf, sizeof buf, 0);
        if (n <= 0) {
          dead = true;
        } else {
          std::vector<std::string> frames;
          s.dn.feed(buf, static_cast<size_t>(n), frames);
          for (const std::string& b : frames) logFrame(static_cast<int>(i), "DN", b);
          size_t off = 0;
          while (off < static_cast<size_t>(n) && s.cfd >= 0) {
            const ssize_t w = ::send(s.cfd, buf + off, static_cast<size_t>(n) - off, MSG_NOSIGNAL);
            if (w <= 0) { dead = true; break; }
            off += static_cast<size_t>(w);
          }
        }
      }
      if (!dead && s.sfd >= 0 && (rs & (POLLHUP | POLLERR))) dead = true;
      if (dead) {
        closeSession(i);
        s.cfd = -1;  // zostaw wpis jako nieaktywny (przy kolejnych poll pomijany)
      }
    }
  }

  for (size_t i = 0; i < g_sessions.size(); ++i) {
    if (g_sessions[i].cfd >= 0) closeSession(i);
  }

  std::printf("\n--- podsumowanie ramek (dir kind type -> liczba) ---\n");
  for (const auto& kv : g_stats.counts) std::printf("  %-28s %ld\n", kv.first.c_str(), kv.second);
  if (g_save) {
    std::fprintf(g_save, "# --- podsumowanie ---\n");
    for (const auto& kv : g_stats.counts) std::fprintf(g_save, "#  %-28s %ld\n", kv.first.c_str(), kv.second);
    std::fprintf(g_save, "# --- koniec, zapisano %ld ramek ---\n", g_saved);
    std::fclose(g_save);
  }
  std::printf("Koniec.\n");
  return 0;
}
