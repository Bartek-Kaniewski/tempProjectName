// amr_state_cli - wypisuje pozycje i faze zadania bazy Helios (Matrix OS), tylko odczyt.
//
// Budowanie bez ROS:
//   g++ -std=c++17 -O2 -Iinclude src/amr_state_cli.cpp -o amr_state_cli
// Uzycie:
//   ./amr_state_cli                      # do Ctrl-C, linia przy kazdej zmianie
//   ./amr_state_cli --seconds 30 --all   # 30 s, kazdy push
//   ./amr_state_cli --ip 192.168.71.50 --user admin --pass-md5 <md5 hasla>
//   ./amr_state_cli --full               # dodatkowo pelne drzewo ostatniej wiadomosci stanu
//   ./amr_state_cli --segments           # dodatkowo wypisz odcinki zaplanowanej trasy (3.7.12)
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "amr_matrix.hpp"

int main(int argc, char** argv) {
  amr::MatrixClient::Options opt;
  double seconds = 0;
  bool all = false;
  bool full = false;
  bool segments = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--ip") opt.host = next();
    else if (a == "--port") opt.port = std::atoi(next().c_str());
    else if (a == "--user") opt.user = next();
    else if (a == "--pass-md5") opt.pass_md5 = next();
    else if (a == "--seconds") seconds = std::atof(next().c_str());
    else if (a == "--all") all = true;
    else if (a == "--full") full = true;
    else if (a == "--segments") segments = true;
    else {
      std::cerr << "Nieznany argument: " << a << "\n";
      return 2;
    }
  }

  amr::MatrixClient client(opt);
  std::string err;
  if (!client.connect(err)) {
    std::cerr << "Blad polaczenia z " << opt.host << ":" << opt.port << ": " << err << "\n";
    return 1;
  }
  std::cout << "Polaczono z " << opt.host << ":" << opt.port << ", login wyslany\n";

  const auto t0 = std::chrono::steady_clock::now();
  amr::State last;
  bool have = false;
  for (;;) {
    const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (seconds > 0 && el >= seconds) break;
    const auto r = client.poll(200, [&](const amr::State& s) {
      if (!have || all || s != last) {
        std::printf("[+%6.1fs] %s\n",
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
                    amr::stateLine(s).c_str());
        if (full) std::printf("%s", amr::dumpBody(s.raw).c_str());
        if (segments) {
          for (size_t i = 0; i < s.route.size(); ++i) {
            const amr::RouteSegment& g = s.route[i];
            std::printf("            odcinek[%zu]: (%lld, %lld) -> (%lld, %lld) mm\n", i,
                        static_cast<long long>(g.sx), static_cast<long long>(g.sy),
                        static_cast<long long>(g.ex), static_cast<long long>(g.ey));
          }
        }
        std::fflush(stdout);
        last = s;
        have = true;
      }
    });
    if (r == amr::Recv::Closed) {
      std::cerr << "Polaczenie zamkniete przez baze\n";
      return 3;
    }
  }
  return 0;
}
