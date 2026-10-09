// amr_selftest - testy rdzenia protokolu (bez robota, bez sieci).
//
// Sprawdza: parser/serializer protobuf (zgodnosc bajt w bajt), koperta, patchowanie pol,
// dekodowanie stanu (pola 3.2 / 3.4 / 3.7 / 3.10), diff ramek, hex.
//
// Budowanie i uruchomienie: make test
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "amr_matrix.hpp"

static int g_fail = 0;

#define CHECK(cond)                                                              \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::printf("  BLAD: %s (linia %d)\n", #cond, __LINE__);                   \
      ++g_fail;                                                                  \
    }                                                                            \
  } while (0)

static amr::State makeState(int64_t x, int64_t y, int64_t yaw_mrad, int64_t phase, int64_t dist_cm,
                            const std::string& map) {
  // Buduje wiadomosc stanu (pole 3 tresci) i zwraca ja przez decodeState w tescie.
  std::vector<amr::Node> pose;
  pose.push_back(amr::varintNode(2, x));
  pose.push_back(amr::varintNode(3, y));
  pose.push_back(amr::varintNode(7, yaw_mrad));

  std::vector<amr::Node> seg;
  seg.push_back(amr::varintNode(2, 0));
  seg.push_back(amr::varintNode(3, -22));
  seg.push_back(amr::varintNode(4, 0));
  seg.push_back(amr::varintNode(5, -1532));

  std::vector<amr::Node> task;
  task.push_back(amr::varintNode(8, dist_cm));
  task.push_back(amr::stringNode(12, amr::serializeTree(seg)));

  std::vector<amr::Node> st;
  st.push_back(amr::varintNode(2, phase));
  st.push_back(amr::stringNode(4, amr::serializeTree(pose)));
  st.push_back(amr::stringNode(7, amr::serializeTree(task)));
  st.push_back(amr::stringNode(10, map));

  std::vector<amr::Node> payload;
  payload.push_back(amr::varintNode(1, 2));
  payload.push_back(amr::stringNode(3, amr::serializeTree(st)));

  // pelna ramka: koperta kind=2 + pola stanu
  const std::string body = amr::encodeEnvelope(2, 7, 0x1122334455, 5, amr::serializeTree(payload));
  amr::State s;
  const bool ok = amr::stateFromBody(body, s);
  CHECK(ok);
  return s;
}

int main() {
  std::printf("amr_selftest %s\n", amr::kVersion);

  // --- protobuf: hex -> drzewo -> bajty (zgodnosc 1:1)
  {
    const std::string raw = std::string("\x08\x96\x01\x12\x03""abc\x1d\x00\x00\x80\x3f", 13);
    std::vector<amr::Node> t;
    CHECK(amr::parseTree(raw, t));
    CHECK(amr::serializeTree(t) == raw);
    CHECK(t.size() == 3);
    CHECK(t[0].num == 1 && t[0].val == 150);
    CHECK(t[1].wire == 2 && t[1].str == "abc");
    CHECK(t[2].wire == 5);
  }

  // --- hex
  {
    std::string b;
    CHECK(amr::hexDecode("0a 12:34", b) && b.size() == 3 && b[0] == 0x0a && b[2] == 0x34);
    CHECK(!amr::hexDecode("abc", b));
    CHECK(amr::hexEncode("\x01\xff") == "01ff");
  }

  // --- patchowanie pol
  {
    std::vector<amr::Node> t;
    std::string err;
    CHECK(amr::applyPatch(t, "4.2.3=1200", err));
    CHECK(amr::applyPatch(t, "1=17", err));
    CHECK(amr::applyPatch(t, "10:s=mapa1", err));
    CHECK(amr::applyPatch(t, "6.1:z=-5", err));
    CHECK(amr::applyPatch(t, "7.1:f=1.5", err));
    CHECK(!amr::applyPatch(t, "4.2.3", err));       // brak '='
    CHECK(!amr::applyPatch(t, "4.2.3=abc", err));   // zla liczba
    const std::string ser = amr::serializeTree(t);
    std::vector<amr::Node> back;
    CHECK(amr::parseTree(ser, back));
    // 6.1 zigzag: -5 -> 9 (po sparsowaniu pole 6 trzeba rozwinac, bo drzewo czyta leniwie)
    bool found = false;
    for (const amr::Node& n : back) {
      if (n.num != 6 || n.wire != 2) continue;
      std::vector<amr::Node> kids;
      if (!amr::parseTree(n.str, kids)) continue;
      for (const amr::Node& k : kids) {
        if (k.num == 1 && k.wire == 0) found = (k.val == 9);
      }
    }
    CHECK(found);
  }

  // --- koperta i typ
  {
    std::vector<amr::Node> inner;
    inner.push_back(amr::varintNode(1, 17));
    const std::string body = amr::encodeEnvelope(0, 42, 0xabc, 4, amr::serializeTree(inner));
    const amr::FrameInfo fi = amr::frameInfo(body);
    CHECK(fi.ok && fi.kind == 0 && fi.seq == 42 && fi.session == 0xabc && fi.type == 17);
  }

  // --- dekodowanie stanu (jak push typu 2)
  {
    const amr::State s = makeState(1234, -5678, -1570, 7, 151, "map1");
    CHECK(std::fabs(s.x_mm - 1234) < 1e-9);
    CHECK(std::fabs(s.y_mm + 5678) < 1e-9);
    CHECK(std::fabs(s.yaw_rad + 1.570) < 1e-9);
    CHECK(s.phase == 7);
    CHECK(s.dist_raw == 151);
    CHECK(s.map == "map1");
    CHECK(s.route.size() == 1);
    if (s.route.size() == 1) {
      CHECK(s.route[0].sx == 0 && s.route[0].sy == -22 && s.route[0].ex == 0 && s.route[0].ey == -1532);
    }
  }

  // --- diff ramek
  {
    std::vector<amr::Node> a{{amr::varintNode(1, 1), amr::varintNode(2, 100)}};
    std::vector<amr::Node> b{{amr::varintNode(1, 1), amr::varintNode(2, 250)}};
    const std::vector<std::string> d = amr::diffBodies(amr::serializeTree(a), amr::serializeTree(b));
    CHECK(d.size() == 1);
    if (d.size() == 1) std::printf("  przyklad roznicy: %s\n", d[0].c_str());
  }

  // --- loginFrame: poprawna dlugosc prefiksu i dekodowalne pola
  {
    const std::string f = amr::loginFrame("admin", "21232f297a57a5a743894a0e4a801fc3");
    std::vector<amr::Node> t;
    CHECK(amr::parseTree(std::string_view(f).substr(4), t));
    const amr::FrameInfo fi = amr::frameInfo(std::string_view(f).substr(4));
    CHECK(fi.ok && fi.kind == 0 && fi.seq == 1);
  }

  // --- stateLine (czy sie nie wywala na dziwnych danych)
  {
    amr::State s;
    std::printf("  stateLine: %s\n", amr::stateLine(s).c_str());
  }

  std::printf(g_fail == 0 ? "OK: wszystkie testy przeszly\n" : "BLEDY: %d\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
