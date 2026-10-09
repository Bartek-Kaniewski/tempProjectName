// amr_matrix.hpp - klient Matrix OS (baza Helios) - wersja 0.2: ODCZYT + WYSYLANIE.
//
// Bez zadnych bibliotek poza libc/libstdc++ (WebSocket i protobuf zrobione recznie).
// Protokol ustalony z przechwyconego ruchu panelu Matrix (nie z dokumentacji producenta):
//   * ws://IP:5002/, ramka aplikacji = u32 big-endian (dlugosc) + protobuf; jedna ramka moze byc
//     rozcieta na kilka wiadomosci WebSocket,
//   * koperta: 1 = rodzaj, 2 = numer sekwencji (fixed32), 3 = sesja (fixed64), 4/5 = tresc,
//   * pierwsza wiadomosc to logowanie (konto + MD5 hasla), potem baza sama wysyla stan.
// Pola stanu (push typu 2): x = 3.4.2 [mm], y = 3.4.3 [mm], kat = 3.4.7 [1/1000 rad],
// dystans = 3.7.8 (1 jedn. = 10 mm = 1 cm), faza = 3.2, mapa = 3.10, trasa = 3.7.12[n].
//
// NOWE W 0.2 (to, co potrzebne, zeby ruszac robotem):
//   * drzewo protobuf (Node) + serializacja + patchowanie pol po sciezce ("3.4.2=1200"),
//   * podglad ramek do tekstu (dumpBody) i porownywanie ramek (diffBodies) - do szukania roznic
//     miedzy "nic nie robie" a "wciskam przod w panelu",
//   * sendBody / sendRequest / sendReplay - wysylanie ramek, w tym ODTWORZENIE ramki
//     przechwyconej od panelu (z podmiana seq/sesji na nasze).
#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <random>
#include <string>
#include <string_view>
#include <vector>

namespace amr {

inline constexpr const char* kVersion = "0.2";

// ------------------------------------------------------------------ protobuf (wire, odczyt)

struct Field {
  uint32_t num = 0;
  int wire = 0;
  uint64_t value = 0;       // varint / fixed32 / fixed64
  std::string_view bytes;   // pole z dlugoscia
};

inline bool readVarint(std::string_view b, size_t& i, uint64_t& out) {
  out = 0;
  for (int shift = 0; shift <= 63 && i < b.size(); shift += 7) {
    const uint8_t c = static_cast<uint8_t>(b[i++]);
    out |= static_cast<uint64_t>(c & 0x7F) << shift;
    if (!(c & 0x80)) return true;
  }
  return false;
}

inline bool parseMessage(std::string_view b, std::vector<Field>& out) {
  out.clear();
  size_t i = 0;
  while (i < b.size()) {
    uint64_t key;
    if (!readVarint(b, i, key)) return false;
    Field f;
    f.num = static_cast<uint32_t>(key >> 3);
    f.wire = static_cast<int>(key & 7);
    if (f.num == 0) return false;
    switch (f.wire) {
      case 0:
        if (!readVarint(b, i, f.value)) return false;
        break;
      case 1:  // little-endian host (x86/ARM Linux)
        if (i + 8 > b.size()) return false;
        std::memcpy(&f.value, b.data() + i, 8);
        i += 8;
        break;
      case 5: {
        if (i + 4 > b.size()) return false;
        uint32_t v;
        std::memcpy(&v, b.data() + i, 4);
        f.value = v;
        i += 4;
        break;
      }
      case 2: {
        uint64_t n;
        if (!readVarint(b, i, n) || n > b.size() - i) return false;
        f.bytes = b.substr(i, static_cast<size_t>(n));
        i += static_cast<size_t>(n);
        break;
      }
      default:
        return false;
    }
    out.push_back(f);
  }
  return true;
}

inline const Field* findField(const std::vector<Field>& v, uint32_t num, int wire) {
  for (const auto& f : v) {
    if (f.num == num && f.wire == wire) return &f;
  }
  return nullptr;
}

// Brak pola w protobuf = wartosc 0.
inline int64_t intField(const std::vector<Field>& v, uint32_t num) {
  const Field* f = findField(v, num, 0);
  return f ? static_cast<int64_t>(f->value) : 0;
}

inline std::string varintBytes(uint64_t v) {
  std::string s;
  do {
    uint8_t b = v & 0x7F;
    v >>= 7;
    if (v) b |= 0x80;
    s.push_back(static_cast<char>(b));
  } while (v);
  return s;
}

// ------------------------------------------------------------------ hex

inline std::string hexEncode(std::string_view b) {
  static const char* D = "0123456789abcdef";
  std::string s;
  s.reserve(b.size() * 2);
  for (const unsigned char c : b) {
    s.push_back(D[c >> 4]);
    s.push_back(D[c & 15]);
  }
  return s;
}

// Przyjmuje biale znaki, przecinki i dwukropki jako separatory; opcjonalny prefiks 0x.
inline bool hexDecode(std::string_view h, std::string& out) {
  out.clear();
  size_t i = 0;
  if (h.size() >= 2 && h[0] == '0' && (h[1] == 'x' || h[1] == 'X')) i = 2;
  int hi = -1;
  for (; i < h.size(); ++i) {
    const unsigned char c = static_cast<unsigned char>(h[i]);
    if (c == 'x' || c == 'X') { hi = -1; continue; }  // sklejone "0x" w srodku - pomijamy
    if (!std::isxdigit(c)) continue;
    const int d = (c <= '9') ? (c - '0') : ((c | 0x20) - 'a' + 10);
    if (hi < 0) {
      hi = d;
    } else {
      out.push_back(static_cast<char>((hi << 4) | d));
      hi = -1;
    }
  }
  return hi < 0;  // nieparzysta liczba cyfr = blad
}

// ------------------------------------------------------------------ protobuf: drzewo (zapis / podglad)

// Wersja mutowalna wiadomosci. wire: 0 varint, 1 fixed64, 2 dlugosc, 5 fixed32.
// Dla wire==2: albo surowe bajty w `str`, albo rozwiniete dzieci w `kids` (dzieci maja priorytet).
struct Node {
  uint32_t num = 0;
  int wire = 0;
  uint64_t val = 0;
  std::string str;
  std::vector<Node> kids;
};

inline Node varintNode(uint32_t num, int64_t v) {
  Node n;
  n.num = num;
  n.wire = 0;
  n.val = static_cast<uint64_t>(v);
  return n;
}

inline Node stringNode(uint32_t num, const std::string& s) {
  Node n;
  n.num = num;
  n.wire = 2;
  n.str = s;
  return n;
}

inline bool parseTree(std::string_view b, std::vector<Node>& out) {
  out.clear();
  size_t i = 0;
  while (i < b.size()) {
    uint64_t key;
    if (!readVarint(b, i, key)) return false;
    Node n;
    n.num = static_cast<uint32_t>(key >> 3);
    n.wire = static_cast<int>(key & 7);
    if (n.num == 0) return false;
    switch (n.wire) {
      case 0:
        if (!readVarint(b, i, n.val)) return false;
        break;
      case 1:
        if (i + 8 > b.size()) return false;
        std::memcpy(&n.val, b.data() + i, 8);
        i += 8;
        break;
      case 5: {
        if (i + 4 > b.size()) return false;
        uint32_t v;
        std::memcpy(&v, b.data() + i, 4);
        n.val = v;
        i += 4;
        break;
      }
      case 2: {
        uint64_t sz;
        if (!readVarint(b, i, sz) || sz > b.size() - i) return false;
        n.str.assign(b.substr(i, static_cast<size_t>(sz)));
        i += static_cast<size_t>(sz);
        break;
      }
      default:
        return false;
    }
    out.push_back(std::move(n));
  }
  return true;
}

inline std::string encodeNode(const Node& n) {
  std::string s = varintBytes((static_cast<uint64_t>(n.num) << 3) | static_cast<uint64_t>(n.wire));
  if (n.wire == 2) {
    if (!n.kids.empty()) {
      const std::string inner = [&] {
        std::string t;
        for (const Node& k : n.kids) t += encodeNode(k);
        return t;
      }();
      s += varintBytes(inner.size());
      s += inner;
    } else {
      s += varintBytes(n.str.size());
      s += n.str;
    }
    return s;
  }
  switch (n.wire) {
    case 0:
      s += varintBytes(n.val);
      break;
    case 1:
      s.append(reinterpret_cast<const char*>(&n.val), 8);
      break;
    case 5: {
      const uint32_t v = static_cast<uint32_t>(n.val);
      s.append(reinterpret_cast<const char*>(&v), 4);
      break;
    }
    default:
      break;
  }
  return s;
}

inline std::string serializeTree(const std::vector<Node>& v) {
  std::string s;
  for (const Node& n : v) s += encodeNode(n);
  return s;
}

// ------------------------------------------------------------------ patchowanie pol po sciezce

inline bool parseU64(const std::string& s, uint64_t& v) {
  if (s.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const unsigned long long x = std::strtoull(s.c_str(), &end, 0);
  if (errno != 0 || end == s.c_str() || *end != '\0') return false;
  v = x;
  return true;
}

inline bool splitDots(const std::string& s, std::vector<std::string>& out) {
  out.clear();
  size_t p = 0;
  while (true) {
    const size_t q = s.find('.', p);
    if (q == std::string::npos) {
      out.push_back(s.substr(p));
      break;
    }
    out.push_back(s.substr(p, q - p));
    p = q + 1;
  }
  for (const auto& e : out) {
    if (e.empty()) return false;
  }
  return true;
}

// Ustawia/podmienia pole w drzewie. Sciezka: "4.2.3=1200", typy: brak/v = varint, u = varint bez
// znaku, z = zigzag, s = tekst, f32 / f64 = liczby zmiennoprzecinkowe. Sciezka jest wzgledna do
// calego body (razem z koperta), wiec np. "5.3.4.2=1000" to x w wiadomosci stanu.
inline bool applyPatch(std::vector<Node>& root, const std::string& spec, std::string& err) {
  const size_t eq = spec.find('=');
  if (eq == std::string::npos) {
    err = "brak '=' w: " + spec;
    return false;
  }
  const std::string path = spec.substr(0, eq);
  const std::string value = spec.substr(eq + 1);
  std::vector<std::string> parts;
  if (!splitDots(path, parts)) {
    err = "zla sciezka: " + path;
    return false;
  }

  auto parseElem = [&](const std::string& el, uint64_t& num, char& type) -> bool {
    type = 'v';
    std::string numStr = el;
    const size_t c = el.find(':');
    if (c != std::string::npos) {
      numStr = el.substr(0, c);
      if (c + 1 >= el.size()) return false;
      type = el[c + 1];
    }
    return parseU64(numStr, num) && num > 0;
  };

  std::vector<Node>* lvl = &root;
  for (size_t i = 0; i + 1 < parts.size(); ++i) {
    uint64_t num = 0;
    char type = 'v';
    if (!parseElem(parts[i], num, type)) {
      err = "zly element sciezki: " + parts[i];
      return false;
    }
    Node* found = nullptr;
    for (Node& n : *lvl) {
      if (n.num == num && n.wire == 2) {
        found = &n;
        break;
      }
    }
    if (!found) {
      Node n;
      n.num = static_cast<uint32_t>(num);
      n.wire = 2;
      lvl->push_back(std::move(n));
      found = &lvl->back();
    }
    if (found->kids.empty() && !found->str.empty()) {
      std::vector<Node> kids;
      if (!parseTree(found->str, kids)) {
        err = "pole " + parts[i] + " nie jest komunikatorem (nie da sie wejsc glebiej)";
        return false;
      }
      found->kids = std::move(kids);
      found->str.clear();
    }
    lvl = &found->kids;
  }

  uint64_t num = 0;
  char type = 'v';
  if (!parseElem(parts.back(), num, type)) {
    err = "zly element sciezki: " + parts.back();
    return false;
  }
  Node* target = nullptr;
  for (Node& n : *lvl) {
    if (n.num == num) {
      target = &n;
      break;
    }
  }
  if (!target) {
    Node n;
    n.num = static_cast<uint32_t>(num);
    lvl->push_back(std::move(n));
    target = &lvl->back();
  } else if (!target->kids.empty()) {
    err = "pole " + std::to_string(num) + " ma podpola - nie podmieniam na wartosc";
    return false;
  }
  target->kids.clear();
  target->str.clear();
  target->val = 0;

  errno = 0;
  char* end = nullptr;
  switch (type) {
    case 'v':
    case 'i': {
      const long long v = std::strtoll(value.c_str(), &end, 0);
      if (errno != 0 || end == value.c_str() || *end != '\0') {
        err = "zla liczba (varint): " + value;
        return false;
      }
      target->wire = 0;
      target->val = static_cast<uint64_t>(v);
      break;
    }
    case 'u': {
      uint64_t v = 0;
      if (!parseU64(value, v)) {
        err = "zla liczba (u): " + value;
        return false;
      }
      target->wire = 0;
      target->val = v;
      break;
    }
    case 'z': {
      const long long v = std::strtoll(value.c_str(), &end, 0);
      if (errno != 0 || end == value.c_str() || *end != '\0') {
        err = "zla liczba (zigzag): " + value;
        return false;
      }
      target->wire = 0;
      target->val = (static_cast<uint64_t>(v) << 1) ^ static_cast<uint64_t>(v >> 63);
      break;
    }
    case 's':
      target->wire = 2;
      target->str = value;
      break;
    case 'f': {
      const float f = std::strtof(value.c_str(), &end);
      if (end == value.c_str() || *end != '\0') {
        err = "zla liczba (float): " + value;
        return false;
      }
      uint32_t u;
      std::memcpy(&u, &f, 4);
      target->wire = 5;
      target->val = u;
      break;
    }
    case 'd': {
      const double d = std::strtod(value.c_str(), &end);
      if (end == value.c_str() || *end != '\0') {
        err = "zla liczba (double): " + value;
        return false;
      }
      uint64_t u;
      std::memcpy(&u, &d, 8);
      target->wire = 1;
      target->val = u;
      break;
    }
    default:
      err = std::string("nieznany typ pola ':") + type + "' (dozwolone: v,u,z,s,f,d)";
      return false;
  }
  return true;
}

// ------------------------------------------------------------------ podglad i porownywanie ramek

inline bool looksPrintable(std::string_view b) {
  if (b.empty()) return false;
  for (const unsigned char c : b) {
    if (c < 0x20 || c == 0x7F) return false;
  }
  return true;
}

inline void dumpTree(const std::vector<Node>& v, std::string& out, int depth) {
  const std::string pad(static_cast<size_t>(depth) * 2, ' ');
  for (const Node& n : v) {
    out += pad;
    out += "#" + std::to_string(n.num) + " ";
    if (n.wire == 0) {
      out += "varint " + std::to_string(static_cast<long long>(n.val));
    } else if (n.wire == 1) {
      char buf[64];
      std::snprintf(buf, sizeof buf, " (0x%llx)", static_cast<unsigned long long>(n.val));
      out += "fixed64 " + std::to_string(n.val) + buf;
    } else if (n.wire == 5) {
      // float pokazujemy tylko, gdy wyglada sensownie (pole 2 to zwykle licznik, nie float)
      float f;
      const uint32_t t = static_cast<uint32_t>(n.val);
      std::memcpy(&f, &t, 4);
      std::string extra;
      if (f == 0.0f || (std::fabs(static_cast<double>(f)) >= 1e-6 &&
                        std::fabs(static_cast<double>(f)) <= 1e12 &&
                        static_cast<double>(f) > -1e12)) {
        char buf[64];
        std::snprintf(buf, sizeof buf, " [f=%.6g]", static_cast<double>(f));
        extra = buf;
      }
      out += "fixed32 " + std::to_string(n.val) + extra;
    } else if (n.wire == 2) {
      std::vector<Node> kids;
      if (!n.str.empty() && parseTree(n.str, kids) && !kids.empty()) {
        out += "msg (" + std::to_string(n.str.size()) + " B)\n";
        dumpTree(kids, out, depth + 1);
        continue;
      }
      if (looksPrintable(n.str)) {
        out += "str \"" + n.str + "\"";
      } else {
        const std::string h = hexEncode(n.str);
        out += "bytes[" + std::to_string(n.str.size()) + "] " +
               (h.size() > 64 ? h.substr(0, 64) + "..." : h);
      }
    }
    out += "\n";
  }
}

inline std::string dumpBody(std::string_view body) {
  std::vector<Node> t;
  if (!parseTree(body, t)) return "(nie da sie zdekodowac jako protobuf)";
  std::string out;
  dumpTree(t, out, 0);
  return out;
}

inline void flattenTree(const std::vector<Node>& v, const std::string& prefix,
                        std::vector<std::pair<std::string, std::string>>& out) {
  std::map<uint32_t, int> seen;
  for (const Node& n : v) {
    const int k = seen[n.num]++;
    const std::string head = prefix.empty() ? std::to_string(n.num) : prefix + "." + std::to_string(n.num);
    const std::string path = head + (k ? "[" + std::to_string(k) + "]" : "");
    if (n.wire == 2) {
      std::vector<Node> kids;
      if (!n.str.empty() && parseTree(n.str, kids) && !kids.empty()) {
        flattenTree(kids, path, out);
      } else if (looksPrintable(n.str)) {
        out.emplace_back(path, "\"" + n.str + "\"");
      } else {
        const std::string h = hexEncode(n.str);
        out.emplace_back(path, "hex:" + (h.size() > 32 ? h.substr(0, 32) + ".." : h));
      }
    } else {
      out.emplace_back(path, std::to_string(static_cast<long long>(n.val)));
    }
  }
}

// Roznice miedzy dwiema ramkami (po sciezkach pol). Puste = ramki identyczne.
inline std::vector<std::string> diffBodies(std::string_view a, std::string_view b, size_t limit = 32) {
  std::vector<std::string> diffs;
  std::vector<Node> ta, tb;
  if (!parseTree(a, ta) || !parseTree(b, tb)) {
    diffs.push_back("(nie da sie zdekodowac)");
    return diffs;
  }
  std::vector<std::pair<std::string, std::string>> fa, fb;
  flattenTree(ta, "", fa);
  flattenTree(tb, "", fb);
  std::map<std::string, std::string> ma, mb;
  for (auto& p : fa) ma[p.first] = p.second;
  for (auto& p : fb) mb[p.first] = p.second;
  for (const auto& p : mb) {
    const auto it = ma.find(p.first);
    if (it == ma.end()) {
      if (diffs.size() < limit) diffs.push_back("+ " + p.first + " = " + p.second);
    } else if (it->second != p.second) {
      if (diffs.size() < limit) diffs.push_back("~ " + p.first + ": " + it->second + " -> " + p.second);
    }
  }
  for (const auto& p : ma) {
    if (!mb.count(p.first) && diffs.size() < limit) diffs.push_back("- " + p.first + " = " + p.second);
  }
  return diffs;
}

// ------------------------------------------------------------------ koperta ramki

struct FrameInfo {
  bool ok = false;
  uint64_t kind = 0;
  uint32_t seq = 0;
  uint64_t session = 0;
  uint32_t type = 0;  // kod zadania/pushu = pole 1 tresci (pola 4 lub 5)
  uint32_t op = 0;    // opkod kanalu komend = pole 2 tresci pola 6 (tak panel wysyla jog: op=16)
  bool has_seq = false, has_session = false, has_type = false, has_op = false;
};

inline FrameInfo frameInfo(std::string_view body) {
  FrameInfo fi;
  std::vector<Node> t;
  if (!parseTree(body, t)) return fi;
  fi.ok = true;
  for (const Node& n : t) {
    if (n.num == 1 && n.wire == 0) {
      fi.kind = n.val;
    } else if (n.num == 2 && n.wire == 5) {
      fi.seq = static_cast<uint32_t>(n.val);
      fi.has_seq = true;
    } else if (n.num == 3 && n.wire == 1) {
      fi.session = n.val;
      fi.has_session = true;
    } else if ((n.num == 4 || n.num == 5) && n.wire == 2) {
      std::vector<Node> inner;
      if (parseTree(n.str, inner)) {
        for (const Node& m : inner) {
          if (m.num == 1 && m.wire == 0) {
            fi.type = static_cast<uint32_t>(m.val);
            fi.has_type = true;
          }
        }
      }
    } else if (n.num == 6 && n.wire == 2) {
      std::vector<Node> inner;
      if (parseTree(n.str, inner)) {
        for (const Node& m : inner) {
          if (m.num == 2 && m.wire == 0) {
            fi.op = static_cast<uint32_t>(m.val);
            fi.has_op = true;
          }
        }
      }
    }
  }
  return fi;
}

inline std::string describeFrame(std::string_view body) {
  const FrameInfo fi = frameInfo(body);
  if (!fi.ok) return "? (nie protobuf)";
  std::string s = "kind=" + std::to_string(fi.kind);
  if (fi.has_seq) s += " seq=" + std::to_string(fi.seq);
  if (fi.has_session) {
    char buf[32];
    std::snprintf(buf, sizeof buf, " sess=0x%llx", static_cast<unsigned long long>(fi.session));
    s += buf;
  }
  if (fi.has_type) s += " type=" + std::to_string(fi.type);
  if (fi.has_op) s += " op=" + std::to_string(fi.op);
  s += " len=" + std::to_string(body.size());
  return s;
}

inline std::string encodeEnvelope(uint64_t kind, uint32_t seq, uint64_t session, uint32_t content_field,
                                  const std::string& content) {
  std::string body = varintBytes((1u << 3) | 0u) + varintBytes(kind);
  body.push_back(static_cast<char>(0x15));  // pole 2, fixed32
  body.append(reinterpret_cast<const char*>(&seq), 4);
  body.push_back(static_cast<char>(0x19));  // pole 3, fixed64
  body.append(reinterpret_cast<const char*>(&session), 8);
  body += varintBytes((static_cast<uint64_t>(content_field) << 3) | 2u);
  body += varintBytes(content.size());
  body += content;
  return body;
}

// ------------------------------------------------------------------ stan bazy

struct RouteSegment {
  int64_t sx = 0, sy = 0, ex = 0, ey = 0;  // mm (kolejnosc pol przyjeta z 3.7.12 - do potwierdzenia)
  bool operator==(const RouteSegment& o) const { return sx == o.sx && sy == o.sy && ex == o.ex && ey == o.ey; }
  bool operator!=(const RouteSegment& o) const { return !(*this == o); }
};

struct State {
  double x_mm = 0, y_mm = 0, yaw_rad = 0;
  int64_t dist_raw = 0;  // 3.7.8: pozostaly dystans do celu, 1 jedn. = 10 mm (wniosek: 151->1 przy trasie 1510 mm)
  int64_t phase = 0;     // 3.2: faza zadania: 2 = spoczynek, 7 = jazda, 10 = start (pozostale nieznane)
  std::string map;       // 3.10
  std::vector<RouteSegment> route;  // 3.7.12[n]
  std::string raw;       // kopia calej wiadomosci stanu (3) - do zrzutu/weryfikacji pol

  bool operator==(const State& o) const {
    return x_mm == o.x_mm && y_mm == o.y_mm && yaw_rad == o.yaw_rad && dist_raw == o.dist_raw &&
           phase == o.phase && map == o.map && route == o.route;
  }
  bool operator!=(const State& o) const { return !(*this == o); }
};

// payload = pole 5 koperty. Stan maja: push typu 2 oraz odpowiedz na logowanie (seq 1).
inline bool decodeState(std::string_view payload, State& st) {
  std::vector<Field> top;
  if (!parseMessage(payload, top)) return false;
  if (const Field* kind = findField(top, 1, 0)) {
    if (kind->value != 2) return false;
  }
  const Field* s = findField(top, 3, 2);
  if (!s) return false;
  std::vector<Field> sf;
  if (!parseMessage(s->bytes, sf)) return false;

  State out;
  out.phase = intField(sf, 2);
  if (const Field* p = findField(sf, 4, 2)) {
    std::vector<Field> pf;
    if (parseMessage(p->bytes, pf)) {
      out.x_mm = static_cast<double>(intField(pf, 2));
      out.y_mm = static_cast<double>(intField(pf, 3));
      out.yaw_rad = static_cast<double>(intField(pf, 7)) / 1000.0;
    }
  }
  if (const Field* r = findField(sf, 7, 2)) {
    std::vector<Field> rf;
    if (parseMessage(r->bytes, rf)) {
      out.dist_raw = intField(rf, 8);
      for (const Field& seg : rf) {
        if (seg.num != 12 || seg.wire != 2) continue;
        std::vector<Field> sgf;
        if (!parseMessage(seg.bytes, sgf)) continue;
        std::vector<int64_t> vals;
        for (const Field& f : sgf) {
          if (f.wire == 0) vals.push_back(static_cast<int64_t>(f.value));
        }
        if (vals.size() >= 4) out.route.push_back({vals[0], vals[1], vals[2], vals[3]});
      }
    }
  }
  if (const Field* m = findField(sf, 10, 2)) out.map = std::string(m->bytes);
  out.raw.assign(s->bytes);  // ostatnia wiadomosc stanu (3)
  st = out;
  return true;
}

// Wyciaga stan z calej ramki (pola 5 koperty), jesli to push typu 2 / odpowiedz logowania.
inline bool stateFromBody(const std::string& body, State& st) {
  std::vector<Node> t;
  if (!parseTree(body, t)) return false;
  for (const Node& n : t) {
    if (n.num == 5 && n.wire == 2) {
      if (decodeState(n.str, st)) return true;
    }
  }
  return false;
}

// Jednolinijkowy opis stanu - wspolny dla wszystkich narzedzi.
inline std::string stateLine(const State& s) {
  char buf[256];
  std::snprintf(buf, sizeof buf, "x=%8.1f mm  y=%9.1f mm  kat=%7.4f rad  dystans=%5lld cm  faza=%-3lld  trasa[%zu]  mapa=%s",
                s.x_mm, s.y_mm, s.yaw_rad, static_cast<long long>(s.dist_raw),
                static_cast<long long>(s.phase), s.route.size(), s.map.c_str());
  return buf;
}


// ------------------------------------------------------------------ zapis ruchu z amr_proxy

// Wybor ramki z zapisu amr_proxy: linie "+<czas> UP|DN <hex>" (linie "#" to opisy).
// dir: "UP" (panel -> baza) lub "DN"; type: kod zadania (-1 = dowolny); nth: ktore wystapienie.
inline bool readCaptureSelect(const std::string& path, const std::string& dir, int type, int nth,
                              std::string& hex_out, std::string& info, std::string& err) {
  std::ifstream f(path);
  if (!f) {
    err = "nie moge otworzyc " + path;
    return false;
  }
  std::string line;
  int seen = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::vector<std::string> tok;
    std::string t;
    while (ss >> t) tok.push_back(t);
    if (tok.size() < 3) continue;  // oczekujemy: "+ <czas> DIR <hex>"
    if (tok[tok.size() - 2] != dir) continue;
    std::string body;
    if (!hexDecode(tok.back(), body)) continue;
    const FrameInfo fi = frameInfo(body);
    if (type >= 0 &&
        !((fi.has_type && static_cast<int>(fi.type) == type) ||
          (fi.has_op && static_cast<int>(fi.op) == type))) {
      continue;
    }
    if (++seen == nth) {
      hex_out = tok.back();
      info = describeFrame(body);
      return true;
    }
  }
  err = "nie znalazlem ramki dir=" + dir + (type >= 0 ? " type=" + std::to_string(type) : "") +
        " nr " + std::to_string(nth) + " w " + path;
  return false;
}

// ------------------------------------------------------------------ znaczniki/ramki Matrix

inline std::string matrixHeader(uint64_t kind, uint32_t seq, uint64_t session) {
  std::string h;
  h.push_back('\x08');
  h += varintBytes(kind);
  h.push_back('\x15');
  h.append(reinterpret_cast<const char*>(&seq), 4);
  h.push_back('\x19');
  h.append(reinterpret_cast<const char*>(&session), 8);
  return h;
}

inline std::string matrixFrame(const std::string& body) {
  const uint32_t n = static_cast<uint32_t>(body.size());
  std::string f;
  f.push_back(static_cast<char>((n >> 24) & 0xFF));
  f.push_back(static_cast<char>((n >> 16) & 0xFF));
  f.push_back(static_cast<char>((n >> 8) & 0xFF));
  f.push_back(static_cast<char>(n & 0xFF));
  return f + body;
}

inline std::string loginFrame(const std::string& user, const std::string& pass_md5_hex) {
  std::string cred;
  cred.push_back('\x0a');
  cred += varintBytes(user.size()) + user;
  cred.push_back('\x12');
  cred += varintBytes(pass_md5_hex.size()) + pass_md5_hex;
  cred += "\x18\x1e";
  std::string inner = std::string("\x08\x00", 2);
  inner.push_back('\x12');
  inner += varintBytes(cred.size()) + cred;
  std::string body = matrixHeader(0, 1, 0);
  body.push_back('\x22');
  body += varintBytes(inner.size()) + inner;
  return matrixFrame(body);
}

inline std::string requestFrame(uint32_t seq, uint64_t session, uint32_t type) {
  std::string inner;
  inner.push_back('\x08');
  inner += varintBytes(type);
  std::string body = matrixHeader(0, seq, session);
  body.push_back('\x22');
  body += varintBytes(inner.size()) + inner;
  return matrixFrame(body);
}

// ------------------------------------------------------------------ WebSocket (klient, ws://)

enum class Recv { Message, Timeout, Closed };

inline std::string base64(const std::string& in) {
  static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  for (; i + 2 < in.size(); i += 3) {
    const uint32_t v = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8) | uint8_t(in[i + 2]);
    out += T[(v >> 18) & 63];
    out += T[(v >> 12) & 63];
    out += T[(v >> 6) & 63];
    out += T[v & 63];
  }
  if (i + 1 == in.size()) {
    const uint32_t v = uint8_t(in[i]) << 16;
    out += T[(v >> 18) & 63];
    out += T[(v >> 12) & 63];
    out += "==";
  } else if (i + 2 == in.size()) {
    const uint32_t v = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8);
    out += T[(v >> 18) & 63];
    out += T[(v >> 12) & 63];
    out += T[(v >> 6) & 63];
    out += '=';
  }
  return out;
}

class WsClient {
 public:
  WsClient() : rng_(std::random_device{}()) {}
  ~WsClient() { shutdown(); }
  WsClient(const WsClient&) = delete;
  WsClient& operator=(const WsClient&) = delete;

  // gniazdo - do wlasnego poll() (np. razem ze stdin w amr_probe)
  int nativeFd() const { return fd_; }

  bool connect(const std::string& host, int port, const std::string& origin, int timeout_ms,
               std::string& err) {
    shutdown();
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    const std::string ps = std::to_string(port);
    if (getaddrinfo(host.c_str(), ps.c_str(), &hints, &res) != 0 || !res) {
      err = "nie moge rozwiazac adresu " + host;
      return false;
    }
    fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd_ < 0) {
      freeaddrinfo(res);
      err = std::string("socket: ") + std::strerror(errno);
      return false;
    }
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    const int one = 1;
    setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    if (::connect(fd_, res->ai_addr, res->ai_addrlen) != 0) {
      err = std::string("connect: ") + std::strerror(errno);
      freeaddrinfo(res);
      shutdown();
      return false;
    }
    freeaddrinfo(res);

    std::string key_raw(16, '\0');
    for (auto& c : key_raw) c = static_cast<char>(rng_() & 0xFF);
    const std::string req = "GET / HTTP/1.1\r\nHost: " + host + ":" + ps +
                            "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: " +
                            base64(key_raw) + "\r\nSec-WebSocket-Version: 13\r\nOrigin: " + origin +
                            "\r\n\r\n";
    if (!sendAll(req)) {
      err = "handshake: blad wysylania";
      shutdown();
      return false;
    }
    std::string hdr;
    size_t end = std::string::npos;
    while ((end = hdr.find("\r\n\r\n")) == std::string::npos) {
      char buf[1024];
      const ssize_t n = ::recv(fd_, buf, sizeof buf, 0);
      if (n <= 0 || hdr.size() > 16384) {
        err = "handshake: brak odpowiedzi";
        shutdown();
        return false;
      }
      hdr.append(buf, static_cast<size_t>(n));
    }
    if (hdr.compare(0, 12, "HTTP/1.1 101") != 0) {
      err = "handshake odrzucony: " + hdr.substr(0, hdr.find("\r\n"));
      shutdown();
      return false;
    }
    rx_ = hdr.substr(end + 4);  // ewentualne bajty ramek, ktore przyszly razem z naglowkiem
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);  // dalej czekamy przez poll()
    return true;
  }

  bool sendBinary(const std::string& data) { return sendFrame(0x2, data); }

  Recv receive(std::string& msg, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
      const int r = tryFrame(msg);
      if (r == 1) return Recv::Message;
      if (r < 0) return Recv::Closed;
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                            deadline - std::chrono::steady_clock::now())
                            .count();
      pollfd p{fd_, POLLIN, 0};
      const int pr = ::poll(&p, 1, static_cast<int>(std::max<long long>(0, left)));
      if (pr == 0) return Recv::Timeout;
      if (pr < 0) {
        if (errno == EINTR) continue;
        return Recv::Closed;
      }
      char buf[4096];
      const ssize_t n = ::recv(fd_, buf, sizeof buf, 0);
      if (n <= 0) return Recv::Closed;
      rx_.append(buf, static_cast<size_t>(n));
    }
  }

  void shutdown() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    rx_.clear();
    frag_.clear();
  }

 private:
  bool sendAll(const std::string& s) {
    size_t off = 0;
    while (off < s.size()) {
      const ssize_t n = ::send(fd_, s.data() + off, s.size() - off, MSG_NOSIGNAL);
      if (n <= 0) return false;
      off += static_cast<size_t>(n);
    }
    return true;
  }

  bool sendFrame(uint8_t opcode, const std::string& data) {
    if (fd_ < 0) return false;
    const size_t n = data.size();
    std::string f;
    f.push_back(static_cast<char>(0x80 | opcode));
    if (n < 126) {
      f.push_back(static_cast<char>(0x80 | n));
    } else if (n < 65536) {
      f.push_back(static_cast<char>(0x80 | 126));
      f.push_back(static_cast<char>(n >> 8));
      f.push_back(static_cast<char>(n & 0xFF));
    } else {
      f.push_back(static_cast<char>(0x80 | 127));
      for (int i = 7; i >= 0; --i) f.push_back(static_cast<char>((static_cast<uint64_t>(n) >> (8 * i)) & 0xFF));
    }
    uint8_t mask[4];
    const uint32_t r = rng_();
    std::memcpy(mask, &r, 4);
    f.append(reinterpret_cast<const char*>(mask), 4);
    for (size_t i = 0; i < n; ++i) f.push_back(static_cast<char>(static_cast<uint8_t>(data[i]) ^ mask[i & 3]));
    return sendAll(f);
  }

  // 1 = jest pelna wiadomosc, 0 = trzeba wiecej danych, -1 = polaczenie zamkniete
  int tryFrame(std::string& msg) {
    for (;;) {
      if (rx_.size() < 2) return 0;
      const uint8_t b0 = static_cast<uint8_t>(rx_[0]);
      const uint8_t b1 = static_cast<uint8_t>(rx_[1]);
      const bool fin = b0 & 0x80;
      const int op = b0 & 0x0F;
      const bool masked = b1 & 0x80;
      uint64_t len = b1 & 0x7F;
      size_t pos = 2;
      if (len == 126) {
        if (rx_.size() < 4) return 0;
        len = (static_cast<uint8_t>(rx_[2]) << 8) | static_cast<uint8_t>(rx_[3]);
        pos = 4;
      } else if (len == 127) {
        if (rx_.size() < 10) return 0;
        len = 0;
        for (int i = 0; i < 8; ++i) len = (len << 8) | static_cast<uint8_t>(rx_[2 + i]);
        pos = 10;
      }
      if (len > (64u << 20)) return -1;
      uint8_t mask[4] = {0, 0, 0, 0};
      if (masked) {
        if (rx_.size() < pos + 4) return 0;
        std::memcpy(mask, rx_.data() + pos, 4);
        pos += 4;
      }
      if (rx_.size() < pos + len) return 0;
      std::string payload = rx_.substr(pos, static_cast<size_t>(len));
      rx_.erase(0, pos + static_cast<size_t>(len));
      if (masked) {
        for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<char>(static_cast<uint8_t>(payload[i]) ^ mask[i & 3]);
      }
      switch (op) {
        case 0x0:
        case 0x1:
        case 0x2:
          if (op != 0x0) frag_.clear();
          frag_ += payload;
          if (fin) {
            msg.swap(frag_);
            frag_.clear();
            return 1;
          }
          break;
        case 0x8:
          sendFrame(0x8, payload.substr(0, 2));
          return -1;
        case 0x9:
          sendFrame(0xA, payload);
          break;
        default:
          break;  // pong i nieznane - ignoruj
      }
    }
  }

  int fd_ = -1;
  std::string rx_;
  std::string frag_;
  std::mt19937 rng_;
};

// ------------------------------------------------------------------ sesja Matrix

class MatrixClient {
 public:
  struct Options {
    std::string host = "192.168.71.50";
    int port = 5002;
    std::string user = "admin";
    // MD5 hasla w hex. Domyslny to skrot, ktory panel Matrix sam wysyla (MD5 z "admin").
    // Inne haslo: echo -n 'haslo' | md5sum
    std::string pass_md5 = "21232f297a57a5a743894a0e4a801fc3";
    int connect_timeout_ms = 3000;
    // Po odpowiedzi na logowanie (seq 1) wyslij te same trzy zapytania, co panel Matrix.
    bool bootstrap_requests = true;
    // Panel wysyla zapytanie typu 18 co ~3 s (heartbeat). 0 = nie wysylaj.
    int heartbeat_ms = 3000;
  };

  using StateCb = std::function<void(const State&)>;
  using FrameCb = std::function<void(const std::string& body)>;  // kazda ramka przychodzaca

  explicit MatrixClient(Options o) : opt_(std::move(o)) {}

  const Options& options() const { return opt_; }

  bool connect(std::string& err) {
    stream_.clear();
    session_ = 0;
    seq_ = 1;
    requests_sent_ = false;
    if (!ws_.connect(opt_.host, opt_.port, "http://" + opt_.host, opt_.connect_timeout_ms, err)) return false;
    if (!ws_.sendBinary(loginFrame(opt_.user, opt_.pass_md5))) {
      err = "nie udalo sie wyslac logowania";
      return false;
    }
    last_tx_ = std::chrono::steady_clock::now();  // pierwszy heartbeat dopiero po ~3 s
    return true;
  }

  void close() { ws_.shutdown(); }

  int nativeFd() const { return ws_.nativeFd(); }
  uint64_t session() const { return session_; }
  bool logged_in() const { return session_ != 0; }
  uint32_t nextSeq() { return ++seq_; }

  // Czeka najwyzej timeout_ms na dane; dla kazdego odebranego stanu wola on_state.
  Recv poll(int timeout_ms, const StateCb& on_state) { return pollAny(timeout_ms, nullptr, on_state); }

  // Jak wyzej, ale dodatkowo wola on_frame dla KAZDEJ ramki przychodzacej (body bez prefiksu).
  Recv pollAny(int timeout_ms, const FrameCb& on_frame, const StateCb& on_state) {
    maybeHeartbeat();
    std::string msg;
    const Recv r = ws_.receive(msg, timeout_ms);
    if (r != Recv::Message) return r;
    stream_ += msg;
    for (;;) {
      if (stream_.size() < 4) break;
      const uint32_t n = (static_cast<uint8_t>(stream_[0]) << 24) | (static_cast<uint8_t>(stream_[1]) << 16) |
                         (static_cast<uint8_t>(stream_[2]) << 8) | static_cast<uint8_t>(stream_[3]);
      if (n > (64u << 20)) {
        ws_.shutdown();
        return Recv::Closed;
      }
      if (stream_.size() < 4 + static_cast<size_t>(n)) break;
      const std::string body = stream_.substr(4, n);
      stream_.erase(0, 4 + static_cast<size_t>(n));
      handleFrame(body, on_frame, on_state);
    }
    return Recv::Message;
  }

  // Wysyla gotowe body (koperta bez 4-bajtowego prefiksu dlugosci).
  bool sendBody(const std::string& body) {
    last_tx_ = std::chrono::steady_clock::now();
    return ws_.sendBinary(matrixFrame(body));
  }

  // Zbuduj i wyslij zadanie kind=0: tresc (pole 4) = {1: type} + pola z `extra` (surowy protobuf).
  bool sendRequest(uint32_t type, const std::string& extra = {}, std::string* effective = nullptr) {
    std::vector<Node> inner;
    inner.push_back(varintNode(1, type));
    if (!extra.empty()) {
      std::vector<Node> e;
      if (!parseTree(extra, e)) return false;
      for (Node& n : e) inner.push_back(std::move(n));
    }
    const uint32_t seq = nextSeq();
    std::string body = encodeEnvelope(0, seq, session_, 4, serializeTree(inner));
    if (effective) *effective = body;
    return sendBody(body);
  }

  // ODTWORZENIE ramki przechwyconej od panelu: podmienia sekwencje i sesje na nasze, reszta bez zmian.
  bool sendReplay(const std::string& captured_body, std::string* effective = nullptr) {
    std::vector<Node> t;
    if (!parseTree(captured_body, t)) return false;
    for (Node& n : t) {
      if (n.num == 2 && n.wire == 5) {
        n.val = nextSeq();
      } else if (n.num == 3 && n.wire == 1) {
        n.val = session_;
      }
    }
    std::string body = serializeTree(t);
    if (effective) *effective = body;
    return sendBody(body);
  }

  // Jak panel: co heartbeat_ms bezczynnosci wyslij zapytanie typu 18.
  void maybeHeartbeat() {
    if (opt_.heartbeat_ms <= 0 || session_ == 0) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_tx_ >= std::chrono::milliseconds(opt_.heartbeat_ms)) sendRequest(18);
  }

  void handleFrame(const std::string& body, const FrameCb& on_frame, const StateCb& on_state) {
    std::vector<Field> env;
    if (!parseMessage(body, env)) {
      if (on_frame) on_frame(body);
      return;
    }
    const Field* seq = findField(env, 2, 5);
    if (const Field* s = findField(env, 3, 1)) {
      if (s->value != 0) session_ = s->value;
    }
    if (seq && seq->value == 1 && !requests_sent_ && opt_.bootstrap_requests) {
      requests_sent_ = true;
      sendRequest(1);
      sendRequest(17);
      sendRequest(4);
    }
    if (on_frame) on_frame(body);
    if (const Field* p = findField(env, 5, 2)) {
      State st;
      if (decodeState(p->bytes, st) && on_state) on_state(st);
    }
  }

  Options opt_;
  WsClient ws_;
  std::string stream_;
  uint64_t session_ = 0;
  uint32_t seq_ = 1;
  bool requests_sent_ = false;
  std::chrono::steady_clock::time_point last_tx_{};
};

}  // namespace amr
