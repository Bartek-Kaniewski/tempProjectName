# Helios / Matrix OS - narzedzia do bazy (C++17, bez zewnetrznych bibliotek)
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Iinclude
BIN      := amr_state_cli amr_probe amr_proxy amr_cmd amr_wasd amr_selftest

all: $(BIN)

amr_state_cli: src/amr_state_cli.cpp include/amr_matrix.hpp
	$(CXX) $(CXXFLAGS) $< -o $@

amr_probe: src/amr_probe.cpp include/amr_matrix.hpp
	$(CXX) $(CXXFLAGS) $< -o $@

amr_proxy: src/amr_proxy.cpp include/amr_matrix.hpp
	$(CXX) $(CXXFLAGS) $< -o $@

amr_cmd: src/amr_cmd.cpp include/amr_matrix.hpp
	$(CXX) $(CXXFLAGS) $< -o $@

amr_wasd: src/amr_wasd.cpp include/amr_matrix.hpp
	$(CXX) $(CXXFLAGS) $< -o $@

amr_selftest: src/amr_selftest.cpp include/amr_matrix.hpp
	$(CXX) $(CXXFLAGS) $< -o $@

# Testy rdzenia protokolu (bez robota): parser, koperta, patchowanie, dekodowanie stanu
test: amr_selftest
	./amr_selftest

# Lawka testowa: atrapa bazy na porcie 5003
bench:
	python3 tools/bench_server.py --port 5003 --auto-mission

clean:
	rm -f $(BIN)

.PHONY: all test bench clean
