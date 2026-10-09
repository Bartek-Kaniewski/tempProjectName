#pragma once
#include <string>

namespace llm_commands {

// Przetwarza tekst od AI, wykrywa tagi #komenda# i uruchamia je w tle.
// Zwraca tekst oczyszczony z tagów (gotowy do wyświetlenia).
std::string process(const std::string& text);

} // namespace llm_commands