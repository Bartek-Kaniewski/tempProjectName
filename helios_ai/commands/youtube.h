#pragma once

#include <string>

namespace commands {

// Wyszukuje zapytanie w YouTube Data API i uruchamia znaleziony wynik w mpv.
void youtube(const std::string& query);

} // namespace commands
