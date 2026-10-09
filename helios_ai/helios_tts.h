#ifndef HELIOS_TTS_H
#define HELIOS_TTS_H

#include <string>

namespace helios_tts {
    // Funkcja odtwarzająca tekst na głos przy użyciu Pipera
    void speak(const std::string& text);
}

#endif // HELIOS_TTS_H