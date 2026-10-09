#include "helios_tts.h"
#include <iostream>
#include <cstdio>

namespace helios_tts {
    
    // Ścieżki i konfiguracja karty dźwiękowej
    const std::string PIPER_EXECUTABLE = "/home/std/piper/piper";
    const std::string MODEL_PATH = "/home/std/piper_models/pl_PL-darkman-medium.onnx";
    const std::string AUDIO_DEVICE = "plughw:CARD=REF,DEV=0";

    void speak(const std::string& text) {
        if (text.empty()) return;

        // Usuwamy cudzysłowy z wypowiedzi, żeby nie popsuć komendy bash
        std::string safe_text = text;
        size_t pos;
        while ((pos = safe_text.find("\"")) != std::string::npos) {
            safe_text.replace(pos, 1, " ");
        }

        // Budujemy dokładnie taką samą komendę, jaka zadziałała w terminalu
        std::string cmd = "echo \"" + safe_text + "\" | " + PIPER_EXECUTABLE + 
                          " --model " + MODEL_PATH + 
                          " --output-raw 2>/dev/null | aplay -D " + AUDIO_DEVICE + 
                          " -r 22050 -f S16_LE -t raw -q 2>/dev/null";

        FILE* pipe = popen(cmd.c_str(), "w");
        if (!pipe) {
            std::cerr << "[TTS Error] Nie można uruchomić procesu TTS.\n";
            return;
        }
        pclose(pipe);
    }
}