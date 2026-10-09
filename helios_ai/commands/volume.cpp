#include "volume.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace commands {

void volume(const std::string& value) {
    try {
        std::size_t position = 0;
        const int percentage = std::stoi(value, &position);

        if (position != value.size() || percentage < 0 || percentage > 80) {
            std::cerr << "[G³oœnoœæ] Podaj wartoœæ od 0 do 80. Otrzymano: "
                      << value << "\n";
            return;
        }

        const std::string level = std::to_string(percentage) + "%";
        const pid_t pid = fork();

        if (pid < 0) {
            std::cerr << "[G³oœnoœæ] Nie mo¿na uruchomiæ amixer.\n";
            return;
        }

        if (pid == 0) {
            // Karta USB REF; u¿ywamy nazwy karty zamiast numeru 3,
            // poniewa¿ numer ALSA mo¿e zmieniæ siê po restarcie.
            execlp(
                "amixer",
                "amixer",
                "-c",
                "REF",
                "sset",
                "PCM",
                level.c_str(),
                static_cast<char*>(nullptr)
            );

            std::cerr << "[G³oœnoœæ] Nie znaleziono programu amixer.\n";
            _exit(127);
        }

        int status = 0;
        waitpid(pid, &status, 0);

        if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
            std::cout << "[G³oœnoœæ] Ustawiono PCM na "
                      << percentage << "%\n";
        } else {
            std::cerr << "[G³oœnoœæ] Nie uda³o siê ustawiæ g³oœnoœci karty REF.\n";
        }
    } catch (const std::exception&) {
        std::cerr << "[G³oœnoœæ] Nieprawid³owa wartoœæ: "
                  << value << "\n";
    }
}

} // namespace commands
