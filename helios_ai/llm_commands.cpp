#include "llm_commands.h"
#include "commands/wave.h"
#include "commands/drag_on.h"
#include "commands/drag_off.h"
#include "commands/stats.h"
#include "commands/youtube.h"
#include "commands/volume.h"
#include "commands/pos_zero.h"

#include <iostream>
#include <regex>
#include <thread>
#include <functional>
#include <unordered_map>

namespace llm_commands {

using NoArgCommand = std::function<void()>;
using ArgCommand = std::function<void(const std::string&)>;

static const std::unordered_map<std::string, NoArgCommand> COMMAND_REGISTRY = {
    {"wave", commands::wave},
    {"dragOff", commands::dragOff},
    {"stats", commands::stats}
};

static const std::unordered_map<std::string, ArgCommand> ARG_COMMAND_REGISTRY = {
    {"youtube", commands::youtube},
    {"volume", commands::volume},
    {"dragOn", commands::dragOn},
    {"posZero", commands::posZero}
};

std::string process(const std::string& text) {
    // Komendy z argumentem, np. #youtube:18 lat Axotox#
    const std::regex argCommandRegex(R"(#(\w+):([^#]+)#)");
    auto argBegin = std::sregex_iterator(text.begin(), text.end(), argCommandRegex);
    const auto argEnd = std::sregex_iterator();

    for (auto it = argBegin; it != argEnd; ++it) {
        const std::string command = (*it)[1].str();
        const std::string argument = (*it)[2].str();
        const auto found = ARG_COMMAND_REGISTRY.find(command);

        if (found != ARG_COMMAND_REGISTRY.end()) {
            std::cout << "\n  [Uruchamiam komendę: " << command
                      << ", argument: " << argument << "]\n";
            std::thread(found->second, argument).detach();
        } else {
            std::cout << "\n  [Nieznana komenda z argumentem: "
                      << command << "]\n";
        }
    }

    // Komendy bez argumentów, np. #wave#.
    const std::regex commandRegex(R"(#(\w+)#)");
    auto begin = std::sregex_iterator(text.begin(), text.end(), commandRegex);
    const auto end = std::sregex_iterator();

    for (auto it = begin; it != end; ++it) {
        const std::string command = (*it)[1].str();
        const auto found = COMMAND_REGISTRY.find(command);

        if (found != COMMAND_REGISTRY.end()) {
            std::cout << "\n  [Uruchamiam komendę: " << command << "]\n";
            std::thread(found->second).detach();
        } else {
            std::cout << "\n  [Nieznana komenda: " << command << "]\n";
        }
    }

    // Usuń zarówno tagi z argumentami, jak i zwykłe tagi.
    std::string cleaned = std::regex_replace(text, argCommandRegex, "");
    cleaned = std::regex_replace(cleaned, commandRegex, "");
    return cleaned;
}

} // namespace llm_commands
