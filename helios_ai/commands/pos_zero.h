#pragma once

#include <string>

namespace commands {
    // #posZero:left#      -> lewa rêka
    // #posZero:right#     -> prawa rêka
    // #posZero:both#      -> oba ramiona
    // #posZero:torso#     -> tu³ów
    // #posZero:all#       -> oba ramiona + tu³ów
    void posZero(const std::string& target);
}
