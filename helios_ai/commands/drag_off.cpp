#include "drag_off.h"
#include <iostream>
#include <memory>
#include "rokae/robot.h"
#include "rokae/data_types.h"

namespace commands {
    void dragOff() {
        const std::string LEWA_IP  = "192.168.71.161";
        const std::string PRAWA_IP = "192.168.71.160";
        const std::string LOCAL_IP = "192.168.71.51";

        std::error_code ec;

        // --- LEWE RAMIĘ ---
        rokae::xMateErProRobot lewa(LEWA_IP, LOCAL_IP);
        lewa.connectToRobot(ec);
        if (ec) {
            std::cerr << "[ERROR dragOff] Lewa ręka - błąd połączenia: " << ec.message() << "\n";
        } else {
            lewa.disableDrag(ec);
            if (ec) {
                std::cout << "Błąd wyłączania Drag Mode: " << ec.message() << "\n";
            }
        }

        // --- PRAWE RAMIĘ ---
        rokae::xMateErProRobot prawa(PRAWA_IP, LOCAL_IP);
        prawa.connectToRobot(ec);
        if (ec) {
            std::cerr << "[ERROR dragOff] Prawa ręka - błąd połączenia: " << ec.message() << "\n";
        } else {
            prawa.disableDrag(ec);
            if (ec) {
                std::cout << "Błąd wyłączania Drag Mode: " << ec.message() << "\n";
            }
        }
    }
}