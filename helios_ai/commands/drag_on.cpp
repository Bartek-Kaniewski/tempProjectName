#include "drag_on.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>

#include "rokae/robot.h"
#include "rokae/data_types.h"

namespace commands {

namespace {

const std::string LEWA_IP  = "192.168.71.161";
const std::string PRAWA_IP = "192.168.71.160";
const std::string LOCAL_IP = "192.168.71.51";

/*
 * Czy drag ma działać BEZ trzymania przycisku na końcówce?
 *
 *   false = ramię stoi i rusza się TYLKO wtedy, gdy trzymasz przycisk na
 *           końcówce (末端按键). To jest ustawienie domyślne - ramię nie
 *           "lata" swobodnie, nie opada i nie przesuwa się samo.
 *
 *   true  = ramię daje się prowadzić od razu, bez trzymania czegokolwiek.
 *
 * Znaczenie tego argumentu jest wprost opisane w instrukcji ROKAE (SDK, enableDrag):
 *   "true  - 打开拖动功能之后可以直接拖动机器人，不需要按住末端按键"
 *         = "po włączeniu draga można prowadzić robota bezpośrednio, bez trzymania
 *            przycisku na końcówce"
 *   "false - 打开拖动功能之后需要按住末端按键才能拖动机器人"
 *         = "po włączeniu draga trzeba trzymać przycisk na końcówce, żeby prowadzić"
 *
 * Jeśli chcesz kiedyś przetestować wariant bez przycisku (np. do szybkiego
 * ustawienia ramienia), nie zmieniaj kodu - ustaw w terminalu:
 *      export HELIOS_DRAG_BEZ_GUZIKA=1
 */
const bool DRAG_BEZ_PRZYCISKU = [] {
    const char* value = std::getenv("HELIOS_DRAG_BEZ_GUZIKA");
    return value != nullptr && std::string(value) == "1";
}();

std::string nazwaZasilania(rokae::PowerState state) {
    switch (state) {
        case rokae::PowerState::on:
            return "on (上电 - silniki włączone)";
        case rokae::PowerState::off:
            return "off (下电 - silniki wyłączone)";
        case rokae::PowerState::estop:
            return "estop (WCIŚNIĘTY przycisk bezpieczeństwa)";
        case rokae::PowerState::gstop:
            return "gstop (otwarte drzwiczki / bariera bezpieczeństwa)";
        default:
            return "nieznany";
    }
}

std::string nazwaStanuPracy(rokae::OperationState state) {
    switch (state) {
        case rokae::OperationState::idle:
            return "idle (stoi)";
        case rokae::OperationState::jog:
            return "jog";
        case rokae::OperationState::rtControlling:
            return "sterowanie czasu rzeczywistego";
        case rokae::OperationState::drag:
            return "drag (DRAG WŁĄCZONY)";
        case rokae::OperationState::rlProgram:
            return "program RL";
        case rokae::OperationState::moving:
            return "ruch";
        case rokae::OperationState::jogging:
            return "jog w ruchu";
        default:
            return "inny";
    }
}

// Czeka, aż kontroler osiągnie zadany stan zasilania.
template <typename Robot>
bool czekajNaZasilanie(Robot& robot, rokae::PowerState oczekiwany, double sekundy) {
    const auto koniec = std::chrono::steady_clock::now() + std::chrono::duration<double>(sekundy);
    while (std::chrono::steady_clock::now() < koniec) {
        std::error_code ec;
        const rokae::PowerState stan = robot.powerState(ec);
        if (!ec && stan == oczekiwany) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

template <typename Robot>
bool enableDragForArm(Robot& robot, const std::string& armName) {
    std::error_code ec;

    std::cout << "[dragOn] " << armName
              << " - łączenie z kontrolerem...\n";

    robot.connectToRobot(ec);

    if (ec) {
        std::cerr << "[dragOn] " << armName
                  << " - błąd połączenia: "
                  << ec.message() << "\n";
        return false;
    }

    std::cout << "[dragOn] " << armName
              << " - połączono.\n";

    // 1. Tryb sterowania: zwykłe (nie-czasu-rzeczywistego) polecenia ruchu.
    ec.clear();
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    if (ec) {
        std::cout << "[dragOn] " << armName
                  << " - setMotionControlMode: " << ec.message()
                  << " (pomijam, dla draga nie jest krytyczne)\n";
    }

    // 2. Alarmy serwonapędów. Brak alarmu to nie błąd - nie przerywamy pracy.
    ec.clear();
    robot.clearServoAlarm(ec);
    if (ec) {
        std::cout << "[dragOn] " << armName
                  << " - clearServoAlarm: " << ec.message()
                  << " (normalne, gdy nie ma alarmu)\n";
    } else {
        std::cout << "[dragOn] " << armName
                  << " - alarmy serwonapędów skasowane.\n";
    }

    // 3. Stan zasilania PRZED zmianami.
    ec.clear();
    rokae::PowerState power = robot.powerState(ec);
    if (ec) {
        std::cerr << "[dragOn] " << armName << " - powerState(): " << ec.message() << "\n";
        return false;
    }
    std::cout << "[dragOn] " << armName
              << " - stan zasilania: " << nazwaZasilania(power) << "\n";

    // 3a. Przycisk bezpieczeństwa - trzeba go najpierw fizycznie zwolnić,
    //     a potem skasować stan awaryjny (recoverState(1) = 急停恢复).
    if (power == rokae::PowerState::estop) {
        std::cout << "[dragOn] " << armName
                  << " - ZWOLNIJ przycisk bezpieczeństwa (czerwony grzybek). "
                     "Próbuję skasować stan awaryjny...\n";
        ec.clear();
        robot.recoverState(1, ec);
        if (ec) {
            std::cerr << "[dragOn] " << armName
                      << " - nie udało się skasować stanu awaryjnego: " << ec.message()
                      << "\n          Zwolnij przycisk i spróbuj ponownie.\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        ec.clear();
        power = robot.powerState(ec);
        std::cout << "[dragOn] " << armName
                  << " - stan zasilania po skasowaniu: " << nazwaZasilania(power) << "\n";
    }

    // 3b. Otwarte drzwiczki / bariera - tego programem nie da się obejść.
    if (power == rokae::PowerState::gstop) {
        std::cerr << "[dragOn] " << armName
                  << " - zamknij drzwiczki / barierę bezpieczeństwa i spróbuj ponownie.\n";
        return false;
    }

    // 4. Tryb ręczny (manual) - konieczny do draga.
    ec.clear();
    robot.setOperateMode(rokae::OperateMode::manual, ec);
    if (ec) {
        std::cerr << "[dragOn] " << armName
                  << " - setOperateMode(manual): " << ec.message() << "\n";
        return false;
    }
    std::cout << "[dragOn] " << armName
              << " - tryb ręczny (manual) ustawiony.\n";

    // 5. *** NAJWAŻNIEJSZY PUNKT ***
    //    Drag wymaga stanu BEZ ZASILANIA (下电). Nie "włączamy zasilania" -
    //    wręcz odwrotnie, wyłączamy je, żeby kontroler przejął ramię w trybie draga.
    //    (W instrukcji ROKAE, przykład path_record.cpp:
    //     "打开拖动前置条件: 需要切换机器人操作模式为手动模式，并下电" =
    //     "warunek włączenia draga: tryb ręczny ORAZ stan bez zasilania".)
    ec.clear();
    robot.setPowerState(false, ec);
    if (ec) {
        std::cerr << "[dragOn] " << armName
                  << " - setPowerState(false): " << ec.message() << "\n";
        return false;
    }

    if (!czekajNaZasilanie(robot, rokae::PowerState::off, 5.0)) {
        ec.clear();
        const rokae::PowerState teraz = robot.powerState(ec);
        std::cerr << "[dragOn] " << armName
                  << " - ramię nie przeszło w stan \"bez zasilania\". Jest: "
                  << nazwaZasilania(teraz) << "\n";
        if (teraz == rokae::PowerState::estop) {
            std::cerr << "[dragOn] " << armName
                      << " - zwolnij przycisk bezpieczeństwa.\n";
        } else if (teraz == rokae::PowerState::gstop) {
            std::cerr << "[dragOn] " << armName
                      << " - zamknij drzwiczki / barierę.\n";
        }
        return false;
    }
    std::cout << "[dragOn] " << armName
              << " - zasilanie wyłączone (wymagane do draga).\n";

    // 6. Kasujemy wcześniej wysłane polecenia ruchu.
    //    (Oficjalny przykład draga w SDK robi to przed włączeniem draga.)
    ec.clear();
    robot.moveReset(ec);
    if (ec) {
        std::cout << "[dragOn] " << armName
                  << " - moveReset: " << ec.message() << " (pomijam)\n";
    }

    // 7. Włączamy drag.
    //    Ostatni argument decyduje o zachowaniu ramienia:
    //      false (nasze ustawienie) - ramię rusza się TYLKO gdy trzymasz przycisk
    //                                 na końcówce; puszczony przycisk = ramię stoi
    //      true                     - ramię daje się prowadzić swobodnie, bez przycisku
    ec.clear();
    robot.enableDrag(
        rokae::DragParameter::cartesianSpace,
        rokae::DragParameter::freely,
        ec,
        DRAG_BEZ_PRZYCISKU
    );

    if (ec) {
        std::cerr << "[dragOn] " << armName
                  << " - enableDrag: " << ec.message() << "\n";
        std::cerr << "[dragOn] " << armName
                  << " - sprawdź: tryb ręczny + bez zasilania + bez alarmów.\n";
        return false;
    }

    // 8. Weryfikacja: kontroler powinien zgłosić stan "drag".
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ec.clear();
    const rokae::OperationState stan = robot.operationState(ec);
    if (!ec && stan == rokae::OperationState::drag) {
        if (DRAG_BEZ_PRZYCISKU) {
            std::cout << "[dragOn] " << armName
                      << " - DRAG WŁĄCZONY (bez przycisku). Możesz prowadzić ramię ręką.\n";
        } else {
            std::cout << "[dragOn] " << armName
                      << " - DRAG WŁĄCZONY (z przyciskiem). Trzymaj przycisk na końcówce, "
                         "żeby prowadzić ramię.\n";
        }
    } else if (!ec) {
        std::cout << "[dragOn] " << armName
                  << " - polecenie wysłane, ale kontroler zgłasza stan: "
                  << nazwaStanuPracy(stan) << "\n";
    } else {
        std::cout << "[dragOn] " << armName
                  << " - polecenie wysłane (nie udało się odczytać stanu pracy).\n";
    }

    return true;
}

bool enableLeftDrag() {
    rokae::xMateErProRobot lewa(LEWA_IP, LOCAL_IP);
    return enableDragForArm(lewa, "Lewa ręka");
}

bool enableRightDrag() {
    rokae::xMateErProRobot prawa(PRAWA_IP, LOCAL_IP);
    return enableDragForArm(prawa, "Prawa ręka");
}

} // namespace

/*
 * Wersja bezargumentowa.
 *
 * #dragOn#
 *
 * Włącza drag dla obu ramion.
 */
void dragOn() {
    const bool leftOk = enableLeftDrag();
    const bool rightOk = enableRightDrag();

    if (leftOk && rightOk) {
        std::cout << "[dragOn] Tryb drag włączony dla obu ramion.\n";
    } else if (leftOk || rightOk) {
        std::cerr << "[dragOn] UWAGA: tryb drag włączony tylko dla jednego ramienia.\n";
    } else {
        std::cerr << "[dragOn] Nie udało się włączyć trybu drag.\n";
    }
}

/*
 * Wersja argumentowa.
 *
 * #dragOn:left#
 * #dragOn:right#
 * #dragOn:both#
 */
void dragOn(const std::string& arm) {
    if (arm == "left" ||
        arm == "lewa" ||
        arm == "lewej") {

        if (enableLeftDrag()) {
            std::cout << "[dragOn] Tryb drag włączony tylko dla lewej ręki.\n";
        } else {
            std::cerr << "[dragOn] Nie udało się włączyć draga lewej ręki.\n";
        }

        return;
    }

    if (arm == "right" ||
        arm == "prawa" ||
        arm == "prawej") {

        if (enableRightDrag()) {
            std::cout << "[dragOn] Tryb drag włączony tylko dla prawej ręki.\n";
        } else {
            std::cerr << "[dragOn] Nie udało się włączyć draga prawej ręki.\n";
        }

        return;
    }

    if (arm == "both" ||
        arm == "obie" ||
        arm == "both_arms") {

        dragOn();
        return;
    }

    std::cerr << "[dragOn] Nieznany argument: "
              << arm << "\n";

    std::cerr << "[dragOn] Dozwolone wartości: "
              << "left, right, both\n";
}

} // namespace commands
