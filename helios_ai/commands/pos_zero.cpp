#include "pos_zero.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "rokae/robot.h"
#include "rokae/data_types.h"

namespace commands {

namespace {

// ---------------------------------------------------------------------------
// Adresy modu³ów
// ---------------------------------------------------------------------------
const std::string LEWA_IP  = "192.168.71.161";
const std::string PRAWA_IP = "192.168.71.160";
const std::string TULOW_IP = "192.168.71.254";
const std::string LOCAL_IP = "192.168.71.51";

// ---------------------------------------------------------------------------
// Parametry ruchu
// ---------------------------------------------------------------------------
constexpr double MOVE_SPEED_MM_S = 200.0;  // prêdkoœæ koñcówki; mniejsza = bezpieczniejsza
constexpr double MOVE_ZONE_MM    = 0.0;    // 0 = dok³adne dojœcie do punktu (fine);

/*
 * Tryb pracy wymagany do wykonania MoveAbsJ.
 *
 * manual    - bezpieczniejszy (kontroler zwykle ogranicza prêdkoœæ),
 * automatic - ten tryb pokazuj¹ przyk³ady SDK i czêœæ kontrolerów przyjmuje
 *             nie-realtime'owe komendy ruchu tylko w nim.
 *
 * Jeœli moveAppend() zg³osi b³¹d, zmieñ tê jedn¹ liniê.
 */
constexpr rokae::OperateMode MOVE_OPERATE_MODE = rokae::OperateMode::manual;

// --- Oczekiwanie na zasilanie ---
constexpr int POWER_ON_TIMEOUT_MS = 10000;
constexpr int POWER_POLL_MS       = 200;

// --- Oczekiwanie na wykonanie ruchu ---
constexpr int    MOVE_START_TIMEOUT_MS = 5000;   // ile czekaæ, a¿ robot ruszy
constexpr int    MOVE_TIMEOUT_MS       = 60000;  // maksymalny czas ca³ego ruchu
constexpr int    MOVE_POLL_MS          = 100;
constexpr double MOVING_EPS_DEG        = 0.02;   // taka zmiana pozycji = ruch
constexpr int    STABLE_POLLS_REQUIRED = 3;      // tyle stabilnych odczytów = stan¹³
constexpr double ZERO_TOLERANCE_DEG    = 1.0;    // tolerancja dojœcia do zera

// ---------------------------------------------------------------------------
// Wykrywanie API draga.
//
// enableDrag/disableDrag s¹ w BaseCobot, a np. PCB4Robot (IndustrialRobot)
// ich nie ma - dlatego ka¿de u¿ycie musi byæ warunkowe.
// ---------------------------------------------------------------------------
template <typename T, typename = void>
struct HasDisableDrag : std::false_type {};

template <typename T>
struct HasDisableDrag<T, std::void_t<
    decltype(std::declval<T&>().disableDrag(std::declval<std::error_code&>()))
>> : std::true_type {};

// ---------------------------------------------------------------------------
// Pomocnicze obliczenia na pozycjach osi (std::array<double, DoF>)
// ---------------------------------------------------------------------------
template <typename JointArray>
double maxAbsDeg(const JointArray& joints) {
    double maximum = 0.0;
    for (const double value : joints) {
        maximum = std::max(maximum, std::fabs(value * 180.0 / M_PI));
    }
    return maximum;
}

template <typename JointArray>
double maxAbsDiffDeg(const JointArray& first, const JointArray& second) {
    double maximum = 0.0;
    const std::size_t count = std::min(first.size(), second.size());
    for (std::size_t i = 0; i < count; ++i) {
        maximum = std::max(maximum,
                           std::fabs((first[i] - second[i]) * 180.0 / M_PI));
    }
    return maximum;
}

template <typename JointArray>
void printJoints(const JointArray& joints) {
    std::cout << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        std::cout << "O" << i + 1 << ": "
                  << joints[i] * 180.0 / M_PI << "  ";
    }
    std::cout << std::defaultfloat;
}

const char* powerStateName(rokae::PowerState state) {
    switch (state) {
        case rokae::PowerState::on:    return "on";
        case rokae::PowerState::off:   return "off";
        case rokae::PowerState::estop: return "estop (wciœniêty wy³¹cznik awaryjny)";
        case rokae::PowerState::gstop: return "gstop (otwarta os³ona bezpieczeñstwa)";
        default:                       return "nieznany";
    }
}

// ---------------------------------------------------------------------------
// Zasilanie silników - bez niego kontroler nie przyjmie ruchu.
// ---------------------------------------------------------------------------
template <typename Robot>
bool ensureMotorPowerOn(Robot& robot, const std::string& moduleName) {
    std::error_code ec;
    const auto state = robot.powerState(ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - powerState(): " << ec.message() << "\n";
    } else {
        std::cout << "[posZero] " << moduleName
                  << " - zasilanie: " << powerStateName(state)
                  << " (wartoœæ " << static_cast<int>(state) << ")\n";

        if (state == rokae::PowerState::on) {
            return true;
        }

        if (state == rokae::PowerState::estop) {
            std::cerr << "[posZero] " << moduleName
                      << " - E-STOP wciœniêty. Zwolnij wy³¹cznik awaryjny i powtórz.\n";
            return false;
        }

        if (state == rokae::PowerState::gstop) {
            std::cerr << "[posZero] " << moduleName
                      << " - otwarta os³ona bezpieczeñstwa. Zamknij j¹ i powtórz.\n";
            return false;
        }
    }

    std::cout << "[posZero] " << moduleName
              << " - w³¹czam zasilanie silników (setPowerState(true))...\n";

    ec.clear();
    robot.setPowerState(true, ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - setPowerState(true): " << ec.message() << "\n";
    }

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(POWER_ON_TIMEOUT_MS);

    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POWER_POLL_MS));

        ec.clear();
        const auto current = robot.powerState(ec);
        if (!ec && current == rokae::PowerState::on) {
            std::cout << "[posZero] " << moduleName
                      << " - zasilanie w³¹czone (potwierdzone przez kontroler).\n";
            return true;
        }
    }

    std::cerr << "[posZero] " << moduleName << " - zasilanie nie w³¹czy³o siê w "
              << (POWER_ON_TIMEOUT_MS / 1000) << " s.\n";
    return false;
}

// ---------------------------------------------------------------------------
// Przy w³¹czonym dragu kontroler nie przyjmie komendy ruchu.
// Wo³ane tylko dla modu³ów, które w ogóle maj¹ drag (coboty).
// ---------------------------------------------------------------------------
template <typename Robot>
void disableDragIfActive(Robot& robot, const std::string& moduleName) {
    if constexpr (!HasDisableDrag<Robot>::value) {
        // np. tu³ów (PCB4Robot): brak API draga - nic nie robimy.
        return;
    } else {
        std::error_code ec;
        const auto state = robot.operationState(ec);

        if (ec || state != rokae::OperationState::drag) {
            return;
        }

        std::cout << "[posZero] " << moduleName
                  << " - drag by³ w³¹czony, wy³¹czam przed ruchem...\n";

        ec.clear();
        robot.disableDrag(ec);

        if (ec) {
            std::cerr << "[posZero] " << moduleName
                      << " - disableDrag: " << ec.message() << "\n";
        } else {
            std::cout << "[posZero] " << moduleName
                      << " - drag wy³¹czony.\n";
        }
    }
}

// ---------------------------------------------------------------------------
// Zatrzymanie ruchu po niepowodzeniu.
// ---------------------------------------------------------------------------
template <typename Robot>
void abortMotion(Robot& robot, const std::string& moduleName) {
    std::error_code ec;
    robot.stop(ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - stop(): " << ec.message() << "\n";
    }

    ec.clear();
    robot.moveReset(ec);   // odrzuca niewykonane komendy

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - moveReset(): " << ec.message() << "\n";
    }

    std::cout << "[posZero] " << moduleName << " - ruch zatrzymany.\n";
}

// ---------------------------------------------------------------------------
// Oczekiwanie na wykonanie ruchu.
//
// Œwiadomie NIE u¿ywamy queryEventInfo(): jego typ EventInfo to
// std::unordered_map<std::string, std::any> i GCC 11 z tego œrodowiska
// wywala na nim twardy b³¹d kompilacji (static_assert w bits/hashtable.h).
// Zamiast tego odpytujemy pozycje osi: najpierw czekamy, a¿ robot ruszy,
// potem a¿ przestanie siê poruszaæ.
// ---------------------------------------------------------------------------
template <typename Robot, typename JointArray>
bool waitUntilMotionFinished(Robot& robot, const std::string& moduleName,
                             const JointArray& startPosition) {
    const auto overallDeadline = std::chrono::steady_clock::now()
                               + std::chrono::milliseconds(MOVE_TIMEOUT_MS);
    const auto startDeadline   = std::chrono::steady_clock::now()
                               + std::chrono::milliseconds(MOVE_START_TIMEOUT_MS);

    bool moving = false;
    int stablePolls = 0;
    JointArray previous = startPosition;

    while (std::chrono::steady_clock::now() < overallDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(MOVE_POLL_MS));

        std::error_code ec;
        const auto current = robot.jointPos(ec);

        if (ec) {
            continue;
        }

        const double delta = maxAbsDiffDeg(current, previous);
        previous = current;

        if (!moving) {
            if (delta > MOVING_EPS_DEG) {
                moving = true;
                std::cout << "[posZero] " << moduleName << " - ruch trwa...\n";
            } else if (std::chrono::steady_clock::now() > startDeadline) {
                std::cerr << "[posZero] " << moduleName
                          << " - robot nie ruszy³ w "
                          << (MOVE_START_TIMEOUT_MS / 1000)
                          << " s (kontroler odrzuci³ komendê?).\n";
                return false;
            }
            continue;
        }

        if (delta <= MOVING_EPS_DEG) {
            if (++stablePolls >= STABLE_POLLS_REQUIRED) {
                return true;
            }
        } else {
            stablePolls = 0;
        }
    }

    std::cerr << "[posZero] " << moduleName
              << " - przekroczono maksymalny czas ruchu ("
              << (MOVE_TIMEOUT_MS / 1000) << " s).\n";
    return false;
}

// ---------------------------------------------------------------------------
// Ruch jednego modu³u do pozycji zerowej.
// ---------------------------------------------------------------------------
template <typename Robot>
bool moveModuleToZero(Robot& robot, const std::string& moduleName) {
    std::error_code ec;

    std::cout << "[posZero] " << moduleName
              << " - ³¹czenie z kontrolerem...\n";

    robot.connectToRobot(ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - b³¹d po³¹czenia: " << ec.message() << "\n";
        return false;
    }

    std::cout << "[posZero] " << moduleName << " - po³¹czono.\n";

    ec.clear();
    robot.clearServoAlarm(ec);

    if (ec) {
        // Nie przerywamy - przy wy³¹czonym zasilaniu kontroler czêsto odmawia
        // czyszczenia alarmów, a zasilanie w³¹czamy dopiero poni¿ej.
        std::cout << "[posZero] " << moduleName
                  << " - clearServoAlarm: " << ec.message() << "\n";
    } else {
        std::cout << "[posZero] " << moduleName
                  << " - alarmy serwo wyczyszczone.\n";
    }

    ec.clear();
    robot.setOperateMode(MOVE_OPERATE_MODE, ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - setOperateMode: " << ec.message() << "\n";
        return false;
    }

    std::cout << "[posZero] " << moduleName
              << " - tryb pracy ustawiony ("
              << static_cast<int>(MOVE_OPERATE_MODE) << ").\n";

    if (!ensureMotorPowerOn(robot, moduleName)) {
        return false;
    }

    disableDragIfActive(robot, moduleName);

    ec.clear();
    const auto startPosition = robot.jointPos(ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - jointPos(): " << ec.message() << "\n";
        return false;
    }

    std::cout << "[posZero] " << moduleName << " - obecna pozycja (stopnie): ";
    printJoints(startPosition);
    std::cout << "\n";

    if (maxAbsDeg(startPosition) <= ZERO_TOLERANCE_DEG) {
        std::cout << "[posZero] " << moduleName
                  << " - ju¿ w pozycji zerowej, ruch pominiêty.\n";
        return true;
    }

    ec.clear();
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - setMotionControlMode(NrtCommand): " << ec.message() << "\n";
        return false;
    }

    // MoveAbsJCommand przyjmuje JointPosition, a nie std::array.
    // JointPosition(size_t n, double v) ustawia n osi na wartoœæ v.
    const rokae::JointPosition zeroTarget(startPosition.size(), 0.0);
    const rokae::MoveAbsJCommand command(zeroTarget, MOVE_SPEED_MM_S, MOVE_ZONE_MM);

    ec.clear();
    robot.moveReset(ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - moveReset: " << ec.message() << "\n";
    }

    std::string cmdID;
    ec.clear();
    robot.moveAppend(command, cmdID, ec);   // przeci¹¿enie dla pojedynczej komendy

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - moveAppend: " << ec.message() << "\n";
        std::cerr << "[posZero] " << moduleName
                  << " - jeœli to b³¹d trybu pracy, ustaw MOVE_OPERATE_MODE "
                  << "na OperateMode::automatic na górze pos_zero.cpp.\n";
        return false;
    }

    std::cout << "[posZero] " << moduleName
              << " - jadê do pozycji zerowej (cmdID: " << cmdID
              << ", " << MOVE_SPEED_MM_S << " mm/s)...\n";

    ec.clear();
    robot.moveStart(ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - moveStart: " << ec.message() << "\n";
        return false;
    }

    if (!waitUntilMotionFinished(robot, moduleName, startPosition)) {
        abortMotion(robot, moduleName);
        return false;
    }

    ec.clear();
    const auto finalPosition = robot.jointPos(ec);

    if (ec) {
        std::cerr << "[posZero] " << moduleName
                  << " - jointPos() po ruchu: " << ec.message() << "\n";
        return false;
    }

    const double deviation = maxAbsDeg(finalPosition);

    std::cout << "[posZero] " << moduleName << " - pozycja po ruchu (stopnie): ";
    printJoints(finalPosition);
    std::cout << " (najwiêksze odchylenie: "
              << std::fixed << std::setprecision(2) << deviation << ")\n";
    std::cout << std::defaultfloat;

    if (deviation > ZERO_TOLERANCE_DEG) {
        std::cerr << "[posZero] " << moduleName
                  << " - UWAGA: odchylenie od zera przekracza "
                  << ZERO_TOLERANCE_DEG << " stopnia.\n";
        return false;
    }

    std::cout << "[posZero] " << moduleName
              << " - pozycja zerowa osi¹gniêta.\n";
    return true;
}

bool posZeroLeft() {
    rokae::xMateErProRobot lewa(LEWA_IP, LOCAL_IP);
    return moveModuleToZero(lewa, "Lewa rêka");
}

bool posZeroRight() {
    rokae::xMateErProRobot prawa(PRAWA_IP, LOCAL_IP);
    return moveModuleToZero(prawa, "Prawa rêka");
}

bool posZeroTorso() {
    rokae::PCB4Robot tulow(TULOW_IP);
    return moveModuleToZero(tulow, "Tu³ów");
}

} // namespace

/*
 * #posZero:left#
 * #posZero:right#
 * #posZero:both#   - oba ramiona
 * #posZero:torso#  - tu³ów
 * #posZero:all#    - oba ramiona + tu³ów
 */
void posZero(const std::string& target) {
    if (target == "left" || target == "lewa" || target == "lewej") {
        if (posZeroLeft()) {
            std::cout << "[posZero] Lewa rêka w pozycji zerowej.\n";
        } else {
            std::cerr << "[posZero] Nie uda³o siê ustawiæ lewej rêki w pozycji zerowej.\n";
        }
        return;
    }

    if (target == "right" || target == "prawa" || target == "prawej") {
        if (posZeroRight()) {
            std::cout << "[posZero] Prawa rêka w pozycji zerowej.\n";
        } else {
            std::cerr << "[posZero] Nie uda³o siê ustawiæ prawej rêki w pozycji zerowej.\n";
        }
        return;
    }

    if (target == "both" || target == "obie" || target == "both_arms") {
        const bool leftOk  = posZeroLeft();
        const bool rightOk = posZeroRight();

        if (leftOk && rightOk) {
            std::cout << "[posZero] Oba ramiona w pozycji zerowej.\n";
        } else {
            std::cerr << "[posZero] UWAGA: nie oba ramiona dosz³y do pozycji zerowej "
                      << "(lewa: " << (leftOk ? "ok" : "b³¹d")
                      << ", prawa: " << (rightOk ? "ok" : "b³¹d") << ").\n";
        }
        return;
    }

    if (target == "torso" || target == "tu³ów" || target == "tulow") {
        if (posZeroTorso()) {
            std::cout << "[posZero] Tu³ów w pozycji zerowej.\n";
        } else {
            std::cerr << "[posZero] Nie uda³o siê ustawiæ tu³owia w pozycji zerowej.\n";
        }
        return;
    }

    if (target == "all" || target == "wszystko" || target == "all_modules") {
        const bool leftOk  = posZeroLeft();
        const bool rightOk = posZeroRight();
        const bool torsoOk = posZeroTorso();

        if (leftOk && rightOk && torsoOk) {
            std::cout << "[posZero] Wszystkie modu³y w pozycji zerowej.\n";
        } else {
            std::cerr << "[posZero] UWAGA: nie wszystkie modu³y dosz³y do pozycji zerowej "
                      << "(lewa: " << (leftOk ? "ok" : "b³¹d")
                      << ", prawa: " << (rightOk ? "ok" : "b³¹d")
                      << ", tu³ów: " << (torsoOk ? "ok" : "b³¹d") << ").\n";
        }
        return;
    }

    std::cerr << "[posZero] Nieznany argument: " << target << "\n";
    std::cerr << "[posZero] Dozwolone wartoœci: left, right, both, torso, all\n";
}

} // namespace commands
