#include "wave.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include "rokae/robot.h"
#include "rokae/data_types.h"

namespace commands {

// ====================================================================
// PRĘDKOŚĆ GLOBALNA
// ====================================================================
static const double PREDKOSC_GLOBALNA = 0.5; 

// ====================================================================
// STRUKTURY POMOCNICZE
// ====================================================================
struct TorsoPose {
    std::array<double, 4> joints;
    std::array<double, 2> head;
};

static std::array<double, 7> deg2rad7(const std::array<double, 7>& deg) {
    std::array<double, 7> rad;
    for (size_t i = 0; i < 7; i++) rad[i] = deg[i] * M_PI / 180.0;
    return rad;
}

// ====================================================================
// PUNKTY TRAJEKTORII
// ====================================================================
static const std::vector<std::array<double, 7>> LEWA_RECE_JOINT_DEG = {
    {-18.53700, 87.43741, -112.00451, 32.77294, 95.16150, 5.14149, 31.47747},
    {-62.11220, 81.76084, -50.00089, 133.80490, 115.12505, 23.40126, 17.11380},
    {-48.75222, 86.42663, -99.68442, 112.55556, 104.40226, 26.36305, 34.79392},
    {-48.75228, 86.42666, -99.68436, 112.55553, 104.40233, 26.36299, 34.79393},
    {-48.75226, 86.42666, -99.68436, 112.55554, 104.40232, 26.36301, 34.79394},
    {-48.75228, 86.42667, -99.68435, 112.55553, 104.40232, 26.36299, 34.79394},
};

static const std::vector<std::array<double, 7>> PRAWA_RECE_JOINT_DEG = {
    {31.57353, 92.47008, 95.09949, 68.55369, -3.01022, 15.08918, 4.09287},
    {-18.96046, 0.15258, 65.67573, 33.63637, 14.49540, 26.17101, 0.92587},
    {5.37921, 72.00226, 12.36546, 87.51457, -4.34218, 15.21070, 3.40150},
    {16.84373, 82.65137, 27.08224, 72.53241, 2.71246, 15.19909, 1.50636},
    {54.64069, 103.64353, 59.53290, 32.41992, -4.19701, 54.60930, 3.54601},
    {38.24495, 96.13247, 32.27489, 36.47362, -30.50856, 55.50552, 23.30014},
    {44.01989, 92.42045, 27.49159, 38.28033, 11.03788, 15.26256, 9.97995},
};

static const std::vector<TorsoPose> TULOW_TRAJEKTORIA = {
    {{0.09892, 1.48937, -6.36630, -0.77294}, {10.98284, -0.42329}},
    {{-2.71312, -1.72648, -10.79361, 4.32171}, {37.18333, 6.15642}},
    {{-2.71194, -5.38175, -21.71761, -12.02508}, {-13.63789, 15.81666}},
    {{-3.46750, 5.38588, -2.72656, -3.60056}, {4.14445, -2.90474}}
};

static const std::array<double, 7> POZYCJA_ZERO_7 = {0, 0, 0, 0, 0, 0, 0};

// ====================================================================
// FUNKCJE POMOCNICZE: Przygotowanie, trajektorie, powrót
// ====================================================================
static bool przygotujRece(rokae::xMateErProRobot& robot, const std::string& nazwa) {
    std::error_code ec;
    robot.recoverState(1, ec);
    robot.clearServoAlarm(ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);
    if (ec) {
        std::cerr << "  [" << nazwa << "] BŁĄD zasilania: " << ec.message() << "\n";
        return false;
    }
    return true;
}

static bool przygotujTulow(rokae::PCB4Robot& robot, const std::string& nazwa) {
    std::error_code ec;
    robot.recoverState(1, ec);
    robot.recoverState(2, ec);
    robot.recoverState(3, ec);
    robot.clearServoAlarm(ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    robot.setTeachPendantMode(false, ec);
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setPowerState(true, ec);
    if (ec) {
        std::cerr << "  [" << nazwa << "] BŁĄD zasilania torsu: " << ec.message() << "\n";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return true;
}

static void czekajNaTulow(rokae::PCB4Robot& robot, const std::string&) {
    std::error_code ec;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    auto start_time = std::chrono::steady_clock::now();
    while (!ec) {
        if (robot.operationState(ec) != rokae::OperationState::moving) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(15)) break;
    }
}

static void wykonajTrajektorieRece(rokae::xMateErProRobot& robot, 
                                    const std::vector<std::array<double, 7>>& punkty_deg,
                                    const std::string& nazwa,
                                    double predkosc = PREDKOSC_GLOBALNA) {
    if (!przygotujRece(robot, nazwa)) return;
    std::error_code ec;
    auto rtCon = robot.getRtMotionController().lock();
    if (!rtCon) return;
    for (const auto& pt : punkty_deg) {
        rtCon->MoveJ(predkosc, robot.jointPos(ec), deg2rad7(pt));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

static void wykonajTrajektorieTulow(rokae::PCB4Robot& robot,
                                     const std::vector<TorsoPose>& punkty,
                                     const std::string& nazwa,
                                     double predkosc = PREDKOSC_GLOBALNA) {
    if (!przygotujTulow(robot, nazwa)) return;
    std::error_code ec;
    robot.moveReset(ec);
    for (const auto& pt : punkty) {
        rokae::JointPosition target_pos;
        target_pos.joints.resize(4);
        target_pos.external.resize(2);
        for (size_t j = 0; j < 4; j++) target_pos.joints[j] = pt.joints[j] * M_PI / 180.0;
        for (size_t j = 0; j < 2; j++) target_pos.external[j] = pt.head[j] * M_PI / 180.0;
        rokae::MoveAbsJCommand cmd(target_pos, predkosc * 100.0);
        std::string cmdID;
        robot.moveAppend(cmd, cmdID, ec);
        robot.moveStart(ec);
        czekajNaTulow(robot, nazwa);
    }
}

static void powrotDoZeraRece(rokae::xMateErProRobot& robot, const std::string& nazwa, double predkosc = PREDKOSC_GLOBALNA) {
    if (!przygotujRece(robot, nazwa)) return;
    std::error_code ec;
    auto rtCon = robot.getRtMotionController().lock();
    if (!rtCon) return;
    rtCon->MoveJ(predkosc, robot.jointPos(ec), POZYCJA_ZERO_7);
}

static void powrotDoZeraTulow(rokae::PCB4Robot& robot, const std::string& nazwa, double predkosc = PREDKOSC_GLOBALNA) {
    if (!przygotujTulow(robot, nazwa)) return;
    std::error_code ec;
    robot.moveReset(ec);
    rokae::JointPosition target_pos;
    target_pos.joints = {0, 0, 0, 0};
    target_pos.external = {0, 0};
    rokae::MoveAbsJCommand cmd(target_pos, predkosc * 100.0);
    std::string cmdID;
    robot.moveAppend(cmd, cmdID, ec);
    robot.moveStart(ec);
    czekajNaTulow(robot, nazwa);
}

// ====================================================================
// GŁÓWNA FUNKCJA KOMENDY: wave()
// ====================================================================
void wave() {
    const std::string LEWA_IP  = "192.168.71.161";
    const std::string PRAWA_IP = "192.168.71.160";
    const std::string TULOW_IP = "192.168.71.254";
    const std::string LOCAL_IP = "192.168.71.51";
    
    std::unique_ptr<rokae::xMateErProRobot> lewa, prawa;
    std::unique_ptr<rokae::PCB4Robot> tulow;
    
    try {
        std::error_code ec;
        std::cout << "[WAVE] Łączenie z robotami...\n";
        
        lewa = std::make_unique<rokae::xMateErProRobot>(LEWA_IP, LOCAL_IP);
        lewa->connectToRobot(ec);
        if (ec) throw std::runtime_error("Lewa ręka: " + ec.message());
        
        prawa = std::make_unique<rokae::xMateErProRobot>(PRAWA_IP, LOCAL_IP);
        prawa->connectToRobot(ec);
        if (ec) throw std::runtime_error("Prawa ręka: " + ec.message());
        
        tulow = std::make_unique<rokae::PCB4Robot>(TULOW_IP);
        tulow->connectToRobot(ec);
        if (ec) {
            std::cerr << "[WAVE] Tułów niedostępny: " << ec.message() << "\n";
            tulow.reset();
        }
        
        przygotujRece(*lewa, "LEWA");
        przygotujRece(*prawa, "PRAWA");
        if (tulow) przygotujTulow(*tulow, "TULOW");
        
        // Ruch do przodu
        {
            std::thread t1([&]{ wykonajTrajektorieRece(*lewa,  LEWA_RECE_JOINT_DEG,  "LEWA"); });
            std::thread t2([&]{ wykonajTrajektorieRece(*prawa, PRAWA_RECE_JOINT_DEG, "PRAWA"); });
            std::thread t3;
            if (tulow) t3 = std::thread([&]{ wykonajTrajektorieTulow(*tulow, TULOW_TRAJEKTORIA, "TULOW"); });
            t1.join(); t2.join();
            if (t3.joinable()) t3.join();
        }

        // Powrót do zera
        {
            std::thread t1([&]{ powrotDoZeraRece(*lewa,  "LEWA"); });
            std::thread t2([&]{ powrotDoZeraRece(*prawa, "PRAWA"); });
            std::thread t3;
            if (tulow) t3 = std::thread([&]{ powrotDoZeraTulow(*tulow, "TULOW"); });
            t1.join(); t2.join();
            if (t3.joinable()) t3.join();
        }
        
        std::cout << "[WAVE] Zakończone pomyślnie.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "[WAVE BŁĄD]: " << e.what() << "\n";
    }
}

} // namespace commands