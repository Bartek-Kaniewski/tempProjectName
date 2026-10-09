cmake_minimum_required(VERSION 3.8)
project(helios_ai)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# 1. Zależności ROS 2 i systemowe
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# -----------------------------------------------------------------------------
# 2. Ścieżki do Rokae SDK
# -----------------------------------------------------------------------------
set(SDK_DIR "/home/std/rokae_ws/src/rokae_ros2-main/rokae_hardware/sdk")
set(SDK_INCLUDE_DIR "${SDK_DIR}/include")
set(SDK_LIB_DIR "${SDK_DIR}/lib")

# -----------------------------------------------------------------------------
# 3. ONNX Runtime (detektor słowa-klucza "Helios")
#
#    Na Jetsonie (aarch64) NIE ma gotowego pakietu z nagłówkami w pipie.
#    Pobierz raz archiwum:
#      scripts/pobierz_onnxruntime.sh
#    i wskaż katalog przez -DONNXRUNTIME_ROOT=/home/std/onnxruntime
# -----------------------------------------------------------------------------
set(ONNXRUNTIME_ROOT "/home/std/onnxruntime" CACHE PATH
    "Katalog z include/onnxruntime_cxx_api.h oraz lib/libonnxruntime.so")

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include"
)
find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
)

# -----------------------------------------------------------------------------
# 4. Pliki źródłowe
# -----------------------------------------------------------------------------
file(GLOB COMMAND_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/commands/*.cpp")

set(HELIOS_SOURCES
  main.cpp
  llm_commands.cpp
  helios_tts.cpp
  speech_to_text.cpp
  mic.cpp
  wakeword.cpp          # detektor słowa-klucza (ONNX Runtime, openWakeWord)
  ${COMMAND_SOURCES}
)

add_executable(main ${HELIOS_SOURCES})

target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SDK_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)

target_compile_definitions(main PRIVATE XMATEMODEL_LIB_SUPPORTED)

# -----------------------------------------------------------------------------
# 5. Statyczne biblioteki Rokae SDK
# -----------------------------------------------------------------------------
add_library(xCoreSDK STATIC IMPORTED)
set_target_properties(xCoreSDK PROPERTIES
  IMPORTED_LOCATION "${SDK_LIB_DIR}/libxCoreSDK.a"
)

add_library(xMateModel STATIC IMPORTED)
set_target_properties(xMateModel PROPERTIES
  IMPORTED_LOCATION "${SDK_LIB_DIR}/libxMateModel.a"
  POSITION_INDEPENDENT_CODE ON
)

# -----------------------------------------------------------------------------
# 6. Linkowanie
# -----------------------------------------------------------------------------
ament_target_dependencies(main rclcpp)

target_link_libraries(main
  xCoreSDK
  xMateModel
  orocos-kdl
  Eigen3::Eigen
  CURL::libcurl
  nlohmann_json::nlohmann_json
  Threads::Threads
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  target_include_directories(main PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(main ${ONNXRUNTIME_LIB})
  message(STATUS "Helios: ONNX Runtime znaleziony -> ${ONNXRUNTIME_LIB}")
else()
  message(FATAL_ERROR
    "Nie znalazłem ONNX Runtime. Pobierz nagłówki i bibliotekę:\n"
    "  scripts/pobierz_onnxruntime.sh\n"
    "albo podaj katalog ręcznie: colcon build --cmake-args -DONNXRUNTIME_ROOT=/ścieżka")
endif()

# -----------------------------------------------------------------------------
# 7. Instalacja
#
#    INSTALL_RPATH wpisuje ścieżkę do libonnxruntime.so na stałe w program,
#    dzięki czemu "ros2 run helios_ai main" działa bez ręcznego ustawiania
#    LD_LIBRARY_PATH przy każdym uruchomieniu.
# -----------------------------------------------------------------------------
install(TARGETS main DESTINATION lib/${PROJECT_NAME})

if(ONNXRUNTIME_LIB)
  get_filename_component(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_LIB}" DIRECTORY)
  set_target_properties(main PROPERTIES
    INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
endif()

ament_package()
