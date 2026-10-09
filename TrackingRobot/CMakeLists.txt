cmake_minimum_required(VERSION 3.16)

project(TrackingRobot VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Wymagane moduły Qt6: Quick oraz QuickControls2 (do przycisków, układów itp.)
find_package(Qt6 REQUIRED COMPONENTS Quick QuickControls2 WebSockets)

# ==========================================================
# KONFIGURACJA OPENCV
# ==========================================================
# Wskazanie ścieżki do rozpakowanego OpenCV na dysku C:
if(NOT DEFINED OpenCV_DIR)
    set(OpenCV_DIR "C:/msys64/mingw64/lib/cmake/opencv4")
endif()

find_package(OpenCV REQUIRED)

message(STATUS "--------------------------------------------------")
message(STATUS "OpenCV znalezione pomyślnie!")
message(STATUS "  Wersja: ${OpenCV_VERSION}")
message(STATUS "  Ścieżki include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "  Biblioteki: ${OpenCV_LIBS}")
message(STATUS "--------------------------------------------------")

qt_standard_project_setup(REQUIRES 6.5)

# Opcjonalny import designu z Figma/Qt Design Studio
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/importedcontent/CMakeLists.txt")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/importedcontent)
endif()

qt_add_executable(appTrackingRobot
    main.cpp
    videoreceiver.h
    videoreceiver.cpp
    videoitem.h
    videoitem.cpp
    posedetector.h
    posedetector.cpp
)

qt_add_qml_module(appTrackingRobot
    URI TrackingRobot
    QML_FILES
        Main.qml
        SOURCES videoreceiver.cpp
        SOURCES videoreceiver.h
        SOURCES videoitem.h
        SOURCES videoitem.cpp
        SOURCES posedetector.h
        SOURCES posedetector.cpp
        SOURCES motionfilter.h
        SOURCES robotclient.h
        SOURCES robotclient.cpp
)

# Właściwości aplikacji
set_target_properties(appTrackingRobot PROPERTIES
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE #FALSE = Włączona konsola, można podglądać jakie dane odbiera
)

# Dołączenie nagłówków OpenCV
target_include_directories(appTrackingRobot PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

# Podpięcie bibliotek Qt oraz OpenCV
target_link_libraries(appTrackingRobot PRIVATE
    Qt6::Quick
    Qt6::QuickControls2
    Qt6::WebSockets
    ${OpenCV_LIBS}
)

install(TARGETS appTrackingRobot
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)