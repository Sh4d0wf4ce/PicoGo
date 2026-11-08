cmake_minimum_required(VERSION 3.16)
project(PicoGoControlPanel VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_AUTOMOC_COMPILER_PREDEFINES OFF)

find_package(Qt6 REQUIRED COMPONENTS Widgets Network)

add_executable(PicoGoControlPanel
    main.cpp
    MainWindow.h
    MainWindow.cpp
)

target_link_libraries(PicoGoControlPanel PRIVATE
    Qt6::Widgets
    Qt6::Network
)

if(WIN32)
    # Ukrycie okna konsoli w tle (opcjonalnie, usuń WIN32 jeśli chcesz widzieć stdout)
    set_target_properties(PicoGoControlPanel PROPERTIES WIN32_EXECUTABLE OFF)
endif()