cmake_minimum_required(VERSION 3.16)
project(PicoGoControlPanel VERSION 2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Network)
find_package(OpenCV REQUIRED)

add_executable(PicoGoControlPanel
    main.cpp
    MainWindow.h
    MainWindow.cpp
)

target_include_directories(PicoGoControlPanel PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(PicoGoControlPanel PRIVATE
    Qt6::Widgets
    Qt6::Network
    ${OpenCV_LIBS}
)

if(WIN32)
    set_target_properties(PicoGoControlPanel PROPERTIES WIN32_EXECUTABLE TRUE)
endif()