cmake_minimum_required(VERSION 2.8.3)
project(qt_joy)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )
find_package(Qt5 COMPONENTS Widgets REQUIRED)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES Base_control
        CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)

add_executable(Client
        src/main.cpp
        src/mainwindow.cpp
        src/mainwindow.h
        src/mainwindow.ui
        )
target_link_libraries(Client ${catkin_LIBRARIES})
target_link_libraries(Client PRIVATE Qt5::Widgets)

