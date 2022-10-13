cmake_minimum_required(VERSION 3.5)

project(qParam LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        sensor_msgs
        cv_bridge
)
find_package(OpenCV)
find_package(PCL REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(qParam
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

add_executable(QtKittiVisualizer
        main.cpp
        KittiConfig.cpp
        KittiConfig.h
        KittiDataset.cpp
        KittiDataset.h
        QtKittiVisualizer.cpp
        QtKittiVisualizer.h
        QtKittiVisualizer.ui
)

target_link_libraries(qParam
        PRIVATE Qt5::Widgets
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
)
