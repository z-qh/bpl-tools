cmake_minimum_required(VERSION 3.0.2)
project(rand_map)
##C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
##添加依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
)
##引入外部依赖的包
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
##为本身构建依赖
catkin_package(
    INCLUDE_DIRS
)
##添加头文件目录
include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)
##生成可执行文件
add_executable(rand_map src/rand_map.cpp)
##链接库
target_link_libraries(rand_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})  
