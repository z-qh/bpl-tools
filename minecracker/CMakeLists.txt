cmake_minimum_required(VERSION 3.0.2)
project(minecracker)

add_compile_options(-std=c++17)

include_directories(
        include
)

add_library(mineMapBase SHARED src/MineMap.cpp)
target_include_directories(mineMapBase PUBLIC include)
