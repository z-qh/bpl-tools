cmake_minimum_required(VERSION 3.0.2)
project(minecracker)

add_compile_options(-std=c++11)

add_executable(tim_handle src/tim_handle.cc src/Scancontext.cpp)
target_link_libraries(tim_handle
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
        ${CHOLMOD_LIBRARIES} gtsam libtbb
        )

