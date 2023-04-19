cmake_minimum_required(VERSION 3.5)
project(slo)

# add_compile_options(-Wall -O0 -g)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        serial
        tf
        tf2
        tf2_geometry_msgs
        rosbag
)

# opencv
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# ceres
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

# yaml
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

# g2o
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(csparse REQUIRED)
# include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${G2O_INCLUDE_DIR})
include_directories("/usr/include/suitesparse")
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal
        g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_core
        g2o_interface g2o_solver_csparse
        g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d
        g2o_csparse_extension g2o_opengl_helper
        g2o_solver_dense g2o_stuff
        g2o_types_sclam2d g2o_parser
        g2o_solver_pcg g2o_types_data
        g2o_types_sim3 cxsparse)

# GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)


# # Boost
find_package(Boost COMPONENTS REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories("/usr/include/boost")

set(slo_LIBS
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${G2O_LIBS}
        ${YAML_CPP_LIBRARIES}
        ${Sophus_LIBRARIES}
        gtsam
        fmt
)

catkin_package(
        CATKIN_DEPENDS
        rosserial_msgs
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# add_executable(mytest src/test.cpp 
#                       include/io/file_reader.cpp 
#                       include/cloud_projection/feature.cpp
#                       include/odometry/odometry.cpp)
# target_link_libraries(mytest ${slo_LIBS})
# target_compile_options(mytest PRIVATE -Wall -O2)
