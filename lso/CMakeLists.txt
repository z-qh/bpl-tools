cmake_minimum_required(VERSION 3.5)
project(lso)

add_compile_options(-std=c++17)

##
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        tf
        tf2
        tf2_geometry_msgs
        rosbag
        message_generation
        std_msgs
        nav_msgs
)

##OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##octomap
#find_package(octomap REQUIRED)
#include_directories(${ocotmap_INCLUDE_DIRS})

## opencv
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

## pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

## ceres
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

## g2o
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(csparse REQUIRED)
#include_directories(${CSPARSE_INCLUDE_DIR})
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
        g2o_types_sim3 cxsparse )

## GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)
find_package(GTSAMCMakeTools)

## glog
find_package(Gflags)
find_package(Glog)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

##

## LIBS
set(lso_EXTERN_INCLUDES
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Ceres_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)
set(lso_EXTERN_LIBS
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${G2O_LIBS}
        gtsam
        )

##
add_message_files(
        DIRECTORY msgs
        FILES
        cloud_info.msg
)
generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
)

##
catkin_package(
        CATKIN_DEPENDS
        message_runtime
        message_generation
        geometry_msgs
        std_msgs
        nav_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})


##Optimizer
add_library(Optimizer SHARED src/Optimizer.cpp)
target_link_libraries(Optimizer ${lso_EXTERN_LIBS})
target_include_directories(Optimizer PUBLIC include ${lso_EXTERN_INCLUDES} )
##Base Pose
##Frame
add_library(Base SHARED src/Base.cpp src/Frame.cpp src/SemanticMark.cpp src/Map.cpp )
target_link_libraries(Base ${lso_EXTERN_LIBS})
target_include_directories(Base PUBLIC include ${lso_EXTERN_INCLUDES} )
##Cluster CVC
add_library(cvc SHARED src/ClusterCVC.cpp)
target_link_libraries(cvc ${lso_EXTERN_LIBS})
target_include_directories(cvc PUBLIC include ${lso_EXTERN_INCLUDES} )
#
set(lso_IN_LIBS
        Base
        cvc
        Optimizer
        )

##fast gicp
#add_library(fast_gicp SHARED
#        src/fast_gicp/gicp/lsq_registration.cpp
#        src/fast_gicp/gicp/fast_gicp.cpp
#        src/fast_gicp/gicp/fast_gicp_st.cpp
#        src/fast_gicp/gicp/fast_vgicp.cpp)
#target_link_libraries(fast_gicp ${PCL_LIBRARIES})
#target_include_directories(fast_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

## Tracking
add_executable(tracking src/Tracking.cpp)
target_link_libraries(tracking ${lso_EXTERN_LIBS} ${lso_IN_LIBS})

## demo for test
add_executable(demo src/demo.cc)
target_link_libraries(demo ${lso_EXTERN_LIBS} ${lso_IN_LIBS})
##

