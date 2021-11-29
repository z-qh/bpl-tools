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
set(lso_LIBS
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



##
add_executable(demo src/demo.cc)
target_link_libraries(demo ${lso_LIBS})

## extract feature points -- feature
add_executable(feature src/featureExtraction.cpp)
target_link_libraries(feature ${lso_LIBS})

## extract ground and non-ground points -- image
add_executable(image src/imageProjection.cpp)
target_link_libraries(image ${lso_LIBS})

## mapping with struct info -- mapping
add_executable(mapping src/mapOptmization.cpp)
target_link_libraries(mapping ${lso_LIBS})

## extract sematic points -- sematic
add_executable(sematic src/sematicProjection.cpp)
target_link_libraries(sematic ${lso_LIBS})

## optimize the sematic landmark and poses -- lso
#add_executable(demo src/demo.cc)
#target_link_libraries(demo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${G2O_LIBS} gtsam)

##

