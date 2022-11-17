cmake_minimum_required(VERSION 3.5)
project(bpltool)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_compile_options(-std=c++17)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        gnss_driver
        serial
        tf
        tf2
        tf2_geometry_msgs
        rosbag
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# opencv
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# ceres
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

# g2o
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)

# yaml
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

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

# libusb
find_package(libusb-1.0)

# GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)
find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# Qt
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# # Boost
find_package(Boost COMPONENTS REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories("/usr/include/boost")

# # glog
find_package(Gflags)
find_package(Glog)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

set(bpl_LIBS
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${G2O_LIBS}
        ${YAML_CPP_LIBRARIES}
        gtsam
)

# # Generate messages in the 'msg' folder
# add_message_files(
# FILES
# Message1.msg
# Message2.msg
# )

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
# generate_messages(
# DEPENDENCIES
# std_msgs  # Or other packages containing msgs
# )
catkin_package(
        CATKIN_DEPENDS
        rosserial_msgs

        # INCLUDE_DIRS include
        # LIBRARIES bpltool
        # CATKIN_DEPENDS other_catkin_pkg
        # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# add_executable(relocateTIM src/source/relocateTIM.cc)
# target_link_libraries(relocateTIM
# ${catkin_LIBRARIES}
# ${PCL_LIBRARIES}
# ${OpenCV_LIBS}
# )
# #
#
# add_executable(lidar2image src/source/lidar2image.cc)
# target_link_libraries(lidar2image
# ${catkin_LIBRARIES}
# ${OpenCV_LIBS}
# ${PCL_LIBRARIES}
# )
#

# #
add_executable(rs9sc src/example/RS9SC.cpp)
target_link_libraries(rs9sc ${bpl_LIBS})

add_executable(handleSM src/source/handleSM.cpp)
target_link_libraries(handleSM ${bpl_LIBS})
add_executable(transTool src/source/transTool.cpp)
target_link_libraries(transTool ${bpl_LIBS})
add_executable(kittiGPSTool src/source/kittiGPSTool.cpp)
target_link_libraries(kittiGPSTool ${bpl_LIBS})

# #
add_executable(avgerror src/source/avgerror.cpp)
target_link_libraries(avgerror ${bpl_LIBS})

# #
add_executable(kitti2bag src/source/kitti2bag.cc)
target_link_libraries(kitti2bag ${bpl_LIBS})

# #
add_executable(oxford2bag src/source/oxfordHandle.cpp)
target_link_libraries(oxford2bag ${bpl_LIBS})

#
add_executable(temp src/source/temp.cc)
target_link_libraries(temp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

#
add_executable(g2oexample src/source/g2oexample.cc)
target_link_libraries(g2oexample ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBS})

# #
add_executable(qParam src/qParam/main.cpp src/qParam/mainwindow.cpp src/qParam/mainwindow.h src/qParam/mainwindow.ui)
target_link_libraries(qParam PRIVATE Qt5::Widgets ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(imuIntegral src/source/imuIntegral.cc)
target_link_libraries(imuIntegral ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(visSematic src/source/visSematic.cc)
target_link_libraries(visSematic ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(elerealy src/example/elerealy.cc)
target_link_libraries(elerealy ${catkin_LIBRARIES} ${PCL_LIBRARIES} usb-1.0)

# #
add_executable(rand_map src/source/rand_map.cpp)
target_link_libraries(rand_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

# #
add_executable(ceresExample src/source/ceresExample.cpp)
target_link_libraries(ceresExample ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# #
add_executable(netCamera src/source/netCamera.cc)
target_link_libraries(netCamera ${catkin_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(binToBag src/source/binToBag.cc)
target_link_libraries(binToBag ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# #
add_executable(gtsamExample src/source/gtsamExample.cc)
target_link_libraries(gtsamExample ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} gtsam)

# #
add_executable(pca src/source/pca.cc)
target_link_libraries(pca ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(plumb src/source/plumb.cc)
target_link_libraries(plumb ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(xyziAndPC src/source/xyziAndPC.cc)
target_link_libraries(xyziAndPC ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# #
add_executable(seg_ground src/source/seg_ground.cc)
target_link_libraries(seg_ground ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(clusterCloud src/source/clusterCloud.cc)
target_link_libraries(clusterCloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# #
add_executable(featureExtract src/source/featureExtract.cc)
target_link_libraries(featureExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(getPCD src/source/getPCD.cc)
target_link_libraries(getPCD ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# # OLD
add_executable(gnss_get_posi src/example/gnss_get_posi.cpp)
target_link_libraries(gnss_get_posi ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(steer_test src/example/steer_test.cpp)
target_link_libraries(steer_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# #
add_executable(pole_release src/source/pole.cpp)
target_link_libraries(pole_release ${bpl_LIBS})

add_executable(pole_debug src/source/pole.cpp)
target_link_libraries(pole_debug ${bpl_LIBS})
target_compile_options(pole_debug PRIVATE -g)

# #
add_executable(testsc src/source/testsc.cpp)
target_link_libraries(testsc ${bpl_LIBS})
target_compile_options(testsc PUBLIC -g -O0)

# #
add_executable(plane src/source/plane.cpp)
target_link_libraries(plane ${bpl_LIBS})

# #
add_executable(mytest src/source/mytest.cpp)
target_link_libraries(mytest )
target_compile_options(mytest PRIVATE -g)
