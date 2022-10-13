cmake_minimum_required(VERSION 3.0.2)
project(sbalm)
SET(CMAKE_BUILD_TYPE Release)

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
        tf
        tf2
        tf2_geometry_msgs
        rosbag
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# # glog
find_package(Gflags)
find_package(Glog)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder

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

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        CATKIN_DEPENDS

        # INCLUDE_DIRS include
        # LIBRARIES sbalm
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
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Ceres_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${YAML_INCLUDE_DIRS}
)

set(LIB_OPENCV_IMGPROC_DIR /usr/lib/x86_64-linux-gnu) # LIB_OPENCV_IMGPROC_DIR 是libtbb库的路径，缺哪个库找哪个，问题描述会有路径
add_library(libtbb SHARED IMPORTED) # 添加库文件
set_target_properties(libtbb PROPERTIES IMPORTED_LOCATION ${LIB_OPENCV_IMGPROC_DIR}/libtbb.so.2) # 对标库文件 这里的LIB_OPENCV_IMGPROC_DIR就是第一行定义的路径

# INSMAP2
# add_library(INSMAP2 SHARED include/sba/basic.cpp
# include/sba/ins_map.cpp)
# target_link_libraries(INSMAP2 ${PCL_LIBRARIES} ${G2O_LIBS})
# target_include_directories(INSMAP2 PUBLIC include ${PCL_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})

# DBSCAN2
# add_library(DBSCAN2 SHARED include/dbscan/DBSCAN.cpp)
# target_link_libraries(DBSCAN2 ${PCL_LIBRARIES})
# target_include_directories(DBSCAN2 PUBLIC include ${PCL_INCLUDE_DIRS})

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single Cins_mapMake context
# # The recommended prefix ensures that target names across packages don't collide
# # USING
# add_executable(sbalm src/sba.cpp)
# target_link_libraries(sbalm
# ${catkin_LIBRARIES}
# ${PCL_LIBRARIES}
# ${CERES_LIBRARIES}
# ${OpenCV_LIBS}
# ${CHOLMOD_LIBRARIES} gtsam libtbb
# ${YAML_CPP_LIBRARIES}
# ${G2O_LIBS}
# DBSCAN2
# INSMAP2
# )

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries

# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
