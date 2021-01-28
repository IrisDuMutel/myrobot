cmake_minimum_required(VERSION 3.0.2)
project(myrobot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    message_generation
    nodelet
    angles
    std_srvs
    std_srvs
    urdf
    dynamic_reconfigure
    rosgraph_msgs
    trajectory_msgs
    image_transport
    polled_camera
    diagnostic_updater
    camera_info_manager
    roscpp
    rospy
    gazebo_ros
    gazebo_plugins
    controller_manager
    joint_state_publisher
    robot_state_publisher
    rospy
    rviz
    std_msgs
    gazebo_msgs
    cv_bridge
    sensor_msgs
    nav_msgs
    genmsg 
    geometry_msgs
    rosconsole
    # openslam_gmapping 
    rosbag_storage
    kdl_parser 
    rostime  
    tf
    tf2_kdl
    tf2_ros
     

)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(OpenCV REQUIRED)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)

##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(DIRECTORY msg FILES vect_msg.msg)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   AddTwoInts.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages(DEPENDENCIES std_msgs nav_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )
add_definitions(
-DMODEL=simple_node -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTID01EQ=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0 -DROS_PROJECT -DMW_SCHED_OTHER -DSTACK_SIZE=64 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DRT=RT )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS plugins/light
                            plugins/contact
                            plugins/depth_camera
                            plugins/actuator
                            plugins/diff_drive
                            plugins/template
                            # plugins/pwm_vel
                            plugins/imu
                            plugins/lidar
                            plugins/gazebo_ros_motors
                            src/simple_node

                        
              CATKIN_DEPENDS 
                            message_runtime
                            rospy
                            nodelet
                            angles
                            std_srvs
                            geometry_msgs
                            nav_msgs
                            urdf
                            tf
                            tf2_ros
                            dynamic_reconfigure
                            rosgraph_msgs
                            trajectory_msgs
                            image_transport
                            camera_info_manager
                            gazebo_plugins 
                            gazebo_ros 
                            roscpp 
                            std_msgs
                            sensor_msgs
                            nav_msgs
                            gazebo_msgs
                            rosconsole
                            kdl_parser 
                            rostime
                            tf2_kdl
                            DEPENDS
                            orocos_kdl 
                            urdfdom_headers)

# Add directories in which the linker will look for libraries.
# link_directories([AFTER|BEFORE] directory1 [directory2 ...])
# Adds the paths in which the linker should search for libraries. Relative paths 
# given to this command are interpreted as relative to the current source directory

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

# Add include directories to the build.
# include_directories([AFTER|BEFORE] [SYSTEM] dir1 [dir2 ...])
# Add the given directories to those the compiler uses to search for include files. 
# Relative paths are interpreted as relative to the current source directory.

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} 
                    ${GAZEBO_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS}  ${OGRE-Terrain_INCLUDE_DIRS}
                    ${OGRE-Paging_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(plugins/light)
include_directories(plugins/contact)
include_directories(plugins/depth_camera)
include_directories(plugins/trackedvehicle)
include_directories(plugins/actuator)
include_directories(plugins/template)
include_directories(plugins/diff_drive)
# include_directories(plugins/pwm_vel)
include_directories(plugins/imu)
include_directories(plugins/lidar)
include_directories(plugins/dc_motor_model)
include_directories(src/simple_node)
include_directories(/usr/local/MATLAB/R2020b/toolbox/target/codertarget/rtos/inc)
include_directories(/usr/local/MATLAB/R2020b/toolbox/ros/codertarget/src)
include_directories(plugins/gazebo_ros_motors)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
                     ${PROJECT_SOURCE_DIR}
                     ${OpenCV_INCLUDE_DIRS}
)

## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/myrobot.cpp
# )
# add_library(<name> [STATIC | SHARED | MODULE]
#             [EXCLUDE_FROM_ALL]
#             [<source>...])
# Adds a library target called <name> to be built from the source files listed in the command invocation. 

# target_link_libraries(<target> ... <item>... ...)
# Specify libraries or flags to use when linking a given target and/or its dependents. Usage requirements from linked library 
# targets will be propagated. Usage requirements of a target’s dependencies affect compilation of its own sources.
# The named <target> must have been created by a command such as add_executable() or add_library()

# Build our plugin
add_library(contact_plugin SHARED plugins/contact/contact_plugin.cc)
target_link_libraries(contact_plugin  ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES}) # found in /home/iris/catkin_ws/devel/lib/libContactPlugin.so
add_library(light_sensor_plugin SHARED plugins/light/light_sensor_plugin.cpp)
target_link_libraries(light_sensor_plugin CameraPlugin ${catkin_LIBRARIES}) 
add_library(actuator_plugin SHARED plugins/actuator/actuator_plugin.cpp)
target_link_libraries(actuator_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(diff_drive_plugin SHARED plugins/diff_drive/diff_drive_plugin.cpp)
target_link_libraries(diff_drive_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_library(pwm_vel_plugin SHARED plugins/pwm_vel/pwm_vel_plugin.cpp)
# target_link_libraries(pwm_vel_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(template_plugin SHARED plugins/template/template_plugin.cpp)
target_link_libraries(template_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})###########template
add_library(depth_camera_plugin SHARED plugins/depth_camera/depth_camera_plugin.cpp)
target_link_libraries(depth_camera_plugin  DepthCameraPlugin ${catkin_LIBRARIES})
add_library(imu_plugin SHARED plugins/imu/imu_plugin.cpp)
target_link_libraries(imu_plugin  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(laser_plugin plugins/lidar/laser_plugin.cpp)
target_link_libraries(laser_plugin RayPlugin ${catkin_LIBRARIES})
add_library(custom_ros_dc_motor plugins/gazebo_ros_motors/gazebo_ros_dc_motor.cpp)
target_link_libraries(custom_ros_dc_motor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_library(Dc_motor_Dev SHARED src/dc_motor_model/ert_main.cpp)
# target_link_libraries(Dc_motor_Dev ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
## add_executable(random_moving src/random_moving.cpp)

# add_executable(camera_depth src/camera_depth.cpp)
# target_link_libraries(camera_depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## Declare executable
add_executable(simple_node
   src/simple_node/ert_main.cpp
   src/simple_node/simple_node.cpp
   src/simple_node/simple_node_data.cpp
   /usr/local/MATLAB/R2020b/toolbox/target/codertarget/rtos/src/linuxinitialize.cpp
   src/simple_node/slros_busmsg_conversion.cpp
   src/simple_node/slros_initialize.cpp
   /usr/local/MATLAB/R2020b/toolbox/ros/codertarget/src/slros_generic_param.cpp
)

add_dependencies(simple_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
## add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS scripts/realsense.py scripts/controller.py scripts/arbiter.py 
                               scripts/force.py scripts/light.py scripts/vector_fcn.py scripts/move_head.py
                               src/respawnGOAL.py scripts/myrobot_dqn_stage_1.py src/env_stage_1.py
                               scripts/action_graph.py scripts/result_graph.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# This command generates installation rules for a project. Install rules specified by calls to 
# the install() command within a source directory are executed in order during installation.

install(TARGETS
  actuator_plugin
  diff_drive_plugin
  # Dc_motor_Dev
  # pwm_vel_plugin
  # slam_gmapping
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


  install(DIRECTORY config launch rviz launch save_model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  gazebo_ros_dc_motor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_myrobot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

##add_dependencies(random_moving beginner_tutorials_generate_messages_cpp)