cmake_minimum_required(VERSION 2.8.3)
project(cicp_slam)
get_filename_component(EXTERNAL_LIBRARIES ../../ ABSOLUTE)
#set(CMAKE_INSTALL_RPATH ${EXTERNAL_LIBRARIES}/external_libraries/g2o_master/install_dir/lib)
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
#LIST( APPEND CMAKE_MODULE_PATH ${EXTERNAL_LIBRARIES}/external_libraries/g2o_master/cmake_modules )
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "DEBUG")
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(PCL_DIR ${EXTERNAL_LIBRARIES}/external_libraries/pcl_1_9/install_dir/share/pcl-1.9)
#set(G2O_ROOT ${EXTERNAL_LIBRARIES}/external_libraries/g2o_master/install_dir/include)
#list(APPEND CMAKE_MODULE_PATH "${EXTERNAL_LIBRARIES}/external_libraries/eigen_3_3_7/install_dir/share/eigen3/cmake")
find_package(Eigen3 REQUIRED PATHS ${EXTERNAL_LIBRARIES}/external_libraries/eigen_3_3_7/install_dir/share/eigen3/cmake NO_DEFAULT_PATH)
find_package(PCL 1.9 REQUIRED PATHS ${EXTERNAL_LIBRARIES}/external_libraries/pcl_1_9/install_dir/share/pcl-1.9 NO_DEFAULT_PATH )
set(G2O_INCLUDE_DIR 
  #g2o/core/base_vertex.h
  #${G2O_ROOT}/include
  #$ENV{G2O_ROOT}/include
  #$ENV{G2O_ROOT}
  ${EXTERNAL_LIBRARIES}/external_libraries/g2o_master/install_dir/include
  #/usr/local/include
  #/usr/include
  #/opt/local/include
  #/sw/local/include
  #/sw/include
  NO_DEFAULT_PATH
  )
macro(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)
  find_library(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS ${EXTERNAL_LIBRARIES}/external_libraries/g2o_master/install_dir/lib   
    NO_DEFAULT_PATH
    #NO_CMAKE_PATH
    #NO_CMAKE_ENVIRONMENT_PATH
    #NO_SYSTEM_ENVIRONMENT_PATH
    #NO_CMAKE_SYSTEM_PATH
    )

endmacro(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)
# Find the core elements

FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)

# G2O itself declared found if we found the core libraries and at least one solver
SET(G2O_FOUND "NO")
IF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
  SET(G2O_FOUND "YES")
ENDIF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
#find_package(g2o REQUIRED PATHS ${EXTERNAL_LIBRARIES}/external_libraries/g2o_master/install_dir/lib/cmake/g2o)

#find_package(g2o REQUIRED PATHS ${EXTERNAL_LIBRARIES}/external_libraries/g2o_master/install_dir/lib/cmake/g2o )
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
  tf
  tf_conversions
  message_filters
  message_generation
  genmsg
  
)
message("G2O_dir "${G2O_TYPES_DATA})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_service_files(FILES
  SaveMap.srv
  DumpGraph.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
  
 )

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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_icp_mcc
   CATKIN_DEPENDS message_filters pcl_conversions roscpp sensor_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
message("g2o_include_dir:"${G2O_INCLUDE_DIR} )
include_directories(include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIR}
   ${G2O_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pcl_icp_mcc.cpp
# )
message("pcl_libs:"${PCL_INCLUDE_DIRS})
add_executable(pcl_icp_mcc src/pcl_icp_mcc.cpp)
target_link_libraries(pcl_icp_mcc ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${Eigen3_LIBRARIES})

add_executable(cicp_slam src/icp_mcc_nodelet.cpp)
message("G2o_path: "${G2O_CORE_LIBRARY})
message("Eigen3_libs: "${EIGEN3_INCLUDE_DIRS})
target_link_libraries(cicp_slam ${G2O_STUFF_LIBRARY} ${G2O_TYPES_DATA} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_PCG} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CHOLMOD} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS} ${catkin_LIBRARIES} ${PCL_LIBRARIES}  )

add_dependencies(cicp_slam cicpk_gencpp)
#add_executable(preprocessing src/preprocessing.cpp)
#target_link_libraries(preprocessing ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${Eigen_LIBRARIES})


## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pcl_icp_mcc_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl_icp_mcc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
