cmake_minimum_required(VERSION 2.8.3)
project(pepper_hand_pose)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  visp_bridge
  whycon
)

find_package(VISP REQUIRED)


find_package(NAOqi QUIET COMPONENTS
  alcommon
  alvision
  alproxies
  alerror
  alvalue
  qi
  qitype
)

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

catkin_package(
  LIBRARIES
    pepper_hand_pose
  
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf

  DEPENDS 
    VISP
)

include_directories(include)

include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${NAOqi_INCLUDE_DIRS}
  ${whycon_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

if( NAOqi_FOUND)
  message(STATUS "VISPNAOQIBRIDGE: NAOqi C++ SDK found; C++ nodes will be built")
else()
  message(STATUS "VISPNAOQIBRIDGE:Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()

# Declare a cpp library
add_library(pepper_hand_pose
  include/pepper_hand_pose.h
  src/pepper_hand_pose.cpp
)

target_link_libraries(pepper_hand_pose
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${NAOqi_LIBRARIES}
  ${whycon_LIBRARIES}
)

## Declare a cpp executable
#add_executable(pepper_hand_pose_node src/pepper_hand_pose.cpp)
add_executable(pepper_hand_pose_node src/main.cpp)

target_link_libraries(pepper_hand_pose_node
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${NAOqi_LIBRARIES}
  ${Boost_LIBRARIES}
  pepper_hand_pose
)

install(
  TARGETS
    pepper_hand_pose
    pepper_hand_pose_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


