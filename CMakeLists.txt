cmake_minimum_required(VERSION 3.3)
set (CMAKE_CXX_STANDARD 14)
project(weitu)

#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

# Find catkin
find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge image_transport camera_info_manager sensor_msgs tf)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS ${WEITU_LIBS} ${OpenCV_LIBS})


find_package(OpenCV)

include_directories(
  include ${catkin_INCLUDE_DIRS}${INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# camera env
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES}
    /opt/iCentral/iCentral/include)
LINK_DIRECTORIES(${LINK_DIRECTORIES}
    /opt/iCentral/iCentral/lib)

# opencv env
find_package(OpenCV 3 REQUIRED)


set(WEITU_LIBS
    MVSDK
    ImageConvert)


set(FINDER_SRC
    src/finder/finderpattern.cpp
    src/finder/finderpatternfinder.cpp)

set(EDCIRCLE_SRC
    src/edcircle/edcircle.cpp)


include_directories(${INCLUDE_DIRECTORIES}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_LIST_DIR}
        include)


add_library(${PROJECT_NAME} ${FINDER_SRC} ${EDCIRCLE_SRC} src/weitu.cpp)
target_link_libraries(${PROJECT_NAME} ${WEITU_LIBS} ${OpenCV_LIBS})


add_executable(${PROJECT_NAME}_test test/test.cpp)
add_dependencies(${PROJECT_NAME}_test ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

add_executable(weitu_ros
  src/weitu_ros.cpp
)
add_dependencies(weitu_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(weitu_ros
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(weitu_finder
  src/weitu_finder.cpp
)
add_dependencies(weitu_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(weitu_finder
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# lib & exe

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_test
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# inc
install(DIRECTORY include/
 DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
)
