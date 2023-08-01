cmake_minimum_required(VERSION 2.8.9)
project(mfpfh_gazebo)

SET(PROJECT_NAME mfpfh_gazebo)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "https://gepgitlab.laas.fr/tlasguigne/multiscale-fpfh")

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
find_package(PCL 1.8.1 REQUIRED)
find_package(Boost REQUIRED thread filesystem system)
find_package(catkin REQUIRED COMPONENTS
    message_filters
    roslib
    roscpp
    sensor_msgs
    rosconsole
    eigen_conversions
    )

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
#headers
include_directories(include)

#Sources
SET(${PROJECT_NAME}_SOURCES
  src/main.cpp
  src/extractor.cpp
)

# Seems mandatory for a catkin package
catkin_package()

# ----------------------------------------------------
# --- EXECUTABLE -------------------------------------
# ----------------------------------------------------

#build Type Release or Debug
SET(CMAKE_BUILD_TYPE Release)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch meshes models urdf worlds)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()