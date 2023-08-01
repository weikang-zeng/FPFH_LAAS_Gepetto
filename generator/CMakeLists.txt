cmake_minimum_required(VERSION 2.8.9)
project(mfpfh_generator)

SET(PROJECT_NAME mfpfh_generator)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "https://gepgitlab.laas.fr/tlasguigne/multiscale-fpfh")

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
find_package(PCL 1.8.1 REQUIRED)
find_package(Boost REQUIRED thread filesystem system program_options)

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
#headers
include_directories(include)

#Sources
SET(${PROJECT_NAME}_SOURCES
  src/main.cpp
  src/mfpfh.cpp
)

# ----------------------------------------------------
# --- EXECUTABLE -------------------------------------
# ----------------------------------------------------

#build Type Release or Debug
SET(CMAKE_BUILD_TYPE Release)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} ${Boost_LIBRARIES})
