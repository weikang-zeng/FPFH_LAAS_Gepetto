cmake_minimum_required(VERSION 2.8.9)
project(mfpfh_localizer)

SET(PROJECT_NAME mfpfh_localizer)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "https://gepgitlab.laas.fr/tlasguigne/multiscale-fpfh")

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
find_package(PCL 1.12.0 REQUIRED)
find_package(Boost REQUIRED thread filesystem system program_options)
find_package(OpenMP)
if (OPENMP_FOUND)
    message("OMP found!")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
#headers
include_directories(include)

#Sources
SET(${PROJECT_NAME}_SOURCES
  src/main.cpp
  src/localizer_base.cpp
  src/localizer_load.cpp
  src/localizer_run_clusters.cpp
  src/localizer_run_surprise.cpp
  src/objectsegmentationplane.cpp
  #src/gmm.cpp
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
