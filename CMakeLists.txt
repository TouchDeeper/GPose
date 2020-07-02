cmake_minimum_required(VERSION 3.9)
project(object_pose_estimation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE  "Debug")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(Threads_FOUND)
    MESSAGE(STATUS "Threads_LIBRARIES =  ${Threads_LIBRARIES}")
endif()
# Eigen
#find_package(Eigen REQUIRED)
include_directories("/usr/local/include/eigen3" )
# PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#no template sophus
#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )
#template sophus
include_directories(/home/wang/slam/thirdparty/Sophus/install/include)
# TdLibrary
set(TdLib_DIR "/home/wang/CLionProjects/TdLibrary/install/lib/CMake/TdLib/")
find_package(TdLib)
include_directories(${TdLib_INCLUDE_DIRS})
#OPENMP
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# json tool
find_package(nlohmann_json 3.2.0 REQUIRED)

# OpenCV
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
if(OpenCV_FOUND)
    MESSAGE(STATUS "OPENCV_INCLUDE_DIRS = " ${OPENCV_INCLUDE_DIRS})
    MESSAGE(STATUS "OpenCV_LIBRARY_DIRS = " ${OpenCV_LIBRARY_DIRS})
    MESSAGE(STATUS "OpenCV_LIBRARIES = " ${OpenCV_LIBRARIES})
endif()
#Boost
find_package(Boost COMPONENTS system filesystem thread chrono REQUIRED)

# add header file
include_directories(include)
# add source file
add_library(localLib SHARED
        src/Matching.cpp
#        add_model/render_synthetic_views.cpp
        src/config_reader.cpp
#        ga_objective.hpp
#        ga_objective.cpp
#        openga/openga_class.cpp
        src/match/LocalPipe.cpp
        src/parameters.cpp
        src/Optimization.cpp
        src/view_graph/view_graph.cpp
        src/access_model_data.cpp
        src/GlobalCorrespondence.cpp
#        src/PathManager.cpp
#        src/CsvReader.cpp
        )
add_library(libdataprocess SHARED src/DataProcessor.cpp src/config_reader.cpp)
target_link_libraries(libdataprocess ${TdLib_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(localLib
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
#        ${TdLib_LIBRARIES}
        nlohmann_json::nlohmann_json
        )

add_executable(local_pipeline local_pipeline.cpp)
target_link_libraries(local_pipeline
        ${PCL_LIBRARIES}
        localLib
        ${TdLib_LIBRARIES}
        Threads::Threads
        )


add_executable(globalPipeline global_pipeline.cpp )
target_link_libraries(globalPipeline
        ${PCL_LIBRARIES}
        localLib
        ${TdLib_LIBRARIES}
        Threads::Threads
       )

add_executable(show_candidate test/show_candidate.cpp )
target_link_libraries(show_candidate
        ${PCL_LIBRARIES}
        localLib
        ${TdLib_LIBRARIES}
        )
add_executable(check_result test/check_results.cpp )
target_link_libraries(check_result
        ${TdLib_LIBRARIES}
        ${Boost_LIBRARIES}
        libdataprocess
         )
add_library(libalignbug SHARED src/bug_src.cpp)
#target_link_libraries(libalignbug ${PCL_LIBRARIES})
add_executable(bug_test2 test/bug_test2.cpp )
target_link_libraries(bug_test2
        #${PCL_LIBRARIES}
        libalignbug
        )
add_executable(data_process test/data_process.cpp )

add_executable(check_view_graph test/check_view_graph.cpp)
target_link_libraries(check_view_graph
        ${PCL_LIBRARIES}
        localLib
        ${TdLib_LIBRARIES}
        )

add_executable(point_pick test/point_pick.cpp)
target_link_libraries(point_pick
        ${PCL_LIBRARIES}
        localLib
        ${TdLib_LIBRARIES}
        )
