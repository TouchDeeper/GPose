cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(offline_data_generation)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization features surface)
find_package(PCL 1.8 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(../include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message("available pcl component: " ${PCL_COMPONENTS})

# TdLibrary
set(TdLib_DIR "/home/wang/CLionProjects/TdLibrary/install/lib/CMake/TdLib/")
find_package(TdLib)
include_directories(${TdLib_INCLUDE_DIRS})
# sophus
#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories(/home/wang/slam/thirdparty/Sophus/install/include)

add_executable (offline_data_generation offline_data_generation.cpp)
add_library (Render_Synthetic_Views render_synthetic_views.cpp)
add_library (View_Graph ../src/view_graph/view_graph.cpp)
add_library (Access_Model_Data ../src/access_model_data.cpp)
add_library (Render_Views_Tesselated_Sphere_Modified render_views_tesselated_sphere_modified.cpp)
add_library (View_Feature_Score view_feature_score.cpp)
add_library (Config_Reader ../src/config_reader.cpp)
add_library (Similar_Object_Recognition similar_object_recognition.cpp)
add_library (Normal_Utility normal_utility.cpp)
add_library (Parameters ../src/parameters.cpp )
target_link_libraries(Parameters Config_Reader)
#target_link_libraries(Render_Synthetic_Views ${PCL_LIBRARIES} ${PCL_SURFACE_LIBRARIES})
target_link_libraries (offline_data_generation ${PCL_LIBRARIES}
#        ${PCL_COMMON_LIBRARIES}
#        ${PCL_IO_LIBRARIES}
#        ${PCL_VISUALIZATION_LIBRARIES}
#        ${PCL_SURFACE_LIBRARIES}
#        ${PCL_FEATURES_LIBRARIES}

        )
target_link_libraries (offline_data_generation Render_Synthetic_Views)
target_link_libraries (offline_data_generation View_Graph)
target_link_libraries (offline_data_generation Access_Model_Data)
target_link_libraries (offline_data_generation Render_Views_Tesselated_Sphere_Modified)
target_link_libraries (offline_data_generation View_Feature_Score)
target_link_libraries (offline_data_generation Config_Reader)
target_link_libraries (offline_data_generation Similar_Object_Recognition)
target_link_libraries (offline_data_generation Normal_Utility)
target_link_libraries (offline_data_generation ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries (offline_data_generation ${Boost_SYSTEM_LIBRARY})
target_link_libraries (offline_data_generation Parameters )
target_link_libraries (offline_data_generation ${TdLib_LIBRARIES})

add_executable(bug_test bug_test.cpp)
target_link_libraries(bug_test
        ${PCL_LIBRARIES}
        Render_Synthetic_Views
        View_Graph
        Access_Model_Data
        Render_Views_Tesselated_Sphere_Modified
        View_Feature_Score Normal_Utility ${Boost_SYSTEM_LIBRARY}
        Config_Reader
        Similar_Object_Recognition
        Parameters
        ${Boost_FILESYSTEM_LIBRARY} ${TdLib_LIBRARIES})

add_executable(check_view_graph check_view_graph.cpp)
target_link_libraries (check_view_graph ${PCL_LIBRARIES})
target_link_libraries (check_view_graph Render_Synthetic_Views)
target_link_libraries (check_view_graph View_Graph)
target_link_libraries (check_view_graph Access_Model_Data)
#target_link_libraries (check_view_graph Render_Views_Tesselated_Sphere_Modified)
#target_link_libraries (check_view_graph View_Feature_Score)
target_link_libraries (check_view_graph Config_Reader)
#target_link_libraries (check_view_graph Similar_Object_Recognition)
#target_link_libraries (check_view_graph Normal_Utility)
target_link_libraries (check_view_graph ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries (check_view_graph ${Boost_SYSTEM_LIBRARY})
target_link_libraries (check_view_graph Parameters )
target_link_libraries (check_view_graph ${TdLib_LIBRARIES})


