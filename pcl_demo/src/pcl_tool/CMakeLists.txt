cmake_minimum_required (VERSION 3.15)

project(pcl_tool_lib CXX)
foreach(dirname ${CMAKE_CURRENT_SOURCE_DIR})
    file(GLOB_RECURSE SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
            "${dirname}/*.h"
            "${dirname}/*.hpp"
            "${dirname}/*.c"
            "${dirname}/*.cpp"
            )
endforeach()



add_library(${PROJECT_NAME} STATIC ${SRC})

#target_link_libraries(${PROJECT_NAME} PUBLIC sutils)

find_package(jsoncpp CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC JsonCpp::JsonCpp)

find_package(PCL CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

find_package(VTK REQUIRED)
include("${VTK_USE_FILE}")
target_link_libraries(${PROJECT_NAME} PUBLIC  ${VTK_LIBRARIES})

