cmake_minimum_required (VERSION 3.15)

project(socket_tool CXX)
foreach(dirname ${CMAKE_CURRENT_SOURCE_DIR})
    file(GLOB_RECURSE SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
            "${dirname}/*.h"
            "${dirname}/*.hpp"
            "${dirname}/*.c"
            "${dirname}/*.cpp"
            )
endforeach()

add_library(${PROJECT_NAME} STATIC ${SRC})


#find_package(asio CONFIG REQUIRED)
#target_link_libraries(${PROJECT_NAME} STATIC asio::asio)