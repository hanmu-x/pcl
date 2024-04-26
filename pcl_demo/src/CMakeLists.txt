cmake_minimum_required (VERSION 3.15)

project(pro_main CXX)

add_subdirectory(pcl_tool)
link_libraries(pcl_tool_lib)

add_subdirectory(socket)
link_libraries(socket_tool)

#add_executable(${PROJECT_NAME}  "main.cpp"  "socket/socket.h" "socket/socket.cpp")

add_executable(${PROJECT_NAME}  "main.cpp" )

#target_link_libraries(${PROJECT_NAME} PUBLIC sutils)




