cmake_minimum_required (VERSION 3.15)



add_executable(filter_tool  "filter_tool.cpp" )
target_link_libraries(filter_tool PUBLIC pcl_tool_lib)






