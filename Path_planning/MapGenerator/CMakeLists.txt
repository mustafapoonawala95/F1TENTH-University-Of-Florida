cmake_minimum_required(VERSION 3.1)

project(MapGenerator)

set(CMAKE_CXX_STANDARD 17)


include_directories(include/)
include_directories(io_tools/)

add_library(io_tools io_tools/io_tools.cpp)
add_library(MyImage src/MyImage.cpp)

target_link_libraries (io_tools MyImage)  # ADD THIS FOR WHENEVER USING FILESYSTEM ==> stdc++fs
#add_executable(main src/main.cpp)
add_executable(MapGenerator src/MapGenerator.cpp)

#target_link_libraries(main MyImage io_tools)
target_link_libraries(MapGenerator MyImage io_tools)
