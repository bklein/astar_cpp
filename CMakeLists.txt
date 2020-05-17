cmake_minimum_required(VERSION 3.17)

project(astar_cpp
  VERSION 0.0.1
  LANGUAGES CXX
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -Weffc++ -Wpedantic")

add_library(astar_cpp INTERFACE)
target_include_directories(astar_cpp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(astar_cpp INTERFACE cxx_std_20)
target_compile_options(astar_cpp INTERFACE -Werror -Wall -Wextra)


add_executable(astar_snap_demo astar_snap_demo.cc)
target_link_libraries(astar_snap_demo PRIVATE astar_cpp)

add_executable(astar_utah_demo astar_utah_demo.cc)
target_link_libraries(astar_utah_demo PRIVATE astar_cpp)
