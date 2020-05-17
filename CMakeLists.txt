cmake_minimum_required(VERSION 3.17)

project(snap_astar
  VERSION 0.0.1
  LANGUAGES CXX
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -Weffc++ -Wpedantic")

add_library(snap_astar INTERFACE)
target_include_directories(snap_astar INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(snap_astar INTERFACE cxx_std_20)
target_compile_options(snap_astar INTERFACE -Werror -Wall -Wextra)


add_executable(snap_astar_demo demo.cc)
target_link_libraries(snap_astar_demo PRIVATE snap_astar)

add_executable(snap_astar_demo2 demo2.cc)
target_link_libraries(snap_astar_demo2 PRIVATE snap_astar)

#add_executable(heap_tests heap_tests.cc)
#target_link_libraries(heap_tests PRIVATE heap_cpp)
