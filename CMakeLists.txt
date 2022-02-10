cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(OpenCV)
find_package(Boost REQUIRED COMPONENTS system iostreams)

# set(HEADER_FILES space.h)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(main main.cpp space.cpp)

target_link_libraries(main ${OpenCV_LIBS} ${Boost_LIBRARIES})
