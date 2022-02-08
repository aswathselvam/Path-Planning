cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(OpenCV)
find_package(Matplot++ REQUIRED)


# set(HEADER_FILES space.h)

add_executable(main main.cpp space.cpp)# space.h ${HEADER_FILES})

target_link_libraries(main PUBLIC Matplot++::matplot ${OpenCV_LIBS}  )
