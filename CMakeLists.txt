cmake_minimum_required(VERSION 3.8)
project(path_planning CXX)

option(ENABLE_CUDA "Enable CUDA support" OFF)
option(ENABLE_VISUALIZATION "Enable Visualization (requires Boost, Gnuplot)" OFF)

if(ENABLE_CUDA)
  find_package(CUDA REQUIRED)
  add_definitions(-DUSE_CUDA)
else()
  add_definitions(-DNO_CUDA)
endif()

if(ENABLE_VISUALIZATION)
  find_package(Boost REQUIRED COMPONENTS system iostreams)
  find_package(OpenCV) # Optional?
  add_definitions(-DENABLE_VISUALIZATION)
endif()

add_library(spacelib STATIC
space.cpp
space.h
  )
target_compile_features(spacelib PUBLIC cxx_std_17)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

if(ENABLE_CUDA)
    target_include_directories(spacelib PRIVATE ${CUDA_INCLUDE_DIRS})
endif()

if(ENABLE_VISUALIZATION)
    target_include_directories(spacelib PRIVATE ${Boost_INCLUDE_DIRS})
endif()

add_executable(main main.cpp)

target_link_libraries(main spacelib)

if(ENABLE_CUDA)
    target_link_libraries(main ${CUDA_LIBRARIES})
endif()

if(ENABLE_VISUALIZATION)
    target_link_libraries(main ${Boost_LIBRARIES})
    if(OpenCV_FOUND)
        target_link_libraries(main ${OpenCV_LIBS})
    endif()
endif()
