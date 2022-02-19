cmake_minimum_required(VERSION 3.8)
project(path_planning CUDA CXX)

SET(CUDA_SEPARABLE_COMPILATION ON)

find_package(OpenCV)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(CUDA REQUIRED)


add_library(spacelib STATIC
space.cu
space.h
  )
  target_compile_features(spacelib PUBLIC cxx_std_11)

# set(CUDA_PROPAGATE_HOST_FLAGS OFF)
# SET(CUDA_HOST_COMPILER /usr/bin/g++)
# LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")

# add_compile_options(-std=c++11 -Wall -Wextra -fdiagnostics-color=auto)
# enable_testing()

# set(CUDA_NVCC_FLAGS "-std=c++11")

# # -D_REETRANT -D_THREAD_SAFE
# set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -D_DEBUG -g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# set(CUDA_NVCC_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})
# set(CUDA_NVCC_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})

# set(cpulib "cpu")
# set(gpulib "gpu")


# set(HEADER_FILES space.h)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
# cuda_include_directories(
#   ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
# )

set_target_properties(spacelib PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
add_executable(main main.cu)
# add_executable(rrt3d rrt3d.cpp space.cpp)

set_target_properties(main PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(main ${OpenCV_LIBS} ${Boost_LIBRARIES} spacelib)
# target_link_libraries(rrt3d ${OpenCV_LIBS} ${Boost_LIBRARIES})
