cmake_minimum_required(VERSION 3.18)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    set(CMAKE_CUDA_ARCHITECTURES native)
  else()
    set(CMAKE_CUDA_ARCHITECTURES 75)
  endif()
endif()

project(ntmesh_ray LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# -D wins, then the environment variable, then a sibling checkout.
function(nt_resolve_root var marker sibling hint)
  if(NOT ${var} AND DEFINED ENV{${var}} AND NOT "$ENV{${var}}" STREQUAL "")
    set(${var} "$ENV{${var}}" PARENT_SCOPE)
    return()
  endif()
  if(NOT ${var} AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${sibling}/${marker}")
    get_filename_component(found "${CMAKE_CURRENT_SOURCE_DIR}/${sibling}" ABSOLUTE)
    set(${var} "${found}" PARENT_SCOPE)
    return()
  endif()
  if(NOT ${var})
    message(FATAL_ERROR "${var} is not set. Pass -D${var}= or export ${var}. ${hint}")
  endif()
endfunction()

nt_resolve_root(NTV_ROOT "src/nvec3.h" "../ntv/ntv" "NTV_ROOT is the ntv repo that contains src/nvec3.h.")
nt_resolve_root(NTMESH_ROOT "src/ntmesh.h" "../ntmesh" "NTMESH_ROOT is the ntmesh repo that contains src/ntmesh.h.")

if(NOT IS_ABSOLUTE "${NTV_ROOT}")
  get_filename_component(NTV_ROOT "${NTV_ROOT}" ABSOLUTE)
endif()
if(NOT IS_ABSOLUTE "${NTMESH_ROOT}")
  get_filename_component(NTMESH_ROOT "${NTMESH_ROOT}" ABSOLUTE)
endif()
if(NOT EXISTS "${NTV_ROOT}/src/nvec3.h")
  message(FATAL_ERROR "NTV_ROOT does not contain src/nvec3.h: ${NTV_ROOT}")
endif()
if(NOT EXISTS "${NTMESH_ROOT}/src/ntmesh.h")
  message(FATAL_ERROR "NTMESH_ROOT does not contain src/ntmesh.h: ${NTMESH_ROOT}")
endif()

set(BUNNY_GEO "${CMAKE_CURRENT_SOURCE_DIR}/samplegeo/mixed_manifold_bunny.geo")
if(NOT EXISTS "${BUNNY_GEO}")
  message(FATAL_ERROR "missing samplegeo/mixed_manifold_bunny.geo")
endif()

find_package(CUDAToolkit REQUIRED)
find_package(Threads REQUIRED)

add_executable(ntmesh_ray_tests test/main.C test/test_ray.C)
target_include_directories(ntmesh_ray_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/test
  ${NTMESH_ROOT}/src
  ${NTV_ROOT}/src
)
target_compile_definitions(ntmesh_ray_tests PRIVATE BUNNY_GEO="${BUNNY_GEO}")
target_compile_options(ntmesh_ray_tests PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ntmesh_ray_tests PRIVATE Threads::Threads)

add_executable(rtiow_bvh
  samples/CUDA/src/main.cu
  samples/CUDA/src/bunny_load.cpp
)
set_target_properties(rtiow_bvh PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
)
target_include_directories(rtiow_bvh PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/samples/CUDA/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${NTMESH_ROOT}/src
  ${NTV_ROOT}/samples/CUDA/src
  ${NTV_ROOT}/src
)
target_compile_definitions(rtiow_bvh PRIVATE BUNNY_GEO="${BUNNY_GEO}")
target_compile_options(rtiow_bvh PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:-DNTL_HD=__host__\ __device__>
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
)
target_link_libraries(rtiow_bvh PRIVATE CUDA::cudart)

add_executable(rtiow_bvh_cpu
  samples/CPU/src/main.cpp
  samples/CUDA/src/bunny_load.cpp
)
target_include_directories(rtiow_bvh_cpu PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/samples/CPU/src
  ${CMAKE_CURRENT_SOURCE_DIR}/samples/CUDA/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${NTMESH_ROOT}/src
  ${NTV_ROOT}/samples/CUDA/src
  ${NTV_ROOT}/src
)
target_compile_definitions(rtiow_bvh_cpu PRIVATE BUNNY_GEO="${BUNNY_GEO}")
target_compile_options(rtiow_bvh_cpu PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rtiow_bvh_cpu PRIVATE Threads::Threads)

message(STATUS "NTV_ROOT: ${NTV_ROOT}")
message(STATUS "NTMESH_ROOT: ${NTMESH_ROOT}")
message(STATUS "CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
