cmake_minimum_required(VERSION 3.16)
project(fast_gicp)

option(BUILD_VGICP_CUDA "Build GPU-powered VGICP and NDT CUDA (optional)" OFF)
option(BUILD_apps "Build standalone test applications" OFF)
option(BUILD_test "Build unit tests" OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration features kdtree)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

if(BUILD_VGICP_CUDA)
  set(_NVBIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nvbio")
  if(NOT EXISTS "${_NVBIO_DIR}/nvbio/basic/priority_queue.h")
    message(FATAL_ERROR
      "nvbio not found at ${_NVBIO_DIR}\n"
      "Run: ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/setup_fast_gicp_deps.sh")
  endif()

  # CMP0146 exists only in CMake >= 3.27 (Ubuntu 22.04 ships 3.22)
  if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
  endif()
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})

  # Ubuntu 22.04: GCC 11 + system CUDA — .cu must use C++14 or nvcc chokes on libstdc++11
  set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS}
    -std=c++14
    --expt-relaxed-constexpr
    --extended-lambda
    -Wno-deprecated-declarations
    -Wno-deprecated-gpu-targets
  )
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    list(APPEND CUDA_NVCC_FLAGS -allow-unsupported-compiler)
    list(APPEND CUDA_NVCC_FLAGS -Xcompiler=-std=c++14)
    list(APPEND CUDA_NVCC_FLAGS -Xcompiler=-Wno-deprecated-declarations)
  endif()
  cuda_select_nvcc_arch_flags(FAST_GICP_CUDA_ARCH Auto)
  list(APPEND CUDA_NVCC_FLAGS ${FAST_GICP_CUDA_ARCH})
endif()

add_library(fast_gicp SHARED
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_gicp_st.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
)

target_include_directories(fast_gicp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(fast_gicp PUBLIC
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_compile_definitions(fast_gicp PUBLIC ${PCL_DEFINITIONS})

if(BUILD_VGICP_CUDA)
  add_definitions(-DUSE_VGICP_CUDA)

  cuda_add_library(fast_vgicp_cuda SHARED
    src/fast_gicp/cuda/fast_vgicp_cuda.cu
    src/fast_gicp/cuda/brute_force_knn.cu
    src/fast_gicp/cuda/covariance_estimation.cu
    src/fast_gicp/cuda/covariance_estimation_rbf.cu
    src/fast_gicp/cuda/covariance_regularization.cu
    src/fast_gicp/cuda/gaussian_voxelmap.cu
    src/fast_gicp/cuda/find_voxel_correspondences.cu
    src/fast_gicp/cuda/compute_derivatives.cu
    src/fast_gicp/cuda/compute_mahalanobis.cu
    src/fast_gicp/cuda/ndt_cuda.cu
    src/fast_gicp/cuda/ndt_compute_derivatives.cu
  )

  target_include_directories(fast_vgicp_cuda PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    ${_NVBIO_DIR}
    ${CUDA_INCLUDE_DIRS}
  )

  cuda_add_cublas_to_target(fast_vgicp_cuda)

  target_sources(fast_gicp PRIVATE
    src/fast_gicp/gicp/fast_vgicp_cuda.cpp
    src/fast_gicp/ndt/ndt_cuda.cpp
  )

  target_link_libraries(fast_gicp PUBLIC fast_vgicp_cuda)
  add_dependencies(fast_gicp fast_vgicp_cuda)
endif()

if(BUILD_apps)
  add_executable(gicp_align src/align.cpp)
  target_link_libraries(gicp_align ${PCL_LIBRARIES} fast_gicp)
  install(TARGETS gicp_align RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS fast_gicp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_VGICP_CUDA)
  install(TARGETS fast_vgicp_cuda
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(fast_gicp)
if(BUILD_VGICP_CUDA)
  ament_export_libraries(fast_vgicp_cuda)
endif()
ament_export_dependencies(PCL Eigen3)

ament_package()
