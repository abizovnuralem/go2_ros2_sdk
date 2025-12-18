cmake_minimum_required(VERSION 3.8)
project(lidar_accelerator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

find_path(WASMTIME_INCLUDE_DIR
  NAMES wasmtime.h
  PATHS
    $ENV{WASMTIME_C_API_ROOT}/include
    /opt/wasmtime-c-api/include
)

find_library(WASMTIME_LIBRARY
  NAMES wasmtime
  PATHS
    $ENV{WASMTIME_C_API_ROOT}/lib
    /opt/wasmtime-c-api/lib
)

include_directories(include)

pybind11_add_module(lidar_accelerator
  src/pybind_module.cpp
  src/processing.cpp
  src/packing.cpp
  src/wasm_decode.cpp
)

target_include_directories(lidar_accelerator PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(lidar_accelerator PRIVATE Python3::Python)
target_link_libraries(lidar_accelerator PRIVATE ament_index_cpp::ament_index_cpp)

if(WASMTIME_INCLUDE_DIR AND WASMTIME_LIBRARY)
  target_include_directories(lidar_accelerator SYSTEM PRIVATE ${WASMTIME_INCLUDE_DIR})
  target_link_libraries(lidar_accelerator PRIVATE ${WASMTIME_LIBRARY})
  target_compile_definitions(lidar_accelerator PRIVATE GO2_WASMTIME_C_API=1)
else()
  message(WARNING "Wasmtime C API not found; decode_and_process will not be available")
endif()

target_compile_definitions(lidar_accelerator PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)

set(PYTHON_INSTALL_DIR
  "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

install(TARGETS lidar_accelerator
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}"
)

ament_package()
