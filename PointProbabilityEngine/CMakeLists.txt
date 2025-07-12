cmake_minimum_required(VERSION 3.14)
project(PointProbabilityEngine VERSION 1.0.0 LANGUAGES C CXX)
# Set build type to Release with maximum optimizations if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Enable maximum optimizations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")

# Additional optimization flags for GCC/Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native -mtune=native")
endif()
# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(PCL REQUIRED)
find_package(GLEW REQUIRED)

# Add library source files here
add_library(PointProbabilityEngine SHARED
  src/PointProbabilityEngine.cc
  src/IcosahedronBackend.cc
  src/Icosahedron.cc
  src/Utility.cc
  src/Viewer.cc
  src/ShaderManager.cc
  src/PointXYZID.cc
  src/PointProbabilityEngine/3DViews.cc

  # Graphics files
  src/Graphics/FloorPlane.cc
  src/Graphics/PointCloud.cc
  src/Graphics/IcosModel.cc
  src/Graphics/CameraModel.cc
  src/Graphics/VonMiserSphere.cc
  
  # Imgui files
  third_party/imgui/imgui.cpp
  third_party/imgui/imgui_draw.cpp
  third_party/imgui/imgui_widgets.cpp
  third_party/imgui/imgui_demo.cpp
  third_party/imgui/imgui_tables.cpp
  third_party/imgui/backends/imgui_impl_glfw.cpp
  third_party/imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(PointProbabilityEngine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
)

target_link_libraries(PointProbabilityEngine
  PUBLIC
    Eigen3::Eigen
    OpenGL::GL
    glfw
    GLEW::GLEW
    ${CMAKE_DL_LIBS}
    ${PCL_LIBRARIES}
)

set_target_properties(PointProbabilityEngine PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON
)

# Demo Build
set(DEMO_SRCS
  demo/src/PointImpl.cc
  demo/src/MapImpl.cc
  demo/src/CameraImpl.cc
  demo/point_probability_engine_demo.cc
)

add_executable(point_probability_engine_demo ${DEMO_SRCS})
target_link_libraries(point_probability_engine_demo PointProbabilityEngine ${PCL_LIBRARIES})
target_include_directories(point_probability_engine_demo
PRIVATE
  ${PCL_INCLUDE_DIRS}
  demo/include
)
