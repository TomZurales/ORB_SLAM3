
cmake_minimum_required(VERSION 3.10)
project(vbee LANGUAGES CXX)

find_package(Eigen3 3.1.0 REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og")

set(LIBRARY_SOURCES
    src/vbee.cc
    src/observability_model.cc
    src/existence_probability_estimator.cc
)

add_library(vbee ${LIBRARY_SOURCES})
set_target_properties(vbee PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(vbee PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
)
target_link_libraries(vbee PUBLIC Eigen3::Eigen -lboost_serialization)
set_target_properties(vbee PROPERTIES OUTPUT_NAME vbee)

# Alias target for add_subdirectory usage
add_library(vbee::vbee ALIAS vbee)

# Install rules
include(GNUInstallDirs)
install(TARGETS vbee
    EXPORT vbeeTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ../include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the targets to a script
install(EXPORT vbeeTargets
    FILE vbeeTargets.cmake
    NAMESPACE vbee::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vbee
)

# Config file for find_package
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/vbeeConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/vbeeConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/vbeeConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vbee
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/vbeeConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/vbeeConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vbee
)
