cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(geodesic-computation LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(cmake/ConfigureCompiler.cmake)
include(cmake/ConfigureOpenGL.cmake)
include(cmake/ConfigureGLEW.cmake)
include(cmake/ConfigureGLFW3.cmake)
include(cmake/ConfigureEigen3.cmake)
include(cmake/ConfigureOpenGP.cmake)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h")

add_executable(demo ${SOURCES} ${HEADERS})
target_link_libraries(demo ${LIBRARIES})
