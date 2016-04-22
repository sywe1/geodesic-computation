if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
    message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}") 
endif()

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
endif()

if(UNIX AND NOT APPLE AND CMAKE_COMPILER_IS_GNUCXX)
    message(STATUS "Using gcc/g++ compiler (Linux)")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pedantic -Wextra -Wno-long-long")
endif()

if(APPLE AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    message(STATUS "Using Clang compiler on (Apple)")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pedantic -Wextra -Wno-long-long")
endif()

if(WIN32)
    # Tell IDE (esp. MSVC) to use folders.
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_USE_MATH_DEFINES")
    add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()
