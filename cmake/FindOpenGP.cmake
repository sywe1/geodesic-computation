find_path(OpenGP_INCLUDE_DIR 
	NAMES
		OpenGP/SurfaceMesh/SurfaceMesh.h
	PATHS
		./
		./src
		../src
		../../src
		../../../src
		./OpenGP/src/
		./../OpenGP/src/
		./../../OpenGP/src
		./external/opengp
		/usr/local/include
		/usr/include
	)

if(OpenGP_INCLUDE_DIR)
	set(OpenGP_FOUND TRUE)
else()
	set(OpenGP_FOUND FALSE)
endif()

if(OpenGP_FOUND)
    if(NOT CMAKE_FIND_QUIETLY)
        message(STATUS "Found OpenGP: ${OpenGP_INCLUDE_DIR}")
    endif()
else()
    if(OpenGP_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find OpenGP")
    endif()
endif()
