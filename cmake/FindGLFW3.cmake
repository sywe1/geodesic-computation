find_path(GLFW3_INCLUDE_DIRS
	NAMES
		GLFW/glfw3.h
	PATHS
		/usr/local/include
		/usr/local/X11R6/include
		/usr/X11/include
		/usr/X11R6/include
		/usr/include/X11
    	/usr/include
    	/opt/X11/include
    	/opt/include 
		)
find_library(GLFW3_LIBRARIES 
	NAMES 
		glfw3
		glfw
	PATHS
	$ENV{GLFWDIR}/lib
    $ENV{GLFWDIR}/support/msvc80/Debug
    $ENV{GLFWDIR}/support/msvc80/Release
    /usr/lib/x86_64-linux-gnu/
    /usr/local/lib
    /usr/local/X11R6/lib
    /usr/X11R6/lib
    /usr/X11/lib
    /usr/lib/X11
    /usr/lib
    /opt/X11/lib
    /opt/lib )

SET(GLFW3_FOUND FALSE)
IF(GLFW3_LIBRARIES AND GLFW3_INCLUDE_DIRS)
	SET(GLFW3_FOUND TRUE)
ENDIF(GLFW3_LIBRARIES AND GLFW3_INCLUDE_DIRS)
