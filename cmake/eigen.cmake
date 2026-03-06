message("importing eigen3...")

FetchContent_Declare(
	eigen3 
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG d8c8ee6fb2cfe1af53b020ff10a70fbd9b47ccad # Mar 5, 2026
	DOWNLOAD_EXTRACT_TIMESTAMP ON
	FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(eigen3)

if(eigen3_POPULATED)
	message("eigen3 found!")
else()
	message("failed to find eigen3")
endif()
