message("importing colmap...")

FetchContent_Declare(
	colmap 
	GIT_REPOSITORY https://github.com/colmap/colmap.git
	GIT_TAG 66636c16a12635435e30bccb460d0e1c92824da7 # Mar 8, 2026
	DOWNLOAD_EXTRACT_TIMESTAMP ON
	FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(colmap)

if(colmap_POPULATED)
	message("colmap found!")
else()
	message("failed to find colmap")
endif()
