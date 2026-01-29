# CMakeList.txt : CMake project for CRoS, include source and define
# project specific logic here.
#

include(FetchContent)

# Add source to this project's executable.
add_executable (CRoS "CRoS.cpp" "generate_data.h" "cros_algorithm.h")
set_property(TARGET CRoS PROPERTY CXX_STANDARD 20)
target_include_directories(CRoS PRIVATE "${CRoS_SOURCE_DIR}/CRoS")

FetchContent_Declare(
	eigen
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG e246f9cb68d07e8f15c60a2404ac9a625c349223
)
FetchContent_MakeAvailable(eigen)
target_link_libraries(CRoS PRIVATE eigen)

# TODO: Add tests and install targets if needed.
