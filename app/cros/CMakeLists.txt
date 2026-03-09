include(FetchContent)

set(CROS_SOURCE 
	"src/source.cpp"
	"src/data_table.h"
 "src/word_iterable.h" "src/cros.h" "src/points.h")

add_executable(cros ${CROS_SOURCE})
target_compile_features(cros PUBLIC cxx_std_20)

include(eigen)
target_link_libraries(cros PUBLIC Eigen3::Eigen)
include(json)
target_link_libraries(cros PUBLIC nlohmann_json::nlohmann_json)