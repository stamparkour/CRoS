message("importing json...")

FetchContent_Declare(
	json 
	GIT_REPOSITORY https://github.com/nlohmann/json.git
	GIT_TAG f534f4f75e12893716ea688679aeb768bff426c4 # Mar 4, 2026
	DOWNLOAD_EXTRACT_TIMESTAMP ON
	FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(json)

if(json_POPULATED)
	message("json found!")
else()
	message("failed to find json")
endif()
