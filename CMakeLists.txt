cmake_minimum_required (VERSION 3.24)
# set the project name
# characterizing rotation of satellites
project (cros VERSION 0.1.0 LANGUAGES CXX C)

#config
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(app/cros)
