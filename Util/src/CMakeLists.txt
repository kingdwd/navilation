# Notice name prefix of this variable, set by CMake according
# to value given with "project()" in the root CMakeLists.txt.
include_directories(${CMAKE_SOURCE_DIR}/Util/include)
add_library(Util math_util.cpp)