cmake_minimum_required(VERSION 3.12)
project(Epiphany)
set(CMAKE_CXX_STANDARD 17)
set(REPO ~/CLionProjects/lib)
set(CMAKE_MODULE_PATH "~/CLionProjects/cmake;$CMAKE_MODULE_PATH")
include(DownloadProject/DownloadProject)


link_libraries(stdc++fs)
add_subdirectory(Navilation)
add_subdirectory(Util)
add_subdirectory(gui)
add_subdirectory(model)


