project(model)
set(CMAKE_CXX_STANDARD 17)
find_package (Threads)
add_library(grampc STATIC IMPORTED)
set_target_properties(grampc PROPERTIES
        IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/lib/grampc/lib/libgrampc.a"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib/grampc/include"
        )
include_directories(${CMAKE_SOURCE_DIR}/Util/include) # include files from ProjectB
include_directories(${CMAKE_SOURCE_DIR}/lib/grampc/include) # include files from ProjectB

add_subdirectory(src)
add_subdirectory(test)

