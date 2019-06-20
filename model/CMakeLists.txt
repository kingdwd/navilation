project(model)
set(CMAKE_CXX_STANDARD 17)
find_package (Threads)
add_library(model src/Controller.cpp src/Vehicle.cpp include/DiffEquationSolver.hpp include/System.hpp)
include_directories(${CMAKE_SOURCE_DIR}/Util/include) # include files from ProjectB
target_include_directories(model PRIVATE "include")
target_link_libraries(model ${CMAKE_THREAD_LIBS_INIT} Util)
add_subdirectory(test)

install(TARGETS model
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
