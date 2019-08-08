project(model)
set(CMAKE_CXX_STANDARD 17)
find_package (Threads)
add_library(model src/Controller.cpp src/Vehicle.cpp
        include/DiffEquationSolver.hpp
        include/System.hpp
        include/ControlBlock.hpp
        src/Deadband.cpp
        include/Deadband.hpp
        include/MpcModel.hpp
        include/LimitBlock.hpp
        include/Spline.hpp src/Spline.cpp)
add_library(grampc STATIC IMPORTED include/block/ControlBlock.hpp include/LimitBlock.hpp)
set_target_properties(grampc PROPERTIES
        IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/lib/grampc/lib/libgrampc.a"
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib/grampc/include"
        )
include_directories(${CMAKE_SOURCE_DIR}/Util/include) # include files from ProjectB
target_include_directories(model PRIVATE "include")
target_link_libraries(model ${CMAKE_THREAD_LIBS_INIT} Util grampc)
add_subdirectory(test)

install(TARGETS model
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
