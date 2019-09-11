project(model)
set(CMAKE_CXX_STANDARD 17)
find_package (Threads)
add_library(model src/DynamicModel.cpp src/Vehicle.cpp
        include/OdeSolver.hpp
        include/System.hpp
        include/ControlBlock.hpp
        src/Deadband.cpp
        include/Deadband.hpp
        include/MpcModel.hpp
        include/ControlBlock.hpp include/LimitBlock.hpp include/OperationMode.hpp
        include/Vehicule.hpp
        include/LimitBlock.hpp
        include/ModelConst.hpp
        include/Model.hpp src/KinematicModel.cpp include/KinematicModel.hpp
        include/Spline.hpp src/Spline.cpp include/DynamicModel.hpp src/odeSolver.cpp src/System.cpp include/MpcFactory.hpp src/MpcFactory.cpp include/ModelConst.hpp include/Model.hpp)
add_library(grampc STATIC IMPORTED)
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
