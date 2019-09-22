include_directories(${CMAKE_SOURCE_DIR}/model/include)
add_library(model
        deadband.cpp
        kinematic_model.cpp
        dynamic_model.cpp
        mpc_factory.cpp
        ode_solver.cpp
        spline.cpp
        system.cpp
        vehicle.cpp
        )
target_link_libraries(model ${CMAKE_THREAD_LIBS_INIT} Util grampc)
