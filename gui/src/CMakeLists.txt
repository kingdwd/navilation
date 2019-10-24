include_directories(${CMAKE_SOURCE_DIR}/gui/include)
include_directories(${CMAKE_SOURCE_DIR}/Util/include) # include files from ProjectB
include_directories(${CMAKE_SOURCE_DIR}/model/include) # include files from ProjectB

add_executable(gui
        ../include/view_model.hpp
        cv_util.cpp
        key_handler.cpp
        main.cpp
        view.cpp
        view_control.cpp
        view_model.cpp
        )

target_link_libraries( gui ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} Util model Qt5::Widgets )

