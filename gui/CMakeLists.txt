project(gui)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
find_package( OpenCV REQUIRED )
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package (Threads)
link_directories(${CMAKE_SOURCE_DIR}/Util/src)
link_directories(${CMAKE_SOURCE_DIR}/model/src)
include_directories(${CMAKE_SOURCE_DIR}/gui/include) # include files from ProjectB
include_directories(${CMAKE_SOURCE_DIR}/Util/include) # include files from ProjectB
include_directories(${CMAKE_SOURCE_DIR}/model/include) # include files from ProjectB

add_executable(gui
        include/view_model.hpp
        src/main.cpp
        src/cv_util.cpp
        src/view_control.cpp
        src/view_model.cpp
        src/view.cpp
        src/key_handler.cpp
        )

target_link_libraries( gui ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} Util model Qt5::Widgets )

