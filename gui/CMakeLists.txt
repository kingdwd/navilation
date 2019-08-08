project(gui)
set(CMAKE_CXX_STANDARD 17)
find_package( OpenCV REQUIRED )
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package (Threads)
add_executable(gui src/main.cpp src/UU.cpp src/ViewBuilder.cpp include/view.hpp )
include_directories(${CMAKE_SOURCE_DIR}/Util/include) # include files from ProjectB
include_directories(${CMAKE_SOURCE_DIR}/model/include) # include files from ProjectB
target_include_directories(gui PRIVATE "include")
target_link_libraries( gui ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} Util model Qt5::Widgets )
