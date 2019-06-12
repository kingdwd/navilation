project(Util)
add_subdirectory(test)
add_library(Util src/U.cpp include/U.h Agv.cpp Agv.h include/TypeUtil.cpp include/UMath.h src/UMath.cpp)
target_include_directories(Util PRIVATE "include")

install(TARGETS Util
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
