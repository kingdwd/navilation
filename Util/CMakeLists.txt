project(Util)
add_subdirectory(src)
add_subdirectory(test)
target_include_directories(Util PRIVATE "include")

#install(TARGETS Util
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib)
