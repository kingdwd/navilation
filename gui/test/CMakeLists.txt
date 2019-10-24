include_directories(${gui_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/Util/include) # include files from ProjectB
#set(GTest_DIR ${CMAKE_SOURCE_DIR}/lib)
#set(GTEST_ROOT ${CMAKE_SOURCE_DIR}/lib/gtest)
find_package(GTest REQUIRED PATHS ${CMAKE_SOURCE_DIR}/lib/gtest)
add_executable(runGuiTests controller_test.cpp leet_code_test.cpp linked_list.cpp linked_list_test.cpp)
target_link_libraries(runGuiTests GTest::gtest GTest::gtest_main Util)
