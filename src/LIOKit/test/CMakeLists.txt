cmake_minimum_required(VERSION 3.20)

add_executable(vcpkg_test_main vcpkg_test_main.cpp)
target_link_libraries(vcpkg_test_main PRIVATE gtsam)

#####################################
# package_test
#####################################
file(GLOB_RECURSE PACKAGE_TEST
    "*_test.cpp"
    "*_test.h"
)

# gtest
find_package(GTest CONFIG REQUIRED)

add_executable(gtest_main gtest_main.cpp
    ${PACKAGE_SRC}
    ${PACKAGE_TEST}
)
target_include_directories(gtest_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(gtest_main PRIVATE
    gtsam
    GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main
)

