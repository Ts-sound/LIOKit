cmake_minimum_required(VERSION 3.20)

add_executable(vcpkg_test vcpkg_test.cpp)

find_package(GTSAM REQUIRED)

target_link_libraries(vcpkg_test gtsam)


