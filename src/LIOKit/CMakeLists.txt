cmake_minimum_required(VERSION 3.20)

project(LIOKit VERSION 0.0.1)

option(LIOKIT_BUILD_TEST "build test" TRUE)


message("CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")

find_package(glog REQUIRED)
find_package(GTSAM REQUIRED)



if(LIOKIT_BUILD_TEST)
  add_subdirectory(test)
endif()
