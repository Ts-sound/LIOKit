cmake_minimum_required(VERSION 3.20)

project(LIOKit VERSION 0.0.1)

option(LIOKIT_BUILD_TEST "build test" TRUE)


message("CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")


if(LIOKIT_BUILD_TEST)
  add_subdirectory(test)
endif()
