cmake_minimum_required(VERSION 3.15)

project(LIOKit VERSION 0.0.1)

option(LIOKIT_BUILD_TEST)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg_export/vcpkg//scripts/buildsystems/vcpkg.cmake)
message("CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")


if(LIOKIT_BUILD_TEST)
  add_subdirectory(test)
endif()
