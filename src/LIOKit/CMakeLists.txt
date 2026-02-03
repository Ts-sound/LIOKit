cmake_minimum_required(VERSION 3.20)

project(LIOKit VERSION 0.0.1)

option(LIOKIT_BUILD_TEST "build test" TRUE)


message("CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")

find_package(glog REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#####################################
# package_src
#####################################
file(GLOB_RECURSE PACKAGE_SRC
  "src/*.cpp"
  "src/*.h"
)

message(STATUS "PACKAGE_SRC")
foreach(file ${PACKAGE_SRC})
  message(STATUS "${file}")
endforeach(file ${PACKAGE_SRC})

add_library(${CMAKE_PROJECT_NAME} SHARED ${PACKAGE_SRC})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
  gtsam
)

if(LIOKIT_BUILD_TEST)
  add_subdirectory(test)
endif()
