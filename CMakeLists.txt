cmake_minimum_required(VERSION 3.2.1)
project (scratch)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# We probably don't want this to run on every build.
option(COVERAGE "Generate Coverage Data" OFF)

if (COVERAGE)
    include(CodeCoverage)
    set(LCOV_REMOVE_EXTRA "'vendor/*'")
    setup_target_for_coverage(code_coverage test/cpp-test coverage)
    set(COVERAGE_SRCS app/main.cpp
                      app/Coordinate.cpp
                      app/DHTable.cpp
                      app/Demo.cpp
                      app/IJoint.cpp
                      app/InverseKinematics.cpp
                      app/PrismaticJoint.cpp
                      app/RevoluteJoint.cpp
                      app/StraightLinePath.cpp
                      include/Coordinate.hpp
                      include/Demo.hpp
                      include/DHTable.hpp
                      include/IJoint.hpp
                      include/IPathPlanner.hpp
                      include/PrismaticJoint.hpp
                      include/RevoluteJoint.hpp
                      include/InverseKinematics.hpp 
                      include/StraightLinePath.hpp
                      include/Demo.hpp 
                      )

    SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
    SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
    SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")
else()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -g")
endif()

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(app)
add_subdirectory(test)
add_subdirectory(vendor/googletest/googletest)
