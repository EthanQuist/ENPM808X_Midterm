set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    mainTest.cpp
    test.cpp
    ${CMAKE_SOURCE_DIR}/app/InverseKinematics.cpp
    ${CMAKE_SOURCE_DIR}/app/Coordinate.cpp
    ${CMAKE_SOURCE_DIR}/app/Joints.cpp
    InverseKinematicsTest.cpp
    ${CMAKE_SOURCE_DIR}/app/StraightLinePath.cpp
    StraightLinePathTest.cpp
    
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
