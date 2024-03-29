set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    mainTest.cpp
    test.cpp
    ${CMAKE_SOURCE_DIR}/app/InverseKinematics.cpp
    ${CMAKE_SOURCE_DIR}/app/Coordinate.cpp
    CoordinateTest.cpp
    ${CMAKE_SOURCE_DIR}/app/IJoint.cpp
    ${CMAKE_SOURCE_DIR}/app/PrismaticJoint.cpp
    PrismaticJointTest.cpp
    ${CMAKE_SOURCE_DIR}/app/RevoluteJoint.cpp
    InverseKinematicsTest.cpp
    ${CMAKE_SOURCE_DIR}/app/StraightLinePath.cpp
    StraightLinePathTest.cpp
    ${CMAKE_SOURCE_DIR}/app/DHTable.cpp
    DHTableTest.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include
                                           ${CMAKE_SOURCE_DIR}/../ThirdParty/eigen3
                                           ${CMAKE_SOURCE_DIR}/../ThirdParty/matplotlib-cpp)
target_link_libraries(cpp-test PUBLIC gtest)
