add_executable(shell-app main.cpp
 InverseKinematics.cpp
 StraightLinePath.cpp
 Coordinate.cpp
 IJoint.cpp
 PrismaticJoint.cpp
 RevoluteJoint.cpp
 Demo.cpp
 DHTable.cpp
)
 message( ${CMAKE_SOURCE_DIR}/../../Eigen)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/../ThirdParty/eigen3
    ${CMAKE_SOURCE_DIR}/../ThirdParty/matplotlib-cpp
)

find_package(PythonLibs 2.7)
message("${PYTHON_INCLUDE_DIR}")
message("${PYTHON_LIBRARY}")
target_include_directories( shell-app PRIVATE ${PYTHON_INCLUDE_DIR})
target_link_libraries( shell-app ${PYTHON_LIBRARY})
