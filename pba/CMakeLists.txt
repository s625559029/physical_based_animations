cmake_minimum_required(VERSION 3.7)
project(ysun3_hw4)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        course/pba/base/DynamicalState.C
        course/pba/base/LinearAlgebra.C
        course/pba/base/Matrix.C
        course/pba/base/ObjReader.C
        course/pba/base/PbaThing.C
        course/pba/base/PbaViewer.C
        course/pba/include/BaseForce.h
        course/pba/include/BaseSolver.h
        course/pba/include/Collision.h
        course/pba/include/Color.h
        course/pba/include/DynamicalState.h
        course/pba/include/Face.h
        course/pba/include/Gravity.h
        course/pba/include/LeapfrogSolver.h
        course/pba/include/LinearAlgebra.h
        course/pba/include/Matrix.h
        course/pba/include/ObjReader.h
        course/pba/include/PbaThing.h
        course/pba/include/PbaViewer.h
        course/pba/include/Point.h
        course/pba/include/RigidBodyCollision.h
        course/pba/include/RigidBodySolver.h
        course/pba/include/RigidBodyStateData.h
        course/pba/include/SixOrderSolver.h
        course/pba/include/SoftBodyStateData.h
        course/pba/include/Triangle.h
        course/pba/include/Types.h
        course/pba/include/Vector.h
        course/pba/swig/PbaThings.i
        course/pba/swig/PbaViewer.i
        course/pba/things/MyPbaThing.h
        course/pba/things/RigidBodyPbaThing.h
        course/pba/things/RotatingCube.h
        course/pba/things/SoftBodyPbaThing.h)

add_executable(ysun3_hw4 ${SOURCE_FILES})