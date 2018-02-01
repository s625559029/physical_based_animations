cmake_minimum_required(VERSION 3.7)
project(course)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        pba/base/DynamicalState.C
        pba/base/LinearAlgebra.C
        pba/base/Matrix.C
        pba/base/ObjReader.C
        pba/base/PbaThing.C
        pba/base/PbaViewer.C
        pba/include/BaseForce.h
        pba/include/BaseSolver.h
        pba/include/Collision.h
        pba/include/Color.h
        pba/include/DynamicalState.h
        pba/include/Face.h
        pba/include/Gravity.h
        pba/include/LeapfrogSolver.h
        pba/include/LinearAlgebra.h
        pba/include/Matrix.h
        pba/include/ObjReader.h
        pba/include/PbaThing.h
        pba/include/PbaViewer.h
        pba/include/Point.h
        pba/include/RigidBodyCollision.h
        pba/include/RigidBodySolver.h
        pba/include/RigidBodyStateData.h
        pba/include/SixOrderSolver.h
        pba/include/SoftBodyStateData.h
        pba/include/Triangle.h
        pba/include/Types.h
        pba/include/Vector.h
        pba/swig/PbaThings.i
        pba/swig/PbaViewer.i
        pba/things/MyPbaThing.h
        pba/things/RigidBodyPbaThing.h
        pba/things/RotatingCube.h
        pba/things/SoftBodyPbaThing.h pba/include/SphereTriangleCollision.h pba/include/SphereStateData.h pba/things/SpherePbaThing.h pba/things/PongGamePbaThing.h)

add_executable(course ${SOURCE_FILES})