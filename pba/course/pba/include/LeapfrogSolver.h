//
// Created by ysun3 on 9/13/17.
//

#ifndef PBA_LEAPFROGSOLVER_H
#define PBA_LEAPFROGSOLVER_H

#include "BaseSolver.h"

class LeapfrogSolver : BaseSolver
{
public:
    LeapfrogSolver() {}
    ~LeapfrogSolver() {}

    static void LeapfrogSolve(const double& dt, pba::DynamicalState DS, const std::vector<BaseForce *> forces)
    {
        PositionSolver(dt/2, DS);
        VelocitySolver(dt, DS, forces);
        PositionSolver(dt/2, DS);
    }

    static void LeapfrogSolveWithCollision(const double& dt, pba::DynamicalState DS, const std::vector<BaseForce *> forces, std::vector<Triangle> triangle_list, float cs, float cr)
    {
        PositionSolverWithCollision(dt/2, DS, triangle_list, cs, cr);
        VelocitySolver(dt, DS, forces);
        PositionSolverWithCollision(dt/2, DS, triangle_list, cs, cr);
    }

    static void SphereLeapfrogSolveWithCollision(const double& dt, SphereState SS, const std::vector<BaseForce *> forces, std::vector<Triangle> triangle_list, float cs, float cr)
    {
        SpherePositionSolverWithCollision(dt/2, SS, triangle_list, cs, cr);
        VelocitySolver(dt, SS, forces);
        SpherePositionSolverWithCollision(dt/2, SS, triangle_list, cs, cr);
    }
};

#endif //PBA_LEAPFROGSOLVER_H
