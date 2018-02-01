//
// Created by ysun3 on 9/13/17.
//

#ifndef PBA_SIXORDERSOLVER_H
#define PBA_SIXORDERSOLVER_H

#include "LeapfrogSolver.h"

class SixOrderSolver : LeapfrogSolver
{
public:
    SixOrderSolver() {}
    ~SixOrderSolver() {}

    static void SixOrderSolve(const double& dt, pba::DynamicalState DS, const std::vector<BaseForce *> forces)
    {
        double a = 1 / (4 - pow(4, (1/3)));
        double b = 1 - (4 * a);
        double dt_a = a * dt;
        double dt_b = b * dt;

        LeapfrogSolve(dt_a, DS, forces);
        LeapfrogSolve(dt_a, DS, forces);
        LeapfrogSolve(dt_b, DS, forces);
        LeapfrogSolve(dt_a, DS, forces);
        LeapfrogSolve(dt_a, DS, forces);
    }

    static void SixOrderSolveWithCollision(const double& dt, pba::DynamicalState DS, const std::vector<BaseForce *> forces, std::vector<Triangle> triangle_list, float cs, float cr)
    {
        double a = 1.0 / (4.0 - pow(4.0, (1.0/3.0)));
        double b = 1.0 - (4.0 * a);
        double dt_a = a * dt;
        double dt_b = b * dt;

        LeapfrogSolveWithCollision(dt_a, DS, forces, triangle_list, cs, cr);
        LeapfrogSolveWithCollision(dt_a, DS, forces, triangle_list, cs, cr);
        LeapfrogSolveWithCollision(dt_b, DS, forces, triangle_list, cs, cr);
        LeapfrogSolveWithCollision(dt_a, DS, forces, triangle_list, cs, cr);
        LeapfrogSolveWithCollision(dt_a, DS, forces, triangle_list, cs, cr);
    }
};

#endif //PBA_SIXORDERSOLVER_H
