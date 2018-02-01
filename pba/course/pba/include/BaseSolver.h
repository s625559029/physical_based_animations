//
// Created by ysun3 on 9/13/17.
//

#ifndef PBA_BASESOLVER_H
#define PBA_BASESOLVER_H

#include <iostream>
#include "DynamicalState.h"
#include "Gravity.h"
#include "Triangle.h"
#include "Collision.h"
#include "SphereTriangleCollision.h"

class BaseSolver
{
public:
    BaseSolver () {}
    ~BaseSolver () {}

protected:
    static void PositionSolver(const double& dt, pba::DynamicalState DS)
    {
        for(size_t i=0; i<DS->nb(); i++) {
            DS->set_pos(i, DS->pos(i)+dt*DS->vel(i));
        }

    }

    static void VelocitySolver(const double& dt, pba::DynamicalState DS, const std::vector<BaseForce *> forces)
    {
        for(size_t i=0; i<DS->nb(); i++) {
            pba::Vector force(0,0,0);
            for(size_t i=0; i < forces.size(); i++)
            {
                force += forces[i] -> calForce(DS,i);
            }
            DS->set_vel(i, DS->vel(i) + dt * (DS->accel(i) + force / DS->mass(i)));
        }

    }

    static void PositionSolverWithCollision(const double& dt, pba::DynamicalState DS, std::vector<Triangle> triangle_list, float cs, float cr)
    {
        PositionSolver(dt, DS);
        CollisionOperation(dt, DS, triangle_list, cs, cr);
    }

    static void SpherePositionSolverWithCollision(const double& dt, SphereState SS, std::vector<Triangle> triangle_list, float cs, float cr)
    {
        PositionSolver(dt, SS);
        SphereCollisionOperation(dt, SS, triangle_list, cs, cr);
    }

};

#endif //PBA_BASESOLVER_H
