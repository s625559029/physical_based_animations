//
// Created by ysun3 on 10/20/17.
//

#ifndef ASSIGNMENT2_RIGIDBODYSOLVER_H
#define ASSIGNMENT2_RIGIDBODYSOLVER_H

#include "RigidBodyStateData.h"
#include "LinearAlgebra.h"
#include "BaseForce.h"
#include "RigidBodyCollision.h"

class RigidBodySolver
{
public:
    RigidBodySolver(){}
    ~RigidBodySolver(){}

    static void PositionSolver(const double& dt, RigidBodyState& RBS)
    {
        pba::Vector Xcm = RBS -> get_Xcm();
        pba::Vector Vcm = RBS -> get_Vcm();
        pba::Vector w = RBS -> get_w();
        pba::Matrix R = RBS -> get_R();

        RBS -> set_Xcm(Xcm + Vcm * dt);
        RBS -> set_R(pba::rotation(w.unitvector(), - w.magnitude() * dt) * R);

        //Compute momentum of inertia
        ComputeI(RBS);
    }

    static void VelocitySolver(const double& dt, RigidBodyState& RBS, const std::vector<BaseForce *> force_for_i)
    {
        pba::Vector Vcm = RBS -> get_Vcm();
        pba::Matrix I = RBS -> get_I();
        double M = RBS -> get_M();

        //Calculate force for center of mass
        pba::Vector Fcm(0,0,0);
        for(size_t i = 0; i < force_for_i.size(); i++)
        {
            Fcm += force_for_i[i] -> calForce(RBS, i);
        }

        //Calculate torque
        pba::Vector L(0,0,0);
        for(size_t i = 0; i < RBS -> nb(); i++)
        {
            L += RBS -> vert_rel_pos(i) ^ force_for_i[i] -> calForce(RBS, i);
        }

        //Update velocity
        RBS -> set_Vcm(Vcm + Fcm / M * dt);
        RBS -> set_w(RBS -> get_w() + I.inverse() * L * dt);
    }

    static void PositionSolverWithCollision(const double& dt, RigidBodyState& RBS, std::vector<Triangle> triangle_list)
    {
        PositionSolver(dt, RBS);
        CollisionOperate(dt, RBS, triangle_list);
    }
};

#endif //ASSIGNMENT2_RIGIDBODYSOLVER_H
