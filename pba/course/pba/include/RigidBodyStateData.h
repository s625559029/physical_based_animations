//
// Created by ysun3 on 10/20/17.
//

#ifndef ASSIGNMENT2_RIGIDBODYSTATEDATA_H
#define ASSIGNMENT2_RIGIDBODYSTATEDATA_H

#include "DynamicalState.h"
#include "Matrix.h"
#include "LinearAlgebra.h"
#include <iostream>

class RigidBodyStateData : public pba::DynamicalStateData
{
public:
    RigidBodyStateData(const std::string& nam = "DynamicDataNoName") : DynamicalStateData(nam),
    initial_velocity(1)
    {}
    ~RigidBodyStateData() {}

    double get_M(){ return total_mass; }
    pba::Vector get_Xcm(){ return center_of_mass; }
    pba::Vector get_Vcm(){ return center_of_mass_velocity; }
    pba::Matrix get_R(){ return angular_rotation; }
    pba::Vector get_w(){ return angular_velocity; }
    pba::Matrix get_I(){ return moment_of_inertia; }

    void set_M(double tmp) { total_mass = tmp; }
    void set_Xcm(pba::Vector tmp) { center_of_mass = tmp; };
    void set_Vcm(pba::Vector tmp) { center_of_mass_velocity = tmp; }
    void set_R(pba::Matrix tmp) { angular_rotation = tmp; }
    void set_w(pba::Vector tmp) { angular_velocity = tmp; }
    void set_I(pba::Matrix tmp) { moment_of_inertia = tmp; }

    void Init();

    pba::Vector vert_rel_pos(size_t i)      //ri
    {
        pba::Vector ri;
        return ri = angular_rotation * pos(i);
    }
    pba::Vector ver_pos(size_t i)           //xi
    {
        pba::Vector xi;
        return xi = center_of_mass + vert_rel_pos(i);
    }

    double initial_velocity;

private:
    double total_mass;                      //M
    pba::Matrix moment_of_inertia;          //I
    pba::Vector center_of_mass;             //Xcm
    pba::Matrix angular_rotation;           //R
    pba::Vector center_of_mass_velocity;    //Vcm
    pba::Vector angular_velocity;           //w
};

void RigidBodyStateData::Init()
{
    total_mass = 0;
    for(size_t i = 0; i < nb(); i++)
    {
        total_mass += mass(i);
    }

    pba::Vector tmp;
    for(size_t i = 0; i < nb(); i++)
    {
        tmp += mass(i) * pos(i);
    }
    center_of_mass = tmp / total_mass;

    center_of_mass_velocity = pba::Vector(drand48() * initial_velocity, drand48() * initial_velocity, drand48() * initial_velocity);

    angular_rotation = pba::unitMatrix();

    angular_velocity = pba::Vector(drand48(), drand48(), drand48());

    for(size_t i = 0; i < nb(); i++)
    {
        set_pos(i, pos(i) - center_of_mass);
    }
}

typedef std::shared_ptr<RigidBodyStateData> RigidBodyState;

RigidBodyState CreateRigidBodyState( const std::string& nam = "RigidBodyDataNoName" )
{
    return RigidBodyState( new RigidBodyStateData(nam) );
}

#endif //ASSIGNMENT2_RIGIDBODYSTATEDATA_H
