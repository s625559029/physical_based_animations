//
// Created by ysun3 on 10/31/17.
//

#ifndef ASSIGNMENT2_SOFTBODYSTATEDATA_H
#define ASSIGNMENT2_SOFTBODYSTATEDATA_H

#include "DynamicalState.h"
#include <iostream>

struct SoftEdge{
    size_t i;
    size_t j;
    double length;
};

class SoftBodyStateData : public pba::DynamicalStateData
{
public:
    SoftBodyStateData(const std::string& nam = "DynamicDataNoName") : DynamicalStateData(nam) ,
                                                                      ks(1.35),
                                                                      kf(4)
    {}
    ~SoftBodyStateData() {}
    void InitPairs();
    void ComputeAccels();
    std::vector<SoftEdge> connected_pairs;

    double ks;
    double kf;
};

void SoftBodyStateData::InitPairs()
{
    for(size_t i = 0; i < nb(); i++)
    {
        int temp = (int)sqrt(nb());
        if((int)i % temp != (temp - 1) && i < (nb() - temp))
        {
            SoftEdge se1;
            se1.i = i;
            se1.j = i+1;
            se1.length = (pos(se1.i) - pos(se1.j)).magnitude();
            connected_pairs.push_back(se1);

            SoftEdge se2;
            se2.i = i;
            se2.j = i+temp;
            se2.length = (pos(se2.i) - pos(se2.j)).magnitude();
            connected_pairs.push_back(se2);

            SoftEdge se3;
            se3.i = i;
            se3.j = i+temp+1;
            se3.length = (pos(se3.i) - pos(se3.j)).magnitude();
            connected_pairs.push_back(se3);

            SoftEdge se4;
            se4.i = i+1;
            se4.j = i+temp;
            se4.length = (pos(se4.i) - pos(se4.j)).magnitude();
            connected_pairs.push_back(se4);
        }
        else if((int)i % temp == (temp - 1) && i < (nb() - temp))
        {
            SoftEdge se;
            se.i = i;
            se.j = i+temp;
            se.length = (pos(se.i) - pos(se.j)).magnitude();
            connected_pairs.push_back(se);
        }
        else if((int)i % temp != (temp - 1) && i >= (nb() - temp))
        {
            SoftEdge se;
            se.i = i;
            se.j = i+1;
            se.length = (pos(se.i) - pos(se.j)).magnitude();
            connected_pairs.push_back(se);
        }
    }
}

void SoftBodyStateData::ComputeAccels()
{
    for(size_t i = 0; i < nb(); i++)
    {
        set_accel(i, pba::Vector(0, 0, 0));
    }
    for(auto m = connected_pairs.begin(); m != connected_pairs.end(); m++)
    {
        size_t i = m -> i;
        size_t j = m -> j;
        double Lij = m -> length;

        pba::Vector Xi = pos(i);
        pba::Vector Xj = pos(j);
        pba::Vector Vi = vel(i);
        pba::Vector Vj = vel(j);

        pba::Vector Xij = (Xj - Xi) / (Xj - Xi).magnitude();

        pba::Vector Fspring = ks * ((Xi - Xj).magnitude() - Lij) * Xij;
        pba::Vector Ffriction = kf * ((Vj - Vi) * Xij) * (Xj - Xi);
        pba::Vector Fstructure = Fspring + Ffriction;

        set_accel(i, accel(i) + Fstructure / mass(i));
        set_accel(j, accel(j) - Fstructure / mass(j));
    }
}

typedef std::shared_ptr<SoftBodyStateData> SoftBodyState;

SoftBodyState CreateSoftBodyState( const std::string& nam = "SoftBodyDataNoName" )
{
    return SoftBodyState( new SoftBodyStateData(nam) );
}

#endif //ASSIGNMENT2_SOFTBODYSTATEDATA_H
