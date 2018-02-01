//
// Created by ysun3 on 11/21/17.
//

#ifndef COURSE_SPHERESTATEDATA_H
#define COURSE_SPHERESTATEDATA_H

#include "DynamicalState.h"
#include <iostream>

class SphereStateData : public pba::DynamicalStateData
{
public:
    SphereStateData(const std::string& nam = "DynamicDataNoName") : DynamicalStateData(nam)
    {}
    ~SphereStateData() {}

    void InitRadius();

    std::vector<double> radius;
};

void SphereStateData::InitRadius()
{
    for(size_t i = 0; i < nb(); i++)
    {
        radius.push_back(1);
    }
}

typedef std::shared_ptr<SphereStateData> SphereState;

SphereState CreateSphereState( const std::string& nam = "SoftBodyDataNoName" )
{
    return SphereState( new SphereStateData(nam) );
}

#endif //COURSE_SPHERESTATEDATA_H
