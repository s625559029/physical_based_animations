//
// Created by ysun3 on 9/17/17.
//

#ifndef COURSE_BASEFORCE_H
#define COURSE_BASEFORCE_H

#include <string>
#include "Vector.h"
#include "DynamicalState.h"

class BaseForce
{
public:
    BaseForce(const std::string nam = "CollidingParticlesThing") : name(nam) {}
    virtual ~BaseForce() {}

    virtual pba::Vector calForce (pba::DynamicalState DS, size_t i) const = 0;
protected:
    std::string name;
};

#endif //COURSE_BASEFORCE_H
