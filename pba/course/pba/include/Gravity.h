//
// Created by ysun3 on 9/14/17.
//

#ifndef PBA_GRAVITY_H
#define PBA_GRAVITY_H

#include "BaseForce.h"

class Gravity : public BaseForce
{
public:
    Gravity(const std::string nam = "Gravity") : BaseForce(nam) {};
    ~Gravity() {};
    void set_g(float _g) { g = _g; };
    pba::Vector calForce(pba::DynamicalState DS, size_t i) const
    {
        return DS->mass(i) * g * pba::Vector(0, -1, 0);
    }
private:
    float g;
};

#endif //PBA_GRAVITY_H
