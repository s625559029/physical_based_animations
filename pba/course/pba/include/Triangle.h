//
// Created by ysun3 on 9/21/17.
//

#ifndef COURSE_TRIANGLE_H
#define COURSE_TRIANGLE_H

#include "Vector.h"

class Triangle
{
public:
    Triangle() {}
    Triangle(const pba::Vector & _v1, const pba::Vector & _v2, const pba::Vector & _v3) :
            v1(_v1), v2(_v2), v3(_v3), np()
    {
        pba::Vector e1 = v2 - v1;
        pba::Vector e2 = v3 - v1;
        np = (e2 ^ e1) / (e2 ^ e1).magnitude();
    }
    ~Triangle() {}
    pba::Vector getV1() { return v1; }
    pba::Vector getV2() { return v2; }
    pba::Vector getV3() { return v3; }
    void setV1(pba::Vector& V) { v1 = V; }
    void setV2(pba::Vector& V) { v2 = V; }
    void setV3(pba::Vector& V) { v3 = V; }
    pba::Vector getNp() { return np; }
private:
    pba::Vector v1, v2, v3;
    pba::Vector np;
};

#endif //COURSE_TRIANGLE_H
