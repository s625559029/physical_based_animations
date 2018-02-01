//
// Created by ysun3 on 9/21/17.
//

#ifndef COURSE_COLLISION_H
#define COURSE_COLLISION_H

#include "DynamicalState.h"
#include "Triangle.h"

bool CollisionDetection(double& dt, Triangle t, double& max_d_ti, Triangle& ti_t, pba::DynamicalState DS, size_t i)
{
    pba::Vector p, v;
    p = DS->pos(i);
    v = DS->vel(i);

    pba::Vector p0, p1, p2;
    pba::Vector e1, e2;
    pba::Vector np;

    p0 = t.getV1();
    p1 = t.getV2();
    p2 = t.getV3();

    e1 = p1 - p0;
    e2 = p2 - p0;

    np = t.getNp();

    //Judge f0 & f1
    float f0, f1;
    f0 = (p - p0) * np;
    if (f0 == 0) return false;
    f1 = (p - v * dt - p0) * np;
    if ((f0 * f1) > 0) return false;
    //std::cout<<"f0 f1 finished" << std::endl;

    //Judge d_ti
    double d_ti;
    d_ti = ((p - p0) * np) / (v * np);
    if (dt * d_ti < 0 || abs(d_ti) > abs(dt)) return false;
    if ((dt - d_ti) / dt < 0.000001) return false;
    //std::cout<<"d_ti finished" << std::endl;

    //Judge a & b from triangle
    pba::Vector pi = p - v * d_ti;

    float a, b;
    a = ((e2 ^ e1) * (e2 ^ (pi - p0))) / (pow((e2 ^ e1).magnitude(), 2));
    //std::cout << "a: " << a << std::endl;
    if (a < 0 || a > 1) return false;
    b = ((e1 ^ e2) * (e1 ^ (pi - p0))) / (pow((e1 ^ e2).magnitude(), 2));
    //std::cout << "b: " << b << std::endl;
    if (b < 0 || b > 1) return false;
    if (a + b < 0 || a + b > 1) return false;
    //std::cout<<"a b finished" << std::endl;

    //Handle reflect collision
    if(abs(d_ti) > abs(max_d_ti))
    {
        max_d_ti = d_ti;
        ti_t = t;
    }
    return true;
}

void CollisionHandle(const double& d_ti, Triangle& t, pba::DynamicalState DS, size_t i, float cs, float cr)
{
    pba::Vector v, vr, pi, p, np, v_vertical;

    np = t.getNp();

    v = DS->vel(i);

    v_vertical = v - np*(np * v);

    vr = cs * v_vertical - cr * np * (np * v);

    p = DS->pos(i);
    pi = p - v * d_ti;
    p = pi + vr * d_ti;

    DS->set_pos(i, p);
    DS->set_vel(i, vr);
}

static void CollisionOperation(const double& dt, pba::DynamicalState DS, std::vector<Triangle> triangle_list, float cs, float cr)
{
    for(size_t i=0; i<DS->nb(); i++)
    {
        bool collisionFlag = true;

        double max_d_ti = 0;
        double temp_dt = dt;

        Triangle ti_t;

        while(collisionFlag)
        {

            collisionFlag = false;
            for(std::vector<Triangle>::iterator iter = triangle_list.begin(); iter != triangle_list.end(); iter++)
            {
                collisionFlag = collisionFlag || CollisionDetection(temp_dt, *iter, max_d_ti, ti_t, DS, i);
            }
            temp_dt = max_d_ti;
            max_d_ti = 0;
            if(collisionFlag == true) {
                CollisionHandle(temp_dt, ti_t, DS, i, cs, cr);
            }
        }

    }
}

#endif //COURSE_COLLISION_H
