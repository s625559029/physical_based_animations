//
// Created by ysun3 on 10/21/17.
//

#ifndef ASSIGNMENT2_RIGIDBODYCOLLISION_H
#define ASSIGNMENT2_RIGIDBODYCOLLISION_H

#include "Triangle.h"
#include "RigidBodyStateData.h"


void ComputeI(RigidBodyState& RBS)
{
    pba::Matrix tmp_matrix;
    for(int m = 0; m < 3; m++)
    {
        for(int n = 0; n < 3; n++)
        {
            int k = 0;
            if(m == n) k=1;

            double sum = 0;

            for(size_t i = 0; i < RBS -> nb(); i++)
            {
                pba::Vector ri = RBS -> vert_rel_pos(i);
                sum += RBS -> mass(i) * (pow(ri.magnitude(), 2) * k - ri.get(m) * ri.get(n));
            }
            tmp_matrix.Set(m, n, sum);
        }
    }
    RBS -> set_I(tmp_matrix);
}

bool inTriangle(pba::Vector x, Triangle t)
{
    pba::Vector p0, p1, p2, e1, e2;

    p0 = t.getV1();
    p1 = t.getV2();
    p2 = t.getV3();

    e1 = p1 - p0;
    e2 = p2 - p0;

    double a, b;
    a = ((e2 ^ e1) * (e2 ^ (x - p0))) / (pow((e2 ^ e1).magnitude(), 2));
    if (a < 0.0 || a > 1.0) return false;

    b = ((e1 ^ e2) * (e1 ^ (x - p0))) / (pow((e1 ^ e2).magnitude(), 2));
    if (b < 0.0 || b > 1.0) return false;

    if (a + b < 0.0 || a + b > 1.0) return false;

    return true;
}

//Dosen't work for some reason, works well when merge its code in CollisionOperate.
bool CollisionDetect(double& dt, RigidBodyState& RBS, Triangle t, double& max_dt_i, Triangle& ti_t, size_t& max_particle, size_t i)
{
    pba::Vector xi = RBS -> ver_pos(i);

    pba::Vector xp = t.getV1();
    pba::Vector np = t.getNp();

    double f0 = (xi - xp) * np;
    if(f0 == 0.0) return false;

    pba::Matrix U = rotation(RBS -> get_w() / RBS->get_w().magnitude(), RBS -> get_w().magnitude() * dt) * RBS->get_R();
    double f1 = (U * RBS -> pos(i) + RBS -> get_Xcm() - (RBS->get_Vcm() * dt) - xp) * np;
    if(f0 * f1 > 0.0) return false;
    if(f1 == 0.0)
    {
        if(fabs(dt) > fabs(max_dt_i))
        {
            max_dt_i = dt;
            ti_t = t;
            max_particle = i;
        }
        return true;
    }

    double t0 = 0.0;
    double t1 = dt;
    double  dt_i;
    bool isConverged = false;
    while(!isConverged)
    {
        dt_i = (t0 + t1) / 2.0;
        pba::Vector w = RBS -> get_w();
        pba::Matrix R = RBS -> get_R();
        pba::Vector pi = RBS -> pos(i);
        pba::Vector Xcm = RBS -> get_Xcm();
        pba::Vector Vcm = RBS -> get_Vcm();
        U = pba::rotation(w.unitvector(), w.magnitude() * dt_i) * R;
        double ff = (U * pi + Xcm - Vcm * dt_i - xp) * np;
        if(ff == 0.0) {isConverged = true; continue;}
        else if(ff * f0 < 0.0)
        {
            f1 = ff;
            t1 = dt_i;
        }
        else
        {
            f0 = ff;
            t0 = dt_i;
        }
        dt_i = t1;
        if(abs((t0 - t1) / dt) < 0.0001){ isConverged = true; continue;}
    }

    U = pba::rotation(RBS->get_w().unitvector(), RBS->get_w().magnitude() * dt_i) * RBS->get_R();
    pba::Vector x_collision = U * RBS->pos(i) + RBS->get_Xcm() - RBS->get_Vcm() * dt_i;

    //Detect if the x_collision is outside the triangle
    pba::Vector p0, p1, p2, e1, e2;

    p0 = t.getV1();
    p1 = t.getV2();
    p2 = t.getV3();

    e1 = p1 - p0;
    e2 = p2 - p0;

    double a, b;
    a = ((e2 ^ e1) * (e2 ^ (x_collision - p0))) / (pow((e2 ^ e1).magnitude(), 2));
    if (a < 0.0 || a > 1.0) return false;

    b = ((e1 ^ e2) * (e1 ^ (x_collision - p0))) / (pow((e1 ^ e2).magnitude(), 2));
    if (b < 0.0 || b > 1.0) return false;

    if (a + b < 0.0 || a + b > 1.0) return false;

    if(dt * dt_i < 0.0) return false;
    if(abs(dt_i) > abs(dt)) return false;
    if ((dt - dt_i) / dt < 0.000001) return false;

    if(fabs(dt_i) > fabs(max_dt_i))
    {
        max_dt_i = dt_i;
        ti_t = t;
        max_particle = i;
    }
    return true;
}

void CollisionHandle(const double dt_i, Triangle t, RigidBodyState& RBS, size_t max_particle)
{
    RBS -> set_Xcm(RBS -> get_Xcm() - RBS -> get_Vcm() * dt_i);
    RBS -> set_R(pba::rotation(RBS -> get_w().unitvector(), RBS -> get_w().magnitude() * dt_i) * RBS -> get_R());

    ComputeI(RBS);

    pba::Vector q = RBS -> get_I().inverse() * (RBS -> vert_rel_pos(max_particle) ^ t.getNp());
    double A = -(2 * RBS -> get_Vcm() * t.getNp() + q * RBS -> get_I() * RBS -> get_w() +
            RBS -> get_w() * RBS -> get_I() * q) / (1 / RBS -> get_M() + q * RBS -> get_I() * q);

    RBS -> set_Vcm(RBS -> get_Vcm() + A * t.getNp() / RBS -> get_M());
    RBS -> set_w(RBS -> get_w() + A * q);

    RBS -> set_Xcm(RBS -> get_Xcm() + RBS -> get_Vcm() * dt_i);
    RBS -> set_R(pba::rotation(RBS -> get_w().unitvector(), - RBS -> get_w().magnitude() * dt_i) * RBS -> get_R());
}

void CollisionOperate(double dt, RigidBodyState& RBS, std::vector<Triangle> triangle_list) {
    bool collisionFlag = true;
    while (collisionFlag)
    {
        collisionFlag = false;

        double max_d_ti = 0.0;
        Triangle ti_t;
        size_t max_particle = 0;

        for (size_t i = 0; i < RBS->nb(); i++)
        {
            for (std::vector<Triangle>::iterator iter = triangle_list.begin(); iter != triangle_list.end(); iter++)
            {
                Triangle t = *iter;

                pba::Vector xi = RBS -> ver_pos(i);

                pba::Vector xp = t.getV1();
                pba::Vector np = t.getNp();

                double f0 = (xi - xp) * np;
                if(f0 == 0.0) {collisionFlag = collisionFlag || false; continue;}

                pba::Matrix U = rotation(RBS -> get_w() / RBS->get_w().magnitude(), RBS -> get_w().magnitude() * dt) * RBS->get_R();
                double f1 = (U * RBS -> pos(i) + RBS -> get_Xcm() - (RBS->get_Vcm() * dt) - xp) * np;
                if(f0 * f1 > 0.0) {collisionFlag = collisionFlag || false;continue;}
                if(f1 == 0.0)
                {
                    if(fabs(dt) > fabs(max_d_ti))
                    {
                        max_d_ti = dt;
                        ti_t = t;
                        max_particle = i;
                    }
                    collisionFlag = collisionFlag || true;
                    continue;
                }

                double t0 = 0.0;
                double t1 = dt;
                double  dt_i;
                bool isConverged = false;
                while(!isConverged)
                {
                    dt_i = (t0 + t1) / 2.0;
                    pba::Vector w = RBS -> get_w();
                    pba::Matrix R = RBS -> get_R();
                    U = pba::rotation(w.unitvector(), w.magnitude() * dt_i) * R;
                    pba::Vector pi = RBS -> pos(i);
                    pba::Vector Xcm = RBS -> get_Xcm();
                    pba::Vector Vcm = RBS -> get_Vcm();
                    double ff = (U * pi + Xcm - Vcm * dt_i - xp) * np;
                    if(ff == 0.0) {isConverged = true; continue;}
                    else if(ff * f0 < 0.0)
                    {
                        f1 = ff;
                        t1 = dt_i;
                    }
                    else
                    {
                        f0 = ff;
                        t0 = dt_i;
                    }
                    dt_i = t1;
                    if(abs((t0 - t1) / dt) < 0.0001){ isConverged = true; continue;}
                }

                U = pba::rotation(RBS->get_w().unitvector(), RBS->get_w().magnitude() * dt_i) * RBS->get_R();
                pba::Vector x_collision = U * RBS->pos(i) + RBS->get_Xcm() - RBS->get_Vcm() * dt_i;

                //Detect if the x_collision is outside the triangle
                pba::Vector p0, p1, p2, e1, e2;

                p0 = t.getV1();
                p1 = t.getV2();
                p2 = t.getV3();

                e1 = p1 - p0;
                e2 = p2 - p0;

                double a, b;
                a = ((e2 ^ e1) * (e2 ^ (x_collision - p0))) / (pow((e2 ^ e1).magnitude(), 2));
                if (a < 0.0 || a > 1.0) {collisionFlag = collisionFlag || false;
                    continue;}

                b = ((e1 ^ e2) * (e1 ^ (x_collision - p0))) / (pow((e1 ^ e2).magnitude(), 2));
                if (b < 0.0 || b > 1.0) {collisionFlag = collisionFlag || false;
                    continue;}

                if (a + b < 0.0 || a + b > 1.0) {collisionFlag = collisionFlag || false;
                    continue;}

                if(dt * dt_i < 0.0) {collisionFlag = collisionFlag || false;
                    continue;}
                if(abs(dt_i) > abs(dt)) {collisionFlag = collisionFlag || false;
                    continue;}
                if ((dt - dt_i) / dt < 0.000001) {collisionFlag = collisionFlag || false;
                    continue;}

                if(fabs(dt_i) > fabs(max_d_ti))
                {
                    max_d_ti = dt_i;
                    ti_t = t;
                    max_particle = i;
                }
                collisionFlag = collisionFlag || true;
            }
        }

        if (collisionFlag == true)
        {
            dt = max_d_ti;
            CollisionHandle(dt, ti_t, RBS, max_particle);
        }
    }
}

#endif //ASSIGNMENT2_RIGIDBODYCOLLISION_H
