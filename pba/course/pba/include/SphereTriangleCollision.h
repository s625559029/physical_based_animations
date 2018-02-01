//
// Created by ysun3 on 11/21/17.
//

#ifndef COURSE_SPHERETRIANGLECOLLISION_H
#define COURSE_SPHERETRIANGLECOLLISION_H

#include "SphereStateData.h"
#include "Triangle.h"

double tttt = 0;

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

bool EdgeIntersectionDetect(pba::Vector e, pba::Vector S, double R, pba::Vector V, pba::Vector p, double& dt_i, double dt)
{
        pba::Vector V_p = V - e * (e * V) / pow(e.magnitude(), 2);
        pba::Vector S_p = (S - p) - e * ((S - p) * e) / pow(e.magnitude(), 2);
        double result = pow((V_p * S_p), 2) + pow(V_p.magnitude(), 2) * (pow(S_p.magnitude(), 2) - pow(R, 2));
        if(result >= 0)
        {
            double q = e * (S - p) / pow(e.magnitude(), 2);
            if (q >= 0 && q <= 1) {
                double q_delta = pow(e * (S - p), 2.0) + (e * e) * (R * R - (S - p) * (S - p));
                if(q_delta < 0) return false;

                double dt_i_1 = -(V_p * S_p) / pow(V_p.magnitude(), 2) +
                                sqrt(result) / pow(V_p.magnitude(), 2);
                double dt_i_2 = -(V_p * S_p) / pow(V_p.magnitude(), 2) -
                                sqrt(result) / pow(V_p.magnitude(), 2);
                if (fabs(dt_i_1) < fabs(dt) && dt_i_1 * dt > 0) {
                    dt_i = dt_i_1;
                    return true;
                } else if (fabs(dt_i_2) < fabs(dt) && dt_i_2 * dt > 0) {
                    dt_i = dt_i_2;
                    return true;
                }
            }
        }
    return false;
}

bool SphereCollisionDetection(double& dt, Triangle t, double& max_dt_i, Triangle& ti_t, SphereState SS, size_t i)
{
    pba::Vector p0 = t.getV1();
    pba::Vector p1 = t.getV2();
    pba::Vector p2 = t.getV3();
    pba::Vector np = t.getNp();

    pba::Vector S = SS -> pos(i);
    pba::Vector V = SS -> vel(i);
    double R = SS -> radius[i];

    double dt_i;

    if(fabs((S - p0) * np) < R)
    {
        //Detect whether the sphere intersect with the triangle
        double dt_i_1 = ((S - p0) * np + R) / (SS -> vel(i) * np);
        double dt_i_2 = ((S - p0) * np - R) / (SS -> vel(i) * np);

        if(fabs(dt_i_1) < fabs(dt) && dt_i_1 * dt > 0)
        {
            dt_i = dt_i_1;
            if(fabs(dt_i) > fabs(dt)) std::cout<<dt_i<<std::endl;
            pba::Vector S_dash = S - V * dt_i;
            pba::Vector p_dash = S_dash + np.unitvector() * R;

            if(inTriangle(p_dash, t))
            {
                if(fabs(dt_i) > fabs(max_dt_i))
                {
                    max_dt_i = dt_i;
                    ti_t = t;
                }
                return true;
            }
        }
        else if(fabs(dt_i_2) < fabs(dt) && dt_i_2 * dt > 0)
        {
            dt_i = dt_i_2;
            if(fabs(dt_i) > fabs(dt)) std::cout<<dt_i<<std::endl;
            pba::Vector S_dash = S - V * dt_i;
            pba::Vector p_dash = S_dash + np.unitvector() * R;

            if(inTriangle(p_dash, t))
            {
                if(fabs(dt_i) > fabs(max_dt_i))
                {
                    max_dt_i = dt_i;
                    ti_t = t;
                }
                return true;
            }
        }
        //Detect whether the sphere intersect with the edges
        else
        {
            //Detect for edge p1 - p0
            pba::Vector e0 = p1 - p0;
            if(EdgeIntersectionDetect(e0, S, R, V, p0, dt_i, dt))
            {
                if(fabs(dt_i) > fabs(max_dt_i))
                {
                    max_dt_i = dt_i;
                    ti_t = t;
                    tttt = max_dt_i;
                }
                return true;
            }
            else
            {
                //Detect for edge p2 - p1
                pba::Vector e1 = p2 - p1;
                if(EdgeIntersectionDetect(e1, S, R, V, p1, dt_i, dt))
                {
                    if(fabs(dt_i) > fabs(max_dt_i))
                    {
                        max_dt_i = dt_i;
                        ti_t = t;
                        tttt = max_dt_i;
                    }
                    return true;
                }
                else
                {
                    //Detect for edge p2 - p0
                    pba::Vector e2 = p2 - p0;
                    if(EdgeIntersectionDetect(e2, S, R, V, p0, dt_i, dt))
                    {
                        if(fabs(dt_i) > fabs(max_dt_i))
                        {
                            max_dt_i = dt_i;
                            ti_t = t;
                            tttt = max_dt_i;
                        }
                        return true;
                    }
                    else
                    {
                        if(pow((S - p0).magnitude(), 2) <= pow(R, 2) && pow((V * (S - p0)), 2) - pow(V.magnitude(), 2) * (pow((S - p0).magnitude(), 2) - pow(R, 2)) >= 0) {
                            dt_i_1 = (V * (S - p0) + sqrt(pow((V * (S - p0)), 2) - pow(V.magnitude(), 2) *
                                                                                   (pow((S - p0).magnitude(), 2) -
                                                                                    pow(R, 2)))) /
                                     pow(V.magnitude(), 2);
                            dt_i_2 = (V * (S - p0) - sqrt(pow((V * (S - p0)), 2) - pow(V.magnitude(), 2) *
                                                                                   (pow((S - p0).magnitude(), 2) -
                                                                                    pow(R, 2)))) /
                                     pow(V.magnitude(), 2);
                            if(fabs(dt_i_1) < fabs(dt) && dt_i_1 * dt > 0)
                            {
                                dt_i = dt_i_1;
                                if(fabs(dt_i) > fabs(max_dt_i))
                                {
                                    max_dt_i = dt_i;
                                    ti_t = t;
                                }
                                return true;
                            }
                            else if(fabs(dt_i_2) < fabs(dt) && dt_i_2 * dt > 0)
                            {
                                dt_i = dt_i_2;
                                if(fabs(dt_i) > fabs(max_dt_i))
                                {
                                    max_dt_i = dt_i;
                                    ti_t = t;
                                }
                                return true;
                            }
                        }
                        else if(pow((S - p1).magnitude(), 2) <= pow(R, 2) && pow((V * (S - p1)), 2) - pow(V.magnitude(), 2) * (pow((S - p1).magnitude(), 2) - pow(R, 2)) >= 0)
                        {
                            dt_i_1 = (V * (S - p1) + sqrt(pow((V * (S - p1)), 2) - pow(V.magnitude(), 2) *
                                                                                   (pow((S - p1).magnitude(), 2) -
                                                                                    pow(R, 2)))) /
                                     pow(V.magnitude(), 2);
                            dt_i_2 = (V * (S - p1) - sqrt(pow((V * (S - p1)), 2) - pow(V.magnitude(), 2) *
                                                                                   (pow((S - p1).magnitude(), 2) -
                                                                                    pow(R, 2)))) /
                                     pow(V.magnitude(), 2);
                            if(fabs(dt_i_1) < fabs(dt) && dt_i_1 * dt > 0)
                            {
                                dt_i = dt_i_1;
                                if(fabs(dt_i) > fabs(max_dt_i))
                                {
                                    max_dt_i = dt_i;
                                    ti_t = t;
                                }
                                return true;
                            }
                            else if(fabs(dt_i_2) < fabs(dt) && dt_i_2 * dt > 0)
                            {
                                dt_i = dt_i_2;
                                if(fabs(dt_i) > fabs(max_dt_i))
                                {
                                    max_dt_i = dt_i;
                                    ti_t = t;
                                }
                                return true;
                            }
                        }
                        else if(pow((S - p2).magnitude(), 2) <= pow(R, 2) && pow((V * (S - p2)), 2) - pow(V.magnitude(), 2) * (pow((S - p2).magnitude(), 2) - pow(R, 2)) >= 0)
                        {
                            dt_i_1 = (V * (S - p2) + sqrt(pow((V * (S - p2)), 2) - pow(V.magnitude(), 2) *
                                                                                   (pow((S - p2).magnitude(), 2) -
                                                                                    pow(R, 2)))) /
                                     pow(V.magnitude(), 2);
                            dt_i_2 = (V * (S - p2) - sqrt(pow((V * (S - p2)), 2) - pow(V.magnitude(), 2) *
                                                                                   (pow((S - p2).magnitude(), 2) -
                                                                                    pow(R, 2)))) /
                                     pow(V.magnitude(), 2);
                            if(fabs(dt_i_1) < fabs(dt) && dt_i_1 * dt > 0)
                            {
                                dt_i = dt_i_1;
                                if(fabs(dt_i) > fabs(max_dt_i))
                                {
                                    max_dt_i = dt_i;
                                    ti_t = t;
                                }
                                return true;
                            }
                            else if(fabs(dt_i_2) < fabs(dt) && dt_i_2 * dt > 0)
                            {
                                dt_i = dt_i_2;
                                if(fabs(dt_i) > fabs(max_dt_i))
                                {
                                    max_dt_i = dt_i;
                                    ti_t = t;
                                }
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}

void SphereCollisionHandle(const double& d_ti, Triangle& t, SphereState SS, size_t i, float cs, float cr)
{
    pba::Vector np = t.getNp();

    pba::Vector V = SS->vel(i);

    pba::Vector _S = SS->pos(i);

    pba::Vector S_dash = SS->pos(i) - V * d_ti;
    SS->set_pos(i, S_dash);

    pba::Vector vr = cs * V - (cs + cr) * np * (np * V);
    SS->set_vel(i, vr);

    pba::Vector S = SS -> pos(i) + SS->vel(i) * d_ti;
    SS->set_pos(i, S);

    tttt = 0;
}

static void SphereCollisionOperation(const double& dt, SphereState SS, std::vector<Triangle> triangle_list, float cs, float cr)
{
    for(size_t i=0; i<SS->nb(); i++)
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
                collisionFlag = collisionFlag || SphereCollisionDetection(temp_dt, *iter, max_d_ti, ti_t, SS, i);
            }
            temp_dt = max_d_ti;
            max_d_ti = 0;
            if(collisionFlag == true) {
                SphereCollisionHandle(temp_dt, ti_t, SS, i, cs, cr);
            }
        }
    }
}

#endif //COURSE_SPHERETRIANGLECOLLISION_H
