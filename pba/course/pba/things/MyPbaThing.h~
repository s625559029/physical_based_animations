//
// Created by ysun3 on 9/12/17.
//

#ifndef PBA_MYPBATHING_H
#define PBA_MYPBATHING_H

#include <iostream>
#include "PbaThing.h"
#include "DynamicalState.h"
#include "SixOrderSolver.h"
#include "Vector.h"
#include "Color.h"
#include "Gravity.h"
#include "../include/Gravity.h"
#include "ObjReader.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
#else
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#endif

class MyPbaThing : public pba::PbaThingyDingy {
public:
    MyPbaThing(const std::string nam = "BoidsMyPbaThing") : PbaThingyDingy(nam), obj_flag(true) {
        std::cout << name << "constructed" << std::endl;
    }

    ~MyPbaThing() {forces.clear();}

    pba::DynamicalState GetDynamicalState() const { return DS; }

    void Force(const Gravity* force);

    void Init( const std::vector<std::string>& args );

    void Display();

    void Keyboard(unsigned char key, int x, int y);

    void Usage();

    void Reset();

    void solve();

    void createCube();

    void solveWithForces(const std::vector<BaseForce *> forces);

    void solveWithCollision(const std::vector<BaseForce *> forces, std::vector<Triangle> triangle_list, float cs, float cr);

    void BoidsModifier();
private:
    //number of particles
    size_t nb;

    pba::DynamicalState DS;

    //flag of sovler, emit and geometry
    bool flag, emit, obj_flag;

    float g;

    std::vector<BaseForce *> forces;

    //geometries
    std::vector<Triangle> triangle_list;
    std::vector<Triangle> triangle_list_obj;

    //coefficient of stickness and restitution
    float cs,cr;

    // vertices
    pba::Vector verts[8];

    // face normals
    pba::Vector normals[6];

    // faces
    std::vector< std::vector<int> > faces;

    // face colors
    pba::Color face_colors[6];

    //constance for boids
    float ka;
    float kv;
    float kc;
    float R1;
    float R2;
    float theta1;
    float theta2;
    float max_a;

protected:
    void ttest();
};

pba::PbaThing BoidsThing() { return pba::PbaThing(new MyPbaThing()); }

void MyPbaThing::Init( const std::vector<std::string>& args )
{
    ka = 0.2;
    kv = 12;
    kc = 5;
    R1 = 0.3;
    R2 = 0.8;
    theta1 = 160;
    theta2 = 250;
    max_a = 10;

    if(triangle_list_obj.size() == 0)
    {
        std::string route = "..//models//bigsphere.obj";
        vector<Point> obj_points;
        vector<Face> obj_faces;
        ObjReader reader(route, obj_points, obj_faces);
        reader.read_Obj();
        for( size_t i=0;i<obj_faces.size();i++ )
        {
            Face face = obj_faces[i];
            pba::Vector v1(face.p1.x, face.p1.y, face.p1.z);
            pba::Vector v2(face.p2.x, face.p2.y, face.p2.z);
            pba::Vector v3(face.p3.x, face.p3.y, face.p3.z);
            Triangle t(v1, v2, v3);
            triangle_list_obj.push_back(t);
        }
    }
    if(triangle_list.size() == 0)
    {

        createCube();
    }

    flag = true;
    emit = false;

    g = 0;

    dt = 1.0 / 24.0;

    nb = 100;
    cs = 1;
    cr = 1;

    Gravity* G = new Gravity();
    G->set_g(g);
    if(forces.size() == 0) {
        forces.push_back(G);
    } else{
        forces.clear();
        forces.push_back(G);
    }

    if(DS)
    {
        DS.reset();
    }

    DS = pba::CreateDynamicalState("DynamicalData" + name);

    DS->add(nb);

    pba::Vector temp_pos;
    pba::Vector temp_vel;
    pba::Color temp_col;

    for(size_t i=0; i<DS->nb(); i++)
    {
        temp_pos = pba::Vector(drand48(), drand48(), drand48());
        temp_vel = pba::Vector((2 * drand48() - 1) / 5, (2 * drand48() - 1) / 5, (2 * drand48() - 1)) / 5;
        temp_col = pba::Color(drand48(), drand48(), drand48(), 255);

        DS->set_pos(i, temp_pos);
        DS->set_vel(i, temp_vel);
        DS->set_ci(i, temp_col);
    }
}

void MyPbaThing::solve()
{
    BoidsModifier();
    if(obj_flag)
    {
        solveWithCollision(forces, triangle_list, cs, cr);
    }
    else{
        solveWithCollision(forces, triangle_list_obj, cs, cr);
    }
}

void MyPbaThing::Display()
{
    if(emit == true)
    {
        DS->add(1);
        nb++;

        pba::Vector temp_pos;
        pba::Vector temp_vel;
        pba::Color temp_col;

        temp_pos = pba::Vector(drand48(), drand48(), drand48());
        temp_vel = pba::Vector((2*drand48()-1)/100, (2*drand48()-1)/100, (2*drand48()-1)/100);
        temp_col = pba::Color(drand48(), drand48(), drand48(), 255);

        DS->set_pos(nb-1, temp_pos);
        DS->set_vel(nb-1, temp_vel);
        DS->set_ci(nb-1, temp_col);
    }

    if(obj_flag)
    {
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        glBegin(GL_TRIANGLES);

        for(auto iter = triangle_list.begin(); iter != triangle_list.end(); iter++)
        {
            glVertex3f(iter->getV1().X(), iter->getV1().Y(), iter->getV1().Z());
            glNormal3f(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

            glVertex3f(iter->getV2().X(), iter->getV2().Y(), iter->getV2().Z());
            glNormal3f(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

            glVertex3f(iter->getV3().X(), iter->getV3().Y(), iter->getV3().Z());
            glNormal3f(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());
        }
        glEnd();
    }
    else
    {
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        glBegin(GL_TRIANGLES);

        for(auto iter = triangle_list_obj.begin(); iter != triangle_list_obj.end(); iter++)
        {
            glVertex3f(iter->getV1().X(), iter->getV1().Y(), iter->getV1().Z());
            glNormal3f(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

            glVertex3f(iter->getV2().X(), iter->getV2().Y(), iter->getV2().Z());
            glNormal3f(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

            glVertex3f(iter->getV3().X(), iter->getV3().Y(), iter->getV3().Z());
            glNormal3f(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());
        }
        glEnd();
    }



    glPointSize(3.6f);
    glBegin(GL_POINTS);
    for(size_t i=0; i<DS->nb(); i++) {
        glColor3f(DS->ci(i).X(), DS->ci(i).Y(), DS->ci(i).Z());
        glVertex3f(DS->pos(i).X(), DS->pos(i).Y(), DS->pos(i).Z());
    }

    glEnd();
}

void MyPbaThing::Keyboard(unsigned char key, int x, int y)
{
    if( key == 't' ){ dt /= 1.1; std::cout << "time step " << dt << std::endl; }
    if( key == 'T' ){ dt *= 1.1; std::cout << "time step " << dt << std::endl; }
    //if( key == 'r'){Reset(); std::cout << "Reset." << std::endl;}
    if( key == 'o') {
        if (obj_flag == true) {
            obj_flag = false;
            std::cout << "Big sphere." << std::endl;
        } else{
            obj_flag = true;
            std::cout << "Cube." << std::endl;
        }
    }

    if(key == 'a') {ka /= 1.1; std::cout << "collision avoidance strength " << ka << std::endl;}
    if(key == 'A') {ka *= 1.1; std::cout << "collision avoidance strength " << ka << std::endl;}
    if(key == 'v') {kv /= 1.1; std::cout << "velocity matching strength " << kv << std::endl;}
    if(key == 'V') {kv *= 1.1; std::cout << "velocity matching strength " << kv << std::endl;}
    if(key == 'c') {kc /= 1.1; std::cout << "centering strength " << kc << std::endl;}
    if(key == 'C') {kc *= 1.1; std::cout << "centering strength " << kc << std::endl;}
    if(key == 'm') {max_a /= 1.1; std::cout << "maximum acceleration threshold " << max_a << std::endl;}
    if(key == 'M') {max_a *= 1.1; std::cout << "maximum acceleration threshold " << max_a << std::endl;}
    if(key == 'd') {R1 /= 1.1; std::cout << "range R1 " << R1 << std::endl;}
    if(key == 'D') {if((R1 * 1.1) <= R2)R1 *= 1.1; else R1 = R2; std::cout << "range R1 " << R1 << std::endl;}
    if(key == 'y') {if((R2 / 1.1) >= R1)R2 /= 1.1; else R2 = R1; std::cout << "range R2 " << R2 << std::endl;}
    if(key == 'Y') {R2 *= 1.1; std::cout << "range R2 " << R2 << std::endl;}
    if(key == 'q') {theta1 /= 1.1;  std::cout << "field of view " << theta1 << std::endl;}
    if(key == 'Q') {if(theta1 < theta2) theta1 *= 1.1; std::cout << "field of view " << theta1 << std::endl;}
}

void MyPbaThing::Usage()
{
    std::cout << "=== BoidsPbaThing ===\n";
    std::cout << "t/T          reduce/increase animation time step\n";
    std::cout << "O            switch model with cube / big sphere\n";
    std::cout << "a/A          reduce/increase collision avoidance strength\n";
    std::cout << "v/V          reduce/increase velocity matching strength\n";
    std::cout << "c/C          reduce/increase centering strength\n";
    std::cout << "m/M          reduce/increase maximum acceleration threshold\n";
    std::cout << "d/D          reduce/increase range R1\n";
    std::cout << "y/Y          reduce/increase range R2\n";
    std::cout << "q/Q          reduce/increase field of view\n";
}

void MyPbaThing::Reset()
{
    std::vector<std::string> nv;
    Init(nv);
}

void MyPbaThing::Force(const Gravity* force)
{
    for(size_t i=0; i<DS->nb(); i++)
    {
        DS->set_accel(i, (force->calForce(DS, i)/ DS->mass(i)));
    }
}

void MyPbaThing::solveWithForces(const std::vector<BaseForce *> forces)
{
    if(flag)   //Using LeapFrog
    {
        LeapfrogSolver::LeapfrogSolve(dt, DS, forces);
    } else  //Using SixOrder
    {
        SixOrderSolver::SixOrderSolve(dt, DS, forces);
    }
}

void MyPbaThing::solveWithCollision(const std::vector<BaseForce *> forces, std::vector<Triangle> triangle_list, float cs, float cr)
{
    if(flag)   //Using LeapFrog
    {
        LeapfrogSolver::LeapfrogSolveWithCollision(dt, DS, forces, triangle_list, cs, cr);
    } else  //Using SixOrder
    {
        SixOrderSolver::SixOrderSolveWithCollision(dt, DS, forces, triangle_list, cs, cr);
    }
}

void MyPbaThing::createCube()
{
    verts[0] = pba::Vector(-2,-2,-2);
    verts[1] = pba::Vector(2,-2,-2);
    verts[2] = pba::Vector(2,2,-2);
    verts[3] = pba::Vector(-2,2,-2);
    verts[4] = pba::Vector(-2,-2,2);
    verts[5] = pba::Vector(2,-2,2);
    verts[6] = pba::Vector(2,2,2);
    verts[7] = pba::Vector(-2,2,2);

    normals[0] = pba::Vector(1,0,0);
    normals[1] = pba::Vector(0,1,0);
    normals[2] = pba::Vector(0,0,1);
    normals[3] = pba::Vector(-1,0,0);
    normals[4] = pba::Vector(0,-1,0);
    normals[5] = pba::Vector(0,0,-1);

    face_colors[0] = pba::Color(1,0,1,0);
    face_colors[1] = pba::Color(1,0,0,0);
    face_colors[2] = pba::Color(0,0,1,0);
    face_colors[3] = pba::Color(0,1,0,0);
    face_colors[4] = pba::Color(1,1,0,0);
    face_colors[5] = pba::Color(0,1,1,0);

    std::vector<int> face;
    face.push_back(1);
    face.push_back(2);
    face.push_back(6);
    face.push_back(5);
    faces.push_back(face);

    face[0] = 2;
    face[1] = 3;
    face[2] = 7;
    face[3] = 6;
    faces.push_back(face);

    face[0] = 0;
    face[1] = 3;
    face[2] = 2;
    face[3] = 1;
    faces.push_back(face);

    face[0] = 0;
    face[1] = 4;
    face[2] = 7;
    face[3] = 3;
    faces.push_back(face);

    face[0] = 0;
    face[1] = 1;
    face[2] = 5;
    face[3] = 4;
    faces.push_back(face);

    face[0] = 5;
    face[1] = 6;
    face[2] = 7;
    face[3] = 4;
    faces.push_back(face);

    for( size_t i=0;i<faces.size();i++ )
    {
        pba::Vector v1, v2, v3, v4;
        std::vector<int>& face = faces[i];
        v1 = verts[face[0]];
        v2 = verts[face[1]];
        v3 = verts[face[2]];
        v4 = verts[face[3]];
        Triangle t1(v1, v2, v3);
        triangle_list.push_back(t1);
        Triangle t2(v3, v4, v1);
        triangle_list.push_back(t2);
    }
}

void MyPbaThing::BoidsModifier()
{
    for(size_t i=0; i<DS->nb(); i++)
    {
        pba::Vector aa, av, ac, dij;

        for(size_t j=0; j<DS->nb(); j++)
        {
            if(j != i)
            {
                dij = DS->pos(j) - DS->pos(i);

                float kr = 0;
                float ktheta = 0;
                float cos_theta_ij;

                //calculate kr for Range Weight
                if(dij.magnitude() <= R1) kr = 1;
                else if(dij.magnitude() >= R2) kr = 0;
                else kr = (R2 - dij.magnitude()) / (R2 - R1);

                //calculate ktheta for Vision
                cos_theta_ij = (DS->vel(i) * dij) / (DS->vel(i).magnitude() * dij.magnitude());

                if(cos_theta_ij >= cos(theta1 / 180 * 3.14 / 2)) ktheta = 1;
                else if(cos_theta_ij <= cos(theta2 / 180 * 3.14 / 2)) ktheta = 0;
                else ktheta = (cos(theta2 / 180 * 3.14 /2) - cos_theta_ij) / (cos(theta2 / 180 * 3.14 / 2) - cos(theta1 / 180 * 3.14 / 2));

                aa += -ka * dij / pow(dij.magnitude(), 2) * kr * ktheta;
                av += kv * (DS->vel(j) - DS->vel(i)) * kr * ktheta;
                ac += kc * dij * kr * ktheta;
            }
        }
        float residual;
        if(aa.magnitude() > max_a)
        {
            aa = aa * (max_a / aa.magnitude());
            av = pba::Vector(0, 0, 0);
            ac = pba::Vector(0, 0, 0);
        }
        else
        {
            residual = max_a - aa.magnitude();
            if(av.magnitude() > residual)
            {
                av = av * (residual / av.magnitude());
                ac = pba::Vector(0, 0, 0);
            }
            else
            {
                residual = residual - av.magnitude();
                if(ac.magnitude() > residual)
                {
                    ac = ac * (residual / ac.magnitude());
                }
            }
        }
        pba::Vector a = aa + av + ac;
        DS->set_accel(i, a);
    }
}

#endif //PBA_MYPBATHING_H
