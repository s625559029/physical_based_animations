//
// Created by ysun3 on 10/20/17.
//

#ifndef ASSIGNMENT2_RIGIDBODYPBATHING_H
#define ASSIGNMENT2_RIGIDBODYPBATHING_H

#ifdef __APPLE__
#include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
#else
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#endif

#include "../include/PbaThing.h"
#include "../include/RigidBodyStateData.h"
#include "../include/Triangle.h"
#include "../include/BaseForce.h"
#include "../include/Point.h"
#include "../include/Face.h"
#include "../include/ObjReader.h"
#include "../include/RigidBodySolver.h"
#include "../include/Gravity.h"
#include <iostream>

class RigidBodyPbaThing : public pba::PbaThingyDingy
{
public:
    RigidBodyPbaThing(const std::string nam = "RigidBodyPbaThing") :
    PbaThingyDingy(nam),
    g(0.98)
    {
        std::cout << name << "constructed" << std::endl;
    }
    ~RigidBodyPbaThing(){}

    void Init( const std::vector<std::string>& args );
    void solve() { RigidBodySolve(); }
    void Display();
    void Usage();
    void Keyboard(unsigned char key, int x, int y);
    void RigidBodySolve();

private:
    RigidBodyState RBS;

    std::vector<Triangle> triangle_list;
    std::vector<Triangle> triangle_list_obj;

    std::vector<BaseForce *> force_for_i;
    double g;

    void createCube();
};

pba::PbaThing RigidBodyThing() { return pba::PbaThing(new RigidBodyPbaThing()); }

void RigidBodyPbaThing::Init(const std::vector<std::string> &args)
{
    dt = 1.0 / 96.0;
    if(triangle_list.size() == 0)
    {
        createCube();
    }

    if(triangle_list_obj.size() == 0)
    {
        std::string route = "..//models//utah_teapot.obj";
        vector<Point> obj_points;
        vector<Face> obj_faces;
        ObjReader reader(route, obj_points, obj_faces);
        reader.read_Obj();

        RBS = CreateRigidBodyState("RigidBodyDynamicalData" + name);
        RBS -> add(obj_points.size());

        pba::Color temp_col;
        for( size_t i=0; i < obj_points.size();i++ )
        {
            temp_col = pba::Color(drand48(), drand48(), drand48(), 255);
            RBS -> set_pos(i, pba::Vector(obj_points[i].x / 100, obj_points[i].y / 100, obj_points[i].z / 100));
            RBS -> set_ci(i, temp_col);
        }
        Gravity* G = new Gravity;
        G -> set_g(g);
        for(size_t i = 0; i < RBS -> nb(); i++)
        {
            force_for_i.push_back(G);
        }
        RBS -> Init();
    }
}

void RigidBodyPbaThing::Display()
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

    glPointSize(3.6f);
    glBegin(GL_POINTS);
    for(size_t i=0; i<RBS->nb(); i++) {
        glColor3f(RBS->ci(i).X(), RBS->ci(i).Y(), RBS->ci(i).Z());
        glVertex3f(RBS->ver_pos(i).X(), RBS->ver_pos(i).Y(), RBS->ver_pos(i).Z());
    }
    glEnd();
}

void RigidBodyPbaThing::RigidBodySolve()
{
    RigidBodySolver::PositionSolverWithCollision(dt/2.0, RBS, triangle_list);
    RigidBodySolver::VelocitySolver(dt, RBS, force_for_i);
    RigidBodySolver::PositionSolverWithCollision(dt/2.0, RBS, triangle_list);
}

void RigidBodyPbaThing::createCube()
{
    // vertices
    pba::Vector verts[8];

    // face normals
    pba::Vector normals[6];

    // faces
    std::vector< std::vector<int> > faces;

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

void RigidBodyPbaThing::Keyboard(unsigned char key, int x, int y)
{
    if( key == 't' ){ dt /= 1.1; std::cout << "time step " << dt << std::endl; }
    if( key == 'T' ){ dt *= 1.1; std::cout << "time step " << dt << std::endl; }
    if( key == 'v' ){ RBS -> initial_velocity /= 1.1; std::cout << "Initial velocity " << RBS -> initial_velocity << std::endl; }
    if( key == 'V' ){ RBS -> initial_velocity *= 1.1; std::cout << "Initial velocity " << RBS -> initial_velocity << std::endl; }
    if( key == 'r'){ RBS -> Init();}
}

void RigidBodyPbaThing::Usage()
{
    std::cout << "=== RigidBodyPbaThing ===\n";
    std::cout << "t/T          reduce/increase animation time step\n";
    std::cout << "v/V          reduce/increase initial velocity.\n";
}

#endif //ASSIGNMENT2_RIGIDBODYPBATHING_H
