//
// Created by ysun3 on 11/22/17.
//

#ifndef COURSE_SPHEREPBATHING_H
#define COURSE_SPHEREPBATHING_H

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
#include "../include/SphereStateData.h"
#include "../include/Triangle.h"
#include "../include/BaseForce.h"
#include "../include/ObjReader.h"
#include "../include/Gravity.h"
#include "../include/LeapfrogSolver.h"
#include <iostream>

class SpherePbaThing : public pba::PbaThingyDingy
{
public:
    SpherePbaThing(const std::string nam = "RigidBodyPbaThing") :
            PbaThingyDingy(nam),
            g(0.98),
            cs(0.8),
            cr(1.0)
    {
        std::cout << name << "constructed" << std::endl;
    }
    ~SpherePbaThing(){}

    void Init( const std::vector<std::string>& args );
    void solve() { SphereSolve(); }
    void Display();
    //void Usage();
    //void Keyboard(unsigned char key, int x, int y);
    void SphereSolve();

private:
    SphereState SS;

    std::vector<BaseForce *> forces;

    std::vector<Triangle> triangle_list_obj;

    std::vector<BaseForce *> force_for_i;
    double g;
    double cs;
    double cr;

};

pba::PbaThing SphereThing() { return pba::PbaThing(new SpherePbaThing()); }

void SpherePbaThing::Init(const std::vector<std::string> &args)
{
    dt = 1.0 / 96.0;

    if(triangle_list_obj.size() == 0)
    {
        std::string route = "..//models//well.obj";
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

    SS = CreateSphereState("SphereBodyDynamicalData" + name);
    SS -> add(1);
    SS -> set_pos(0, pba::Vector(0.6, 1, 0));
    SS -> radius.push_back(0.2);

    Gravity* G = new Gravity;
    G -> set_g(g);
    forces.push_back(G);
}

void SpherePbaThing::Display()
{
    glPolygonMode(GL_FRONT,GL_FILL);
    glPolygonMode(GL_BACK,GL_LINE);
    glBegin(GL_TRIANGLES);

    for(auto iter = triangle_list_obj.begin(); iter != triangle_list_obj.end(); iter++)
    {
        glVertex3d(iter->getV1().X(), iter->getV1().Y(), iter->getV1().Z());
        glNormal3d(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

        glVertex3d(iter->getV2().X(), iter->getV2().Y(), iter->getV2().Z());
        glNormal3d(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

        glVertex3d(iter->getV3().X(), iter->getV3().Y(), iter->getV3().Z());
        glNormal3d(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());
    }
    glEnd();

    for(size_t i = 0; i < SS -> nb(); i++)
    {
        double x = SS -> pos(i).X();
        double y = SS -> pos(i).Y();
        double z = SS -> pos(i).Z();

        glColor3d(1,0,0);

        glPushMatrix();
        glTranslated(x, y, z);
        glutSolidSphere(SS -> radius[i],50,50);
        glPopMatrix();
    }
    glColor3d(255, 255, 255);
}

void SpherePbaThing::SphereSolve()
{
    LeapfrogSolver::SphereLeapfrogSolveWithCollision(dt, SS, forces, triangle_list_obj, cs, cr);
}

#endif //COURSE_SPHEREPBATHING_H
