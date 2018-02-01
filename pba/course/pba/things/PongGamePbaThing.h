//
// Created by ysun3 on 12/3/17.
//

#ifndef COURSE_PONGGAMEPBATHING_H
#define COURSE_PONGGAMEPBATHING_H

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

class PongGamePbaThing : public pba::PbaThingyDingy
{
public:
    PongGamePbaThing(const std::string nam = "PongGamePbaThing") :
            PbaThingyDingy(nam),
            g(0.0),
            cs(1.0),
            cr(1.0)
    {
        std::cout << name << "constructed" << std::endl;
    }
    ~PongGamePbaThing(){}

    void Init( const std::vector<std::string>& args );
    void solve() { SphereSolve(); }
    void Display();
    void Usage();
    void Keyboard(unsigned char key, int x, int y);
    void SphereSolve();

    void moveBarLeft(std::vector<Triangle>& bar);
    void moveBarRight(std::vector<Triangle>& bar);
    void updateTriangles();

private:
    SphereState SS;

    std::vector<BaseForce *> forces;

    std::vector<Triangle> triangle_list_obj;

    std::vector<Triangle> cage;
    std::vector<Triangle> bar_1;
    std::vector<Triangle> bar_2;
    std::vector<Triangle> bricks;

    std::vector<pba::Color> colors;

    std::vector<BaseForce *> force_for_i;
    double g;
    double cs;
    double cr;

    int bar1Count = 0;
    int bar2Count = 0;
    int moveableTimes = 11;

    int bar1Score = 0;
    int bar2Score = 0;
};

pba::PbaThing PongGameThing() { return pba::PbaThing(new PongGamePbaThing()); }

void PongGamePbaThing::Init(const std::vector<std::string> &args)
{
    dt = 1.0 / 115.0;

    if(bricks.size() == 0)
    {
        std::string route = "..//models//bricks.obj";
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
            bricks.push_back(t);
        }
    }

    for(auto iter = bricks.begin(); iter != bricks.end(); iter++)
    {
        pba::Color c(drand48(), drand48(), drand48(), 1);
        colors.push_back(c);
    }

    if(cage.size() == 0)
    {
        std::string route = "..//models//cage.obj";
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
            cage.push_back(t);
        }
    }

    for(auto iter = cage.begin(); iter != cage.end(); iter++)
    {
        pba::Color c(drand48(), drand48(), drand48(), 1);
        colors.push_back(c);
    }

    if(bar_1.size() == 0)
    {
        std::string route = "..//models//bar1.obj";
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
            bar_1.push_back(t);
        }
    }

    for(auto iter = bar_1.begin(); iter != bar_1.end(); iter++)
    {
        pba::Vector v1 = (iter->getV1()) + pba::Vector(0,-4,0);
        iter -> setV1(v1);
        pba::Vector v2 = (iter->getV2()) + pba::Vector(0,-4,0);
        iter -> setV2(v2);
        pba::Vector v3 = (iter->getV3()) + pba::Vector(0,-4,0);
        iter -> setV3(v3);

        pba::Color c(drand48(), drand48(), drand48(), 1);
        colors.push_back(c);
    }

    if(bar_2.size() == 0)
    {
        std::string route = "..//models//bar2.obj";
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
            bar_2.push_back(t);
        }
    }

    for(auto iter = bar_2.begin(); iter != bar_2.end(); iter++)
    {
        pba::Vector v1 = (iter->getV1()) + pba::Vector(0,4,0);
        iter -> setV1(v1);
        pba::Vector v2 = (iter->getV2()) + pba::Vector(0,4,0);
        iter -> setV2(v2);
        pba::Vector v3 = (iter->getV3()) + pba::Vector(0,4,0);
        iter -> setV3(v3);

        pba::Color c(drand48(), drand48(), drand48(), 1);
        colors.push_back(c);
    }

    SS = CreateSphereState("SphereBodyDynamicalData" + name);
    SS -> add(1);
    SS -> set_pos(0, pba::Vector(0, 0, 0));
    SS -> radius.push_back(0.2);
    SS -> set_vel(0, pba::Vector(0, -2.5, 0));

    Gravity* G = new Gravity;
    G -> set_g(g);
    forces.push_back(G);
}

void PongGamePbaThing::Display()
{
    glPolygonMode(GL_FRONT,GL_FILL);
    glPolygonMode(GL_BACK,GL_FILL);
    glBegin(GL_TRIANGLES);

    updateTriangles();

    int i = 0;

    for(auto iter = triangle_list_obj.begin(); iter != triangle_list_obj.end(); iter++)
    {
        glColor3d(colors[i].X(), colors[i].Y(), colors[i].Z());

        glVertex3d(iter->getV1().X(), iter->getV1().Y(), iter->getV1().Z());
        glNormal3d(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

        glVertex3d(iter->getV2().X(), iter->getV2().Y(), iter->getV2().Z());
        glNormal3d(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

        glVertex3d(iter->getV3().X(), iter->getV3().Y(), iter->getV3().Z());
        glNormal3d(iter->getNp().X(), iter->getNp().Y(), iter->getNp().Z());

        i++;
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

        if(y < bar_1[0].getV1().Y() - 3)
        {
            SS -> set_pos(0, pba::Vector(0, 0, 0));
            SS -> set_vel(0, pba::Vector(0, -2.5, 0));
            bar2Score++;
            std::cout << "Top " << bar2Score << "       " << bar1Score << " Bottom" << std::endl;
        }
        if(y > bar_2[0].getV1().Y() + 3)
        {
            SS -> set_pos(0, pba::Vector(0, 0, 0));
            SS -> set_vel(0, pba::Vector(0, 2.5, 0));
            bar1Score++;
            std::cout << "Top " << bar2Score << "       " << bar1Score << " Bottom" << std::endl;
        }
    }
    glColor3d(255, 255, 255);
}

void PongGamePbaThing::Usage()
{
    std::cout << "=== Pong Game! ===" << std::endl;
    std::cout << "Use A & D to control the bottom bar." << std::endl;
    std::cout << "Use J & L to control the top bar." << std::endl;
}

void PongGamePbaThing::Keyboard(unsigned char key, int x, int y)
{
    if (key == 'a' || key == 'A') {
        if(bar1Count-1 > -moveableTimes) {
            moveBarLeft(bar_1); bar1Count--;
        }
    }
    if (key == 'd' || key == 'D') {
        if(bar1Count+1 < moveableTimes) {
            moveBarRight(bar_1);
            bar1Count++;
        }
    }
    if (key == 'j' || key == 'J') {
        if(bar2Count-1 > -moveableTimes) {
            moveBarLeft(bar_2);
            bar2Count--;
        }
    }
    if (key == 'l' || key == 'L') {
        if(bar2Count+1 < moveableTimes) {
            moveBarRight(bar_2);
            bar2Count++;
        }
    }
}

void PongGamePbaThing::SphereSolve()
{
    LeapfrogSolver::SphereLeapfrogSolveWithCollision(dt, SS, forces, triangle_list_obj, cs, cr);
}

void PongGamePbaThing::moveBarLeft(std::vector<Triangle> &bar)
{
    //Range: [-3, 3] on X
    for(auto iter = bar.begin(); iter != bar.end(); iter++)
    {
        pba::Vector v1 = (iter->getV1()) + pba::Vector(0.3,0,0);
        iter -> setV1(v1);
        pba::Vector v2 = (iter->getV2()) + pba::Vector(0.3,0,0);
        iter -> setV2(v2);
        pba::Vector v3 = (iter->getV3()) + pba::Vector(0.3,0,0);
        iter -> setV3(v3);
    }
}

void PongGamePbaThing::moveBarRight(std::vector<Triangle> &bar)
{
    for(auto iter = bar.begin(); iter != bar.end(); iter++)
    {
        pba::Vector v1 = (iter->getV1()) + pba::Vector(-0.3,0,0);
        iter -> setV1(v1);
        pba::Vector v2 = (iter->getV2()) + pba::Vector(-0.3,0,0);
        iter -> setV2(v2);
        pba::Vector v3 = (iter->getV3()) + pba::Vector(-0.3,0,0);
        iter -> setV3(v3);
    }
}

void PongGamePbaThing::updateTriangles()
{
    triangle_list_obj.clear();
    for(auto iter = bricks.begin(); iter != bricks.end(); iter++)
    {
        triangle_list_obj.push_back(*iter);
    }
    for(auto iter = cage.begin(); iter != cage.end(); iter++)
    {
        triangle_list_obj.push_back(*iter);
    }
    for(auto iter = bar_1.begin(); iter != bar_1.end(); iter++)
    {
        triangle_list_obj.push_back(*iter);
    }
    for(auto iter = bar_2.begin(); iter != bar_2.end(); iter++)
    {
        triangle_list_obj.push_back(*iter);
    }
}

#endif //COURSE_PONGGAMEPBATHING_H
