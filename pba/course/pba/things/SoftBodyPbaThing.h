//
// Created by ysun3 on 10/31/17.
//

#ifndef ASSIGNMENT2_SOFTBODYPBATHING_H_H
#define ASSIGNMENT2_SOFTBODYPBATHING_H_H

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
#include "../include/SoftBodyStateData.h"
#include "../include/LeapfrogSolver.h"
#include "../include/SixOrderSolver.h"
#include "../include/ObjReader.h"
#include <iostream>

class SoftBodyPbaThing : public pba::PbaThingyDingy
{
public:
    SoftBodyPbaThing(const std::string nam = "SoftBodyPbaThing") :
            PbaThingyDingy(nam),
            nb(400),
            g(0.98),
            cs(1.0),
            cr(0.1),
            solver_flag(true)
    {
        std::cout << name << "constructed" << std::endl;
    }
    ~SoftBodyPbaThing(){}

    void Init( const std::vector<std::string>& args );
    void InitSoftBodyWithObj();
    void solve(){ SoftBodySolve(); }
    void Display();
    void Usage();
    void Reset();
    void Keyboard(unsigned char key, int x, int y);
    void SoftBodySolve();
    void CreateHole();
    void CreateCube();

private:
    SoftBodyState SBS;

    std::vector<BaseForce *> forces;

    std::vector<Triangle> triangle_list;
    std::vector<Triangle> triangle_list_obj;

    size_t nb;
    double g;
    double cs;
    double cr;

    bool solver_flag;
};

pba::PbaThing SoftBodyThing() { return pba::PbaThing(new SoftBodyPbaThing()); }

void SoftBodyPbaThing::Init(const std::vector<std::string> &args)
{
    dt = 1.0 / 32.0;

    size_t idx = 0;

    SBS = CreateSoftBodyState("SoftBodyDynamicalData" + name);
    SBS -> add(nb);
    for(int i = 0; i < sqrt(nb); i++)
    {
        for(int j = 0; j < sqrt(nb); j++)
        {
            double x = i;
            double y = j;
            SBS -> set_pos(idx, pba::Vector((x - sqrt(nb)/2) / 4.5, 3, (y - sqrt(nb)/2) / 4.5));
            idx++;
        }
    }
    SBS->InitPairs();
    //InitSoftBodyWithObj();

    Gravity* G = new Gravity;
    G -> set_g(g);
    forces.push_back(G);

    if(triangle_list.size() == 0)
    {
        CreateHole();
        //CreateCube();
    }
}

void SoftBodyPbaThing::Reset()
{
    size_t idx = 0;
    for(int i = 0; i < sqrt(nb); i++)
    {
        for(int j = 0; j < sqrt(nb); j++)
        {
            double x = i;
            double y = j;
            SBS -> set_pos(idx, pba::Vector((x - sqrt(nb)/2) / 4.5, 3, (y - sqrt(nb)/2) / 4.5));
            SBS -> set_vel(idx, pba::Vector(0, 0, 0));
            SBS -> set_accel(idx, pba::Vector(0, 0, 0));
            idx++;
        }
    }
}

void SoftBodyPbaThing::InitSoftBodyWithObj()
{
    std::string route = "..//models//utah_teapot.obj";
    vector<Point> obj_points;
    vector<Face> obj_faces;
    ObjReader reader(route, obj_points, obj_faces);
    reader.read_Obj();

    SBS = CreateSoftBodyState("SoftBodyDynamicalData" + name);
    SBS -> add(obj_points.size());

    pba::Color temp_col;
    for( size_t i=0; i < obj_points.size();i++ )
    {
        temp_col = pba::Color(drand48(), drand48(), drand48(), 255);
        SBS -> set_pos(i, pba::Vector(obj_points[i].x / 100, obj_points[i].y / 100, obj_points[i].z / 100));
        SBS -> set_ci(i, temp_col);
    }
}

void SoftBodyPbaThing::Display()
{
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glColor3f(0.5, 0.0, 0.0);
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

    glLineWidth(1.5);
    glColor3f(0.7, 0.7, 0.7);
    glBegin(GL_LINES);
    for(auto p = SBS -> connected_pairs.begin(); p != SBS -> connected_pairs.end(); p++)
    {
        size_t i = p -> i;
        size_t j = p -> j;
        pba::Vector p1 = SBS -> pos(i);
        pba::Vector p2 = SBS -> pos(j);
        glVertex3d(p1.X(), p1.Y(), p1.Z());
        glVertex3d(p2.X(), p2.Y(), p2.Z());
    }
    glEnd();
}

void SoftBodyPbaThing::SoftBodySolve()
{
    SBS -> ComputeAccels();
    if(solver_flag) {
        LeapfrogSolver::LeapfrogSolveWithCollision(dt, SBS, forces, triangle_list, cs, cr);
    }
    else {
        SixOrderSolver::SixOrderSolveWithCollision(dt, SBS, forces, triangle_list, cs, cr);
    }
}

void SoftBodyPbaThing::Keyboard(unsigned char key, int x, int y)
{
    if( key == 't' ){ dt /= 1.1; std::cout << "time step " << dt << std::endl; }
    if( key == 'T' ){ dt *= 1.1; std::cout << "time step " << dt << std::endl; }
    if( key == 's' ){ cs /= 1.1; std::cout << "coefficient of stickiness " << cs << std::endl; }
    if( key == 'S' ){ cs *= 1.1; std::cout << "coefficient of stickiness " << cs << std::endl; }
    if( key == 'x' ){ cr /= 1.1; std::cout << "coefficient of restitution " << cr << std::endl; }
    if( key == 'X' ){ cr *= 1.1; std::cout << "coefficient of restitution " << cr << std::endl; }
    if( key == 'd' ){ SBS -> ks /= 1.1; std::cout << "coefficient of spring " << SBS -> ks << std::endl; }
    if( key == 'D' ){ SBS -> ks *= 1.1; std::cout << "coefficient of spring " << SBS -> ks << std::endl; }
    if( key == 'y' ){ SBS -> kf /= 1.1; std::cout << "coefficient of friction " << SBS -> kf << std::endl; }
    if( key == 'Y' ){ SBS -> kf *= 1.1; std::cout << "coefficient of friction " << SBS -> kf << std::endl; }
    if( key == 'g' ){ g /= 1.1; std::cout << "gravity " << g << std::endl; }
    if( key == 'G' ){ g *= 1.1; std::cout << "gravity " << g << std::endl; }
    if( key == 'o' ){ solver_flag = true; std::cout << "leap frog solver " << std::endl; }
    if( key == 'O' ){ solver_flag = false; std::cout << "six order solver " << std::endl; }
}

void SoftBodyPbaThing::Usage()
{
    std::cout << "=== SoftBodyPbaThing ===\n";
    std::cout << "t/T          reduce/increase animation time step\n";
    std::cout << "s/S          reduce/increase coefficient of stickiness\n";
    std::cout << "x/X          reduce/increase coefficient of restitution.\n";
    std::cout << "d/D          reduce/increase coefficient of spring\n";
    std::cout << "y/Y          reduce/increase coefficient of friction\n";
    std::cout << "g/G          reduce/increase g of gravity\n";
    std::cout << "o/O          switch solver with leap frog and six order\n";
}


void SoftBodyPbaThing::CreateHole()
{
        std::string route = "..//models//PlaneWithHole.obj";
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
            triangle_list.push_back(t);
        }
}

void SoftBodyPbaThing::CreateCube()
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

#endif //ASSIGNMENT2_SOFTBODYPBATHING_H_H
