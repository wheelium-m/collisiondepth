#ifndef JOINT
#define JOINT
#include <btBulletDynamicsCommon.h>
#include <string>
#include <iostream>
#include <vector>
#include "StlFile.h"

#define SPHERE_RADIUS 0.2
#define MODEL_SCALE 0.5

class Joint{
 public:
  std::string name;
  btTransform trans;
  btVector3 axis;
  /*Points represent the centers of spheres*/
  std::vector<btVector3> points;
  /*Radius of spheres*/
  float radius;
  StlFile *mesh;
  Joint() 
    : name(""), trans(btTransform()), axis(btVector3()), 
    points(std::vector<btVector3>()), radius(0.0f), mesh(NULL) {};
  Joint(const Joint& orig)
    : name(orig.name), trans(orig.trans), axis(orig.axis),
      points(orig.points), radius(orig.radius), mesh(NULL) {
    if(orig.mesh) this->mesh = new StlFile(orig.mesh);
  }

  Joint(std::string n, btTransform t, std::vector<btVector3> pts,
        btVector3 a, float r)
    : name(n), trans(t), axis(a), points(pts), radius(r), mesh(NULL) {};

  Joint(std::string n, btTransform t, btVector3 a, float r)
    : name(n), trans(t), axis(a), radius(r), mesh(NULL) {};

  Joint(std::string n, btTransform t, std::vector<btVector3> pts, 
        btVector3 a, float r, const char * filename)
    : name(n), trans(t), axis(a), points(pts), radius(r),mesh(new StlFile(filename)) 
    {
      if(strcmp(filename, "")!=0){
        for(int i = 0; i < mesh->numTriangles; i++){
          btVector3 p1(mesh->triangles[i].pts[0].xyz[0],
                       mesh->triangles[i].pts[0].xyz[1],
                       mesh->triangles[i].pts[0].xyz[2]);
          btVector3 p2(mesh->triangles[i].pts[1].xyz[0],
                       mesh->triangles[i].pts[1].xyz[1],
                       mesh->triangles[i].pts[1].xyz[2]);
          btVector3 p3(mesh->triangles[i].pts[2].xyz[0],
                       mesh->triangles[i].pts[2].xyz[1],
                       mesh->triangles[i].pts[2].xyz[2]);
          btVector3 center = (p1+p2+p3)/3;
          /* Somewhat fixes scaling issue */
          center *= r;
          points.push_back(center);	
      }
      removeOverlaps();
      //std::cout<<filename<<endl;
      //std::cout<<"The number of points in this joint is "<<1+points.size()<<" and mesh->numTriangles is "<<mesh->numTriangles<<" and filename is "<<filename<<std::endl;
      }
    };
  ~Joint() {
    if(mesh) delete mesh;
  }
 private:
  void removeOverlaps(){
    std::vector<btVector3> temp;
    bool noOverlap = true;
    for(int i = 0; i < points.size()-1; i++){
      for(int j = i+1; j<points.size() && noOverlap; j++){
        //if((points[i]-points[j]).length()<0.02)
        if((points[i]-points[j]).length()<0.04)
	  noOverlap=false;
      }
      if(noOverlap){
	temp.push_back(points[i]);
      }
      noOverlap=true;
    }
    points=temp;
  };
};
#endif
