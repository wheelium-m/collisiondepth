#ifndef JOINT
#define JOINT
#include <btBulletDynamicsCommon.h>
#include <string>
#include <vector>

class Joint{
 public:
  std::string name;
  btTransform trans;
  /*Points represent the centers of spheres*/
  std::vector<btVector3> points;
  /*Radius of spheres*/
  float radius;
  Joint() 
    : name(""), trans(btTransform()), points(std::vector<btVector3>()), 
      radius(0.0f) {};
  Joint(const Joint& orig)
  : name(orig.name), trans(orig.trans), points(orig.points), 
    radius(orig.radius) {}
  
  Joint(std::string n, btTransform t, std::vector<btVector3> pts, float r)
    : name(n), trans(t), points(pts), radius(r) {};
  //~Joint();
};
#endif
