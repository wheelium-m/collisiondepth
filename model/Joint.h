#ifndef JOINT
#define JOINT
#include <btBulletDynamicsCommon.h>
class Joint{
 public:
  btTransform trans;
  /*Points represent the centers of spheres*/
  btVector3 *points;
  int numPoints;
  /*Radius of spheres*/
  float radius;
  Joint();
  Joint(btTransform, btVector3*,int, float);
  Joint(btQuaternion, btVector3, btVector3*, int, float);
  //~Joint();
};
#endif
