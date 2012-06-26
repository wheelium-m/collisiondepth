#ifndef JOINT
#define JOINT
#include <btBulletDynamicsCommon.h>
class Joint{
 public:
  btTransform trans;
  btVector3 *points;
  int numPoints;
  float radius;
  Joint();
  Joint(btTransform, btVector3*,int, float);
  Joint(btQuaternion, btVector3, btVector3*, int, float);
  //~Joint();
};
#endif
