#include "Joint.h"
#include <btBulletDynamicsCommon.h>
#include <stdlib.h>

Joint::Joint(){
  trans = btTransform(btQuaternion::getIdentity(), btVector3(0.0,0.0,0.0));
  points = NULL;
  radius = 0.0;
  numPoints = 0;
}

Joint::Joint(btTransform tr, btVector3* pts,int num, float rad){
  trans=tr;
  points=pts;
  radius=rad;
  numPoints = num;
}

Joint::Joint(btQuaternion quat, btVector3 tr, btVector3* pts, int num, float rad){
  btTransform tra(quat, tr);
  trans = tra;
  points = pts;
  radius = rad;
  numPoints = num;
}

/*Joint::~Joint(){
  delete points;
  
}
*/
