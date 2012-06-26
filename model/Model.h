#ifndef MODEL
#define MODEL
#include <btBulletDynamicsCommon.h>
#include "Joint.h"
class Model{
 public:  
  Joint *joints;
  int numJoints;
  Model();
  Model(Joint *, int);
  Model(const Model&);
  
  /*Internal transform modifies the angles of the Model object*/
  void internalTransform(btQuaternion*, int);

  /*External transform does not change the angles of the model object, but instean returns a new object with modified angles*/
  Model externalTransform(btQuaternion*,int);
  //~Model();
}; 
#endif
