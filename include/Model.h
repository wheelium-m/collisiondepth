#ifndef MODEL_H
#define MODEL_H

#include <btBulletDynamicsCommon.h>
#include "Joint.h"
#include "TreeNode.h"
#include <map>
  
// Our model is, for now, just a vector of points.
typedef btAlignedObjectArray<btVector3> Model;
typedef TreeNode<Joint> ModelTree;

// A sphere in the camera's coordinate frame
struct CameraSphere {
  btVector3 center;
  float r;
  string jointName;
  CameraSphere(btVector3 c, float r, string n) : center(c), r(r), jointName(n) {}
};

// Obtain a reference to our static model data.
const Model& model();
const ModelTree& pr2();
const ModelTree& testTree();
ModelTree* poseModel(const ModelTree&, const std::map<std::string,float>&);
void freePosedModel(ModelTree*);
void painterSort(const ModelTree& root,
                 const btTransform& camera,
                 std::vector<CameraSphere>& v);
void transformSpheres(const ModelTree& root, 
                      const btTransform& camera, 
                      vector<CameraSphere>& v);

#endif
