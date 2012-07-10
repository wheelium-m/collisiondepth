#include <btBulletDynamicsCommon.h>
#include "Joint.h"
#include "TreeNode.h"
#include <map>
  
// Our model is, for now, just a vector of points.
typedef btAlignedObjectArray<btVector3> Model;
typedef TreeNode<Joint> ModelTree;

// Obtain a reference to our static model data.
const Model& model();
const ModelTree& pr2();
const ModelTree& testTree();
ModelTree* poseModel(const ModelTree&, const std::map<std::string,float>&);
void freePosedModel(ModelTree*);
