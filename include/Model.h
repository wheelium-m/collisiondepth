#include <btBulletDynamicsCommon.h>

// Our model is, for now, just a vector of points.
typedef btAlignedObjectArray<btVector3> Model;

// Obtain a reference to our static model data.
const Model& model();
