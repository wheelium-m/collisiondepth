#include "Model.h"

// Private initialization only to be called by the model function
// below.
Model* initModel() {
  Model* points = new btAlignedObjectArray<btVector3>;
  points->push_back(btVector3(8,0,0));
  points->push_back(btVector3(-8,0,0));
  return points;
}

// Our model data is a singleton object in the "Construct on First Use
// Idiom."
const Model& model() {
  static Model* points = initModel();
  return *points;
}
