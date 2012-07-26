#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H
#include "Model.h"
#include "DepthMap.h"
#include <btBulletDynamicsCommon.h>

class CollisionChecker {
private:
  std::vector<const ModelTree*> models;
  std::vector<const DepthMap*> depthMaps;
public:
  CollisionChecker(const ModelTree*);
  void addDepthMap(const DepthMap*);
  bool isColliding(const btTransform& camera,
                   const float sphereRadius, 
                   const std::map<std::string,float>&);
  void getCollisionInfo(const btTransform& camera,
                        const float sphereRadius,
                        const std::map<std::string,float>&, 
                        map<std::string,bool>&);
};
#endif
