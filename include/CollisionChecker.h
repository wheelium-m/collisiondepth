#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H
#include "Model.h"
#include "DepthMap.h"
#include <btBulletDynamicsCommon.h>

class CollisionChecker {
private:
  std::vector<const ModelTree*> models;
  std::vector<const DepthMap*> depthMaps;
  int numSpheres;
public:
  CollisionChecker(const ModelTree*);
  void addDepthMap(const DepthMap*);
  const DepthMap* getDepthMap(int i);
  const DepthMap* getActiveDepthMap();
  int numDepthMaps() const;
  void makeJointVector(const std::map<std::string,float>&,
                       std::vector<float>&);
  void makeCollisionMap(const std::vector<bool>&,
                        std::map<std::string,bool>&);
  bool isColliding(const btTransform& camera,
                   const float sphereRadius, 
                   const std::vector<float>&);
  void getCollisionInfo(const btTransform& camera,
                        const float sphereRadius,
                        const std::vector<float>&, 
                        std::vector<bool>&);
  void getCollisionInfoReference(const btTransform& camera,
                                 const float sphereRadius,
                                 const std::map<std::string,float>&, 
                                 std::map<std::string,bool>&);
};
#endif
