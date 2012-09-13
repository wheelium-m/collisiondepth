#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H
#include "Model.h"
#include "DepthMap.h"
#include "body_pose.h"
#include <btBulletDynamicsCommon.h>

class CollisionChecker {
private:
  std::vector<const ModelTree*> models;
  std::vector<const DepthMap*> depthMaps;
  int numSpheres;
  int numJoints;
  std::vector<int> langleRemap;
  std::vector<int> rangleRemap;
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

  // SBPL-compatibility API
  bool checkCollision(std::vector<double> &langles, 
                      std::vector<double> &rangles, 
                      BodyPose &pose, 
                      bool verbose, 
                      unsigned char &dist, 
                      int &debug_code);
  void getCollisionSpheres(std::vector<double> &langles,
                           std::vector<double> &rangles,
                           BodyPose &pose,
                           std::string group_name,
                           std::vector<std::vector<double> > &spheres);
  void levineInit();
  vector<float> getDepthPoses();
};
#endif
