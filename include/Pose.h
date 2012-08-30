#ifndef POSE_H
#define POSE_H
#include <btBulletDynamicsCommon.h>

struct Pose {
  float distFromStart;
  float costToGoal;
  int danceFrame;
  btTransform poseT;
  int poseID;
  int deltaYaw;
  std::vector<btTransform> pathTaken;
  Pose(float d, float c, int f, btTransform p, int dy, 
       std::vector<btTransform> pt) :
    distFromStart(d), costToGoal(c), danceFrame(f), poseT(p),  
    deltaYaw(dy), pathTaken(pt.begin(), pt.end()) { 
    pathTaken.push_back(p);
  }

  Pose(float d, float c, int f, btTransform p, int dy)
  : distFromStart(d), costToGoal(c), danceFrame(f), poseT(p),  
    deltaYaw(dy) {}

  Pose(const Pose& other)
  : distFromStart(other.distFromStart), costToGoal(other.costToGoal),
    danceFrame(other.danceFrame), poseT(other.poseT), poseID(other.poseID),
    deltaYaw(other.deltaYaw),
    pathTaken(other.pathTaken.begin(), other.pathTaken.end()) {}

  bool operator<(const Pose& other) {
    return ((costToGoal+distFromStart) > (other.costToGoal+other.distFromStart));
  }
  friend std::ostream& operator<<(std::ostream& o, const Pose& p) {
    const btVector3 v = p.poseT.getOrigin();
    o << "(" << v.x() << ", " << v.y() << ", " << v.z();
    o << ") at frame " << p.danceFrame << ", cost ";
    o << p.distFromStart+p.costToGoal << ", yaw " << p.deltaYaw;
    return o;
  }
    
};
#endif
