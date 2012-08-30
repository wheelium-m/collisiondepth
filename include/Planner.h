#ifndef PLANNER_H
#define PLANNER_H
#include <btBulletDynamicsCommon.h>
#include "CollisionChecker.h"
#include "ScriptedMotion.h"
#include "Pose.h"

class Planner {
private:
  CollisionChecker& checker;
  btVector3 pathStartLoc, pathGoalLoc;

  // Change in Y coordinate from start to goal per meter along the XZ
  // plane.
  float deltaYPerMeter;

  void followFloor(btVector3&);
  btVector3 followFloorA(btVector3);
  inline void pushPose(const Pose&, const int);
  inline Pose popPose();
  inline bool viablePose(ScriptedMotion&, const Pose&);
  inline bool viablePoseDebug(ScriptedMotion&, const Pose&);
  bool checkPose(ScriptedMotion&, const Pose&);
  void prepPlan(const btTransform& start, const btTransform& goal);
  int poseToCoordinate(const Pose&);
  btTransform coordinateToTransform(const btTransform&, int);
  std::vector<btTransform> reconstructPath(const btTransform, int);
  void addNeighbors(ScriptedMotion&, const int parent, const Pose&);

  // Constants bounding the work space we search.
  const static int xdim = 30;
  const static int zdim = 120; //70;
  const static int yawdim = 20;
  const static int animdim = 100;
  std::vector<int> parents;
  std::vector<Pose> heap;

public:
  Planner(CollisionChecker& checker);
  std::vector<btTransform> findPath(ScriptedMotion& script, 
                                    const btTransform& pathStart,
                                    const btTransform& pathGoal);
};
#endif
