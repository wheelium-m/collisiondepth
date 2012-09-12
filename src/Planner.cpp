#include "Planner.h"
#include <algorithm>
using namespace std;
#define PI 3.1415926535

Planner::Planner(CollisionChecker& c) : checker(c) {}

// Distance between two points projected onto the XZ plane.
float xzDist(const btVector3& a, const btVector3& b) {
  btVector3 d = a - b;
  d.setY(0);
  return d.length();
}

inline float zDist(const btVector3& a, const btVector3& b) {
  return fabs(a.z() - b.z());
}

void Planner::followFloor(btVector3& t) {
  t.setY(deltaYPerMeter * zDist(t,pathStartLoc) + pathStartLoc.y());
}

btVector3 Planner::followFloorA(btVector3 t) {
  t.setY(deltaYPerMeter * zDist(t,pathStartLoc) + pathStartLoc.y());
  return t;
}

// Check a pose for collision.
bool Planner::checkPose(ScriptedMotion& script, const Pose& pose) {
  int tmpFrame = pose.danceFrame;
  const map<string,float>& posture = script.getJointAngles(tmpFrame);
  static vector<float> postureVec;
  checker.makeJointVector(posture, postureVec);
  static vector<bool> collisionVec;

  // We encode poses with a fixed discretization of 0.1m, so we should
  // truncate the pose we're checking to that spatial resolution
  // before checking for collisions.
  btTransform tr = pose.poseT;
  tr.setOrigin(btVector3((int)(tr.getOrigin().x() * 10.0f) / 10.0f,
                         tr.getOrigin().y(),
                         (int)(tr.getOrigin().z() * 10.0f) / 10.0f));
  checker.getCollisionInfo(tr, SPHERE_RADIUS, postureVec, collisionVec);
  for(int i = 0; i < collisionVec.size(); i++) {
    if(collisionVec[i]) return false;
  }
  return true;
}

// Cache some values used during planning.
void Planner::prepPlan(const btTransform& pathStart, 
                       const btTransform& pathGoal) {
  pathStartLoc = pathStart.getOrigin();
  pathGoalLoc = pathGoal.getOrigin();
  float zDistToGoal = zDist(pathStartLoc, pathGoalLoc);
  float deltaY = pathGoalLoc.y() - pathStartLoc.y();
  deltaYPerMeter = deltaY / zDistToGoal;
}

// Convert a pose into an integer index by discretizing its X, Z, and
// yaw components.
int Planner::poseToCoordinate(const Pose& p) {
  const btVector3 v = p.poseT.getOrigin() - pathStartLoc;
  int halfxdim = xdim / 2;
  int halfzdim = zdim / 2;
  const int ix = (int)(v.x() * 10.0f) + halfxdim;
  const int iz = (int)(v.z() * 10.0f) + halfzdim;
  if(ix < 0 || ix >= xdim || iz < 0 || iz >= zdim || 
     abs(p.deltaYaw) > 9 || p.danceFrame > 399) return -1;

  const int iyaw = p.deltaYaw+10;
  const int idance = p.danceFrame / 4;
  return ix + iz*xdim + iyaw*xdim*zdim + idance*xdim*zdim*yawdim;
}

btTransform Planner::coordinateToTransform(const btTransform& start, int i) {
  const int idance = i / (xdim*zdim*yawdim);
  i -= idance*xdim*zdim*yawdim;
  const int iyaw = i / (xdim*zdim);
  i -= iyaw*xdim*zdim;
  const int iz = i / xdim;
  i -= iz*xdim;
  const int ix = i;
  
  const int halfxdim = xdim / 2;
  const int halfzdim = zdim / 2;
  btVector3 t((float)(ix - halfxdim)/10.0f + start.getOrigin().x(), 
              0, 
              (float)(iz - halfzdim)/10.0f + start.getOrigin().z());
  followFloor(t);

  btTransform rot = btTransform(start.getRotation());
  btQuaternion r = btQuaternion(rot(btVector3(0,0,1)), PI*0.06*(float)(iyaw-10));
  return btTransform(r*rot.getRotation(), t);
}

inline bool Planner::viablePose(ScriptedMotion& script, const Pose& p) {
  return p.poseID >= 0 && parents[p.poseID] == -1 && checkPose(script, p);
}

inline bool Planner::viablePoseDebug(ScriptedMotion& script, const Pose& p) {
  if(p.poseID <= 0) {
    cout << "Bad poseID " << p.poseID << endl;
  } else if(!checkPose(script, p)) {
    cout << "Collision for " << p << endl;
  } else if(parents[p.poseID] != -1) {
    cout << "Already visited " << p << endl;
  } else return true;
  return false;
}

inline void Planner::pushPose(const Pose& p, const int parent) {
  heap.push_back(p);
  push_heap(heap.begin(), heap.end());
  parents[p.poseID] = parent;
}
const int goalYaw = 9;

void Planner::addNeighbors(ScriptedMotion& script, 
                           const int parent, const Pose& p) {
  btVector3 loc = p.poseT.getOrigin();
  btTransform rot = btTransform(p.poseT.getRotation());

  float speed = 0.1f;
  //float speed = 0.05f;
  int danceAdvance = 4;

  int nextFrame = p.danceFrame + danceAdvance;
  if(nextFrame >= script.numFrames()) nextFrame -= script.numFrames();

  // forward
  btVector3 newLoc = followFloorA(loc + rot(btVector3(speed,0,0)));
  Pose forward(p.distFromStart + 0.2,
               xzDist(newLoc, pathGoalLoc) + abs(goalYaw - p.deltaYaw),
               nextFrame,
               btTransform(p.poseT.getRotation(), newLoc),
               p.deltaYaw);
  forward.poseID = poseToCoordinate(forward);
  if(viablePose(script, forward)) pushPose(forward, parent);

  // move left
  newLoc = followFloorA(loc + rot(btVector3(0,speed,0)));
  Pose stepLeft(p.distFromStart + 0.2,
                xzDist(newLoc, pathGoalLoc) + abs(goalYaw - p.deltaYaw),
                nextFrame,
                btTransform(p.poseT.getRotation(), newLoc),
                p.deltaYaw);
  stepLeft.poseID = poseToCoordinate(stepLeft);
  if(viablePose(script, stepLeft)) pushPose(stepLeft, parent);

  // move right
  newLoc = followFloorA(loc + rot(btVector3(0,-speed,0)));
  Pose stepRight(p.distFromStart + 0.2,
                 xzDist(newLoc, pathGoalLoc) + abs(goalYaw - p.deltaYaw),
                 nextFrame,
                 btTransform(p.poseT.getRotation(), newLoc),
                 p.deltaYaw);
  stepRight.poseID = poseToCoordinate(stepRight);
  if(viablePose(script, stepRight)) pushPose(stepRight, parent);

  // turn left
  btQuaternion yaw = btQuaternion(rot(btVector3(0,0,1)), PI*0.06);
  Pose turnLeft(p.distFromStart + 0.2,
                //p.costToGoal,
                xzDist(loc, pathGoalLoc) + abs(goalYaw - p.deltaYaw),
                nextFrame,
                btTransform(yaw*p.poseT.getRotation(), loc),
                p.deltaYaw + 1);
  turnLeft.poseID = poseToCoordinate(turnLeft);
  if(viablePose(script, turnLeft)) pushPose(turnLeft, parent);

  // turn right
  yaw = btQuaternion(rot(btVector3(0,0,1)), -PI*0.06);
  Pose turnRight(p.distFromStart + 0.2,
                 //p.costToGoal,
                 xzDist(loc, pathGoalLoc) + abs(goalYaw - p.deltaYaw),
                 nextFrame,
                 btTransform(yaw*p.poseT.getRotation(), loc),
                 p.deltaYaw - 1);
  turnRight.poseID = poseToCoordinate(turnRight);
  if(viablePose(script, turnRight)) pushPose(turnRight, parent);

  // Wait
  Pose wait(p.distFromStart + 5,
            p.costToGoal,
            nextFrame,
            p.poseT,
            p.deltaYaw);
  wait.poseID = poseToCoordinate(wait);
  if(viablePose(script, wait)) pushPose(wait, parent);
}

vector<btTransform> Planner::reconstructPath(const btTransform start, 
                                             int goalIndex) {
  vector<btTransform> v;
  while(goalIndex != -1) {
    v.push_back(coordinateToTransform(start, goalIndex));
    goalIndex = parents[goalIndex];
  }
  reverse(v.begin(), v.end());
  cout << "Path found:" << endl;
  btVector3 prev = start.getOrigin();
  float totalDist = 0.0f;
  for(int i = 0; i < v.size(); i++) {
    btVector3 t = v[i].getOrigin();
    //cout << " (" << t.x() << ", " << t.y() << ", " << t.z() << ")" << endl;
    totalDist += t.distance(prev);
    prev = t;
  }
  cout << "Path is " << totalDist << " meters long." << endl;
  return v;
}

inline Pose Planner::popPose() {
  Pose p = heap.front();
  pop_heap(heap.begin(), heap.end());
  heap.pop_back();
  return p;
}

vector<btTransform> Planner::findPath(ScriptedMotion& script, 
                                      const btTransform& pathStart,
                                      const btTransform& pathGoal) {
  prepPlan(pathStart, pathGoal);
  parents.assign(xdim*zdim*yawdim*animdim, -1);
  //vector<int> parents(xdim*zdim*yawdim*animdim, -1); // X * Z * Yaw * animation
  heap.clear();
  
  Pose startPose(0, xzDist(pathStartLoc, pathGoalLoc), 0, 
                 pathStart, 0, vector<btTransform>());
  startPose.poseID = poseToCoordinate(startPose);
  heap.push_back(startPose);
  make_heap(heap.begin(), heap.end());

  int numSteps;
  const int maxSteps = 500000;
  for(numSteps = 0; numSteps < maxSteps; numSteps++) {
    if(heap.size() == 0) {
      cout << "No moves left!" << endl;
      break;
    }
    Pose p = popPose();

    // Report current progress
    if(numSteps % 10000 == 0) {
      cout << "At " << p << endl;
    }

    if(p.poseT.getOrigin().distance(pathGoalLoc) < 0.2) {
      cout << "Got to goal at frame " << p.danceFrame << "!" << endl;
      cout << numSteps << " nodes expanded!" << endl;
      return reconstructPath(btTransform(pathStart), p.poseID);
    } else {
      addNeighbors(script, p.poseID, p);
    }
  }

  if(numSteps == maxSteps)
    cout << "Ran out of steps!" << endl;
  else
    cout << numSteps << " nodes expanded!" << endl;
  vector<btTransform> noPath;
  noPath.push_back(pathStart);
  return noPath;
}

