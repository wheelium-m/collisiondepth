#include "CollisionChecker.h"
#include <queue>
#include <map>
#include <sstream>
#include <iostream>
using namespace std;

CollisionChecker::CollisionChecker(const ModelTree* root) {
  this->models.push_back(root);
  queue<const ModelTree*> q;
  q.push(root);
  this->numSpheres = 0;
  while(!q.empty()) {
    const ModelTree* m = q.front();
    q.pop();
    this->numSpheres += 1 + m->curr->points.size();
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++) {
      q.push(*it);
    }
  }
  cout << "Model has " << numSpheres << " spheres" << endl;
}

void CollisionChecker::addDepthMap(const DepthMap* depthMap) {
  this->depthMaps.push_back(depthMap);
}

const DepthMap* CollisionChecker::getDepthMap(int i) {
  return this->depthMaps[i];
}

void natToStr(const int i, char* s) {
  if(i < 10) {
    *s = (char)((int)'0' + i);
  } else {
    *s = (char)((int)'0' + (i / 10));
    *(s+1) = (char)((int)'0' + (i % 10));
  }
}

void CollisionChecker::makeJointVector(const map<string,float>& jointAngleMap, 
                                       vector<float>& jointAngleVec) {
  queue<const ModelTree*> q;
  q.push(this->models[0]);
  jointAngleVec.clear();
  while(!q.empty()) {
    const ModelTree* m = q.front();
    q.pop();
    map<string,float>::const_iterator angle = jointAngleMap.find(m->curr->name);
    if(angle == jointAngleMap.end())
      jointAngleVec.push_back(0);
    else
      jointAngleVec.push_back(angle->second);
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++) {
      q.push(*it);
    }
  }
}

void CollisionChecker::makeCollisionMap(const vector<bool>& collisionVec,
                                        map<string,bool>& collisionMap) {
  int sphereIndex = 0;
  queue<const ModelTree*> q;
  q.push(this->models[0]);
  collisionMap.clear();
  while(!q.empty()) {
    const ModelTree* m = q.front();
    q.pop();
    collisionMap[m->curr->name] = collisionVec[sphereIndex++];

    // ASCII string munging.
    int offset = m->curr->name.length();
    char name[offset+5];
    memcpy(name, m->curr->name.c_str(), m->curr->name.length());
    name[offset] = '_';
    name[offset+1] = '_';
    name[offset+2] = 0;
    name[offset+3] = 0;
    name[offset+4] = 0;
    
    for(int i = 0; i < m->curr->points.size(); i++) {
      //stringstream n;
      //n << m->curr->name << "__" << i;
      //collisionMap[n.str()] = collisionVec[sphereIndex++];
      natToStr(i, &name[offset+2]);
      collisionMap[name] = collisionVec[sphereIndex++];
    }
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++) {
      q.push(*it);
    }
  }
}

void CollisionChecker::getCollisionInfo(const btTransform& robotFrame,
                                        const float sphereRadius,
                                        const vector<float>& jointAngles,
                                        vector<bool>& info) {
  const btVector3 origin = btVector3(0,0,0);
  info.clear();
  info.resize(this->numSpheres, true);
  for(vector<const DepthMap*>::const_iterator dit = depthMaps.begin();
      dit != depthMaps.end();
      dit++) {
    const DepthMap* const depth = *dit;
    const float* const dmap = depth->getMap(sphereRadius);
    const int w = depth->width;
    const int h = depth->height;
    const btTransform depthTrans = depth->trans;
    const float focalLength = depth->focalLength;
    const int halfW = w / 2;
    const int halfH = h / 2;

    // Pose the model
    int jointIndex = 0;
    int sphereIndex = -1;

    // Making the queue static should allow for less resizing on
    // successive checks.
    static vector<pair<btTransform, const ModelTree*> > q;
    q.clear();
    q.push_back(make_pair(btTransform::getIdentity(), models[0]));

    int qHead = 0;
    int qRemaining = 1;
    while(qRemaining) {
      btTransform t = q[qHead].first;
      const ModelTree* child = q[qHead].second;
      qHead++;
      qRemaining--;
      const float angle = jointAngles[jointIndex++];
      const Joint* const j = child->curr;
      
      if(angle != 0)
        t = t * btTransform(btQuaternion(j->axis, angle), j->trans.getOrigin());
      else 
        t = t * j->trans;
    
      // Now check the child joint's spheres
      const int sz = j->points.size();
      const btTransform modelToCamera = robotFrame * t;

      btVector3 spherePt = modelToCamera(origin);
      for(int i = 0;; i++) {
        sphereIndex++;
        btVector3 camSpace = depthTrans(spherePt);

        camSpace.setZ(-camSpace.getZ());
        // Can't say anything about a sphere behind the camera
        if(camSpace.z() - sphereRadius >= 0) {
          const float invZ = focalLength / camSpace.z();
          const int screenX = (int)(camSpace.x() * invZ) + halfW;
          const int screenY = (int)(camSpace.y() * invZ) + halfH;

          if(screenX >= 0 && screenX < w &&
             screenY >= 0 && screenY < h) {
            // Make a note if we have have evidence that a sphere is *not*
            // in collision.
            float observedDepth = *(dmap+screenY*w+screenX);
            if(observedDepth && observedDepth < camSpace.z() + sphereRadius)
              info[sphereIndex] = false;
          }
        }
        // Iterate through the child spheres
        if(i == sz) break;
        spherePt = modelToCamera(j->points[i]);
      }

      for(ModelTree::child_iterator it = child->begin(); 
          it != child->end(); it++) {
        q.push_back(make_pair(t, *it));
        qRemaining++;
      }
    }
  }
}

void CollisionChecker::getCollisionInfoReference(const btTransform& camera,
                                                 const float sphereRadius,
                                                 const map<string,float>& jointAngles,
                                                 map<string,bool>& info) {
  ModelTree* root = poseModel(*(models[0]), jointAngles);
  for(vector<const DepthMap*>::const_iterator dit = depthMaps.begin();
      dit != depthMaps.end();
      dit++) {
    vector<CameraSphere> spheres;
    // FIXME: There is confusion between the "camera" transformation
    // and the pose of the RGB-D sensor when a depth map was captured.
    transformSpheres(*root, camera, spheres);
    const DepthMap* depth = *dit;
    const float* map = depth->getMap(sphereRadius);
    for(vector<CameraSphere>::const_iterator sit = spheres.begin();
        sit != spheres.end();
        sit++) {
      // Project the screen-space sphere onto the image plane and
      // account for the center of projection.
      btVector3 camSpace = depth->trans.inverse()((*sit).center);
      
      // Can't say anything about a sphere behind the camera
      if(camSpace.z() - (*sit).r < 0) continue;

      btVector3 screenSpace = camSpace;
      if(screenSpace.z() != 0) screenSpace *= 1.0 / screenSpace.z();
      screenSpace *= btVector3(depth->focalLength, depth->focalLength, 1);
      screenSpace += btVector3(depth->width / 2, depth->height / 2, 0);
      if(screenSpace.x() < 0 || screenSpace.x() >= depth->width ||
         screenSpace.y() < 0 || screenSpace.y() >= depth->height) continue;
      if(depth->collides(map, screenSpace.x(), screenSpace.y(), 
                         camSpace.z() + sphereRadius)) {
        // Shouldn't overwrite a "false"
        if(info.find((*sit).jointName) == info.end())
          info[(*sit).jointName] = true;
      }
      else {
        info[(*sit).jointName] = false;
      }
    }
  }
  freePosedModel(root);
}

bool CollisionChecker::isColliding(const btTransform& camera,
                                   const float sphereRadius, 
                                   const vector<float>& jointAngles) {
  vector<bool> collisions;
  getCollisionInfo(camera, sphereRadius, jointAngles, collisions);
  for(int i = 0; i < collisions.size(); i++)
    if(collisions[i]) return true;
  return false;
}
