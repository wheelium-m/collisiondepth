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
    char name[offset+4];
    memcpy(name, m->curr->name.c_str(), m->curr->name.length());
    name[offset] = '_';
    name[offset+1] = '_';
    name[offset+2] = 0;
    name[offset+3] = 0;
    
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

void CollisionChecker::getCollisionInfo(const btTransform& camera,
                                        const float sphereRadius,
                                        const vector<float>& jointAngles,
                                        vector<bool>& info) {
  const btVector3 origin = btVector3(0,0,0);
  info.clear();
  info.resize(this->numSpheres, true);
  for(vector<const DepthMap*>::const_iterator dit = depthMaps.begin();
      dit != depthMaps.end();
      dit++) {
    const DepthMap* depth = *dit;
    const float* dmap = depth->getMap(sphereRadius);
    const btVector3 imageScale = btVector3(depth->focalLength, 
                                           depth->focalLength, 1);
    const btVector3 imageCenter = btVector3(depth->width / 2, 
                                            depth->height / 2, 0);
    const int w = depth->width;
    const int h = depth->height;
    const btTransform depthTrans = depth->trans.inverse();

    // Pose the model
    int jointIndex = 0;
    int sphereIndex = -1;
    queue<pair<const btTransform, const ModelTree*> > q;
    const ModelTree* root = models[0];
    q.push(make_pair(btTransform::getIdentity(), root));
    while(!q.empty()) {
      btTransform t = q.front().first; // parent transform
      const ModelTree* child = q.front().second;
      q.pop();
      float angle = jointAngles[jointIndex++];
      if(angle != 0)
        t = t * btTransform(btQuaternion(child->curr->axis, angle), 
                            child->curr->trans.getOrigin());
      else 
        t = t * child->curr->trans;
    
      // Now check the child joint's spheres
      const int sz = child->curr->points.size();
      const btTransform modelToCamera = camera * t;
      btVector3 spherePt = modelToCamera(origin);
      int i = 0;
      while(true) {
        sphereIndex++;
        btVector3 camSpace = depthTrans(spherePt);
        
        // Can't say anything about a sphere behind the camera
        if(camSpace.z() - sphereRadius < 0) {
          if(i == sz) break;
          spherePt = modelToCamera(child->curr->points[i]);
          i++;
          continue;
        }

        btVector3 screenSpace = camSpace;
        if(screenSpace.z() != 0) screenSpace *= 1.0 / screenSpace.z();
        screenSpace *= imageScale;
        screenSpace += imageCenter;
        const int screenX = (int)screenSpace.x();
        const int screenY = (int)screenSpace.y();
        if(screenX < 0 || screenX >= w ||
           screenY < 0 || screenY >= h) {
          if(i == sz) break;
          spherePt = modelToCamera(child->curr->points[i]);
          i++;
          continue;
        }
        // Make a note if we have have evidence that a sphere is *not*
        // in collision.
        if(!depth->collides(dmap, screenX, screenY, camSpace.z() + sphereRadius))
          info[sphereIndex] = false;

        // Iterate through the child spheres
        if(i == sz) break;
        spherePt = modelToCamera(child->curr->points[i]);
        i++;
      }
      for(ModelTree::child_iterator it = child->begin(); 
          it != child->end(); it++) {
        q.push(make_pair(t, *it));
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
