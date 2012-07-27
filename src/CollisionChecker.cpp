#include "CollisionChecker.h"
#include <queue>
#include <map>
#include <sstream>
#include <iostream>
using namespace std;

CollisionChecker::CollisionChecker(const ModelTree* root) {
  this->models.push_back(root);
  queue<ModelTree*> q;
  for(ModelTree::child_iterator it = root->begin(); it != root->end(); it++) {
    q.push(*it);
  }
  this->jointHash[root->curr->name] = root->curr;
  while(!q.empty()) {
    ModelTree* m = q.front();
    q.pop();
    this->jointHash[m->curr->name] = m->curr;
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++) {
      q.push(*it);
    }
  }
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

void CollisionChecker::getCollisionInfo(const btTransform& camera,
                                        const float sphereRadius,
                                        const map<string,float>& jointAngles,
                                        map<string,bool>& info) {
  const btVector3 origin = btVector3(0,0,0);
  for(vector<const DepthMap*>::const_iterator dit = depthMaps.begin();
      dit != depthMaps.end();
      dit++) {
    const DepthMap* depth = *dit;
    const float* dmap = depth->getMap(sphereRadius);
    const btVector3 imageScale = btVector3(depth->focalLength, depth->focalLength, 1);
    const btVector3 imageCenter = btVector3(depth->width / 2, depth->height / 2, 0);
    const int w = depth->width;
    const int h = depth->height;
    const btTransform depthTrans = depth->trans.inverse();

    long postureBloom = 0;
    for(map<string,float>::const_iterator it = jointAngles.begin();
        it != jointAngles.end();
        it++) {
      postureBloom |= (long)this->jointHash[it->first];
    }

    // Pose the model
    queue<pair<const btTransform, const ModelTree*> > q;
    const ModelTree* root = models[0];
    q.push(make_pair(btTransform::getIdentity(), root));
    while(!q.empty()) {
      btTransform t = q.front().first; // parent transform
      const ModelTree* child = q.front().second;
      const string baseName = child->curr->name;
      q.pop();
      const long jhash = (long)this->jointHash[baseName];
      if((jhash & postureBloom) == jhash) {
        const map<string,float>::const_iterator angle = 
          jointAngles.find(baseName);

        // Compute the composite joint transform
        if(angle != jointAngles.end())
          t = t * btTransform(btQuaternion(child->curr->axis, angle->second), 
                              child->curr->trans.getOrigin());
        else 
          t = t * child->curr->trans;
      } else {
        t = t * child->curr->trans;
      }
    
      // Now check the child joint's spheres
      const int nameIndexOffset = baseName.length() + 2;
      char name[nameIndexOffset + 2];
      memcpy(name, baseName.c_str(), baseName.length());
      memset(&name[nameIndexOffset-2], 0, 4);
      char* const nameOffset = &name[nameIndexOffset];

      const int sz = child->curr->points.size();
      const btTransform modelToCamera = camera * t;
      btVector3 spherePt = modelToCamera(origin);
      int i = 0;
      while(true) {
        btVector3 camSpace = depthTrans(spherePt);
        if(i == 1) {
          name[nameIndexOffset - 2] = '_';
          name[nameIndexOffset - 1] = '_';
        }
        
        // Can't say anything about a sphere behind the camera
        if(camSpace.z() - sphereRadius < 0) {
          if(i == sz) break;
          natToStr(i, nameOffset);
          //sprintf(&name[nameIndexOffset], "%d", i);
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
          natToStr(i, nameOffset);
          //sprintf(&name[nameIndexOffset], "%d", i);
          spherePt = modelToCamera(child->curr->points[i]);
          i++;
          continue;
        }
        if(depth->collides(dmap, screenX, screenY, camSpace.z() + sphereRadius)) {
          // Shouldn't overwrite a "false" in the collision info table
          if(info.find(name) == info.end())
            info[name] = true;
        } else {
          info[name] = false;
        }

        // Iterate through the child spheres
        if(i == sz) break;
        natToStr(i,nameOffset);
        //sprintf(&name[nameIndexOffset], "%d", i);
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
                                   const map<string,float>& jointAngles) {
  map<string,bool> collisions;
  getCollisionInfo(camera, sphereRadius, jointAngles, collisions);
  map<string,bool>::const_iterator it = collisions.begin();
  for(; it != collisions.end(); it++) {
    if((*it).second) return true;
  }
  return false;
}
