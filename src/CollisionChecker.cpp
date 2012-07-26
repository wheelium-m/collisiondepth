#include "CollisionChecker.h"
using namespace std;

CollisionChecker::CollisionChecker(const ModelTree* rawRoot) {
  this->models.push_back(rawRoot);
}

void CollisionChecker::addDepthMap(const DepthMap* depthMap) {
  this->depthMaps.push_back(depthMap);
}

void CollisionChecker::getCollisionInfo(const btTransform& camera,
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
