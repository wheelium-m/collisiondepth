#include "CollisionChecker.h"
#include <queue>
#include <map>
#include <sstream>
#include <iostream>
#include "PathHelper.h"
#include <dirent.h>
#include <sys/time.h>

// #ifdef MAC
// #include <thread>
// #endif

#ifdef FISHEYE
#define FOCAL_LENGTH 160.0
#else
#define FOCAL_LENGTH 320.0
#endif

using namespace std;

void CollisionChecker::init(const int modelNumber, const ModelTree* root) {
  this->models.push_back(root);
  queue<const ModelTree*> q;
  q.push(root);
  this->numSpheres = 0;
  this->numJoints = 0;
  // Initialize SBPL joint remapping
  const string langleNames[] = {"l_shoulder_pan_link", "l_shoulder_lift_link", 
                                "l_upper_arm_roll_link", "l_elbow_flex_link",
                                "l_forearm_roll_link", "l_wrist_flex_link", 
                                "l_wrist_roll_link"};
  const string rangleNames[] = {"r_shoulder_pan_link", "r_shoulder_lift_link",
                                "r_upper_arm_roll_link", "r_elbow_flex_link",
                                "r_forearm_roll_link", "r_wrist_flex_link",
                                "r_wrist_roll_link"};
  langleRemap.resize(7);
  rangleRemap.resize(7);
  while(!q.empty()) {
    const ModelTree* m = q.front();
    q.pop();
    //cout << "Encountered frame " << m->curr->name << "(" << m->curr->name.size() << ")" << endl;
    for(int i = 0; i < 7; i++) {
      //cout << "  " << langleNames[i] << "(" << langleNames[i].size() << ")" << endl;
      if(langleNames[i].compare(m->curr->name) == 0) {
	//cout << "langleRemap " << i << " to " << numJoints << endl;
	langleRemap[i] = numJoints;
      }
    }
    for(int i = 0; i < 7; i++)
      if(rangleNames[i].compare(m->curr->name) == 0)
	rangleRemap[i] = numJoints;
    this->numJoints++;
    this->numSpheres += m->curr->points.size();
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++) {
      q.push(*it);
    }
  }
  cout << "Model has " << numSpheres << " spheres" << endl;
  stats.modelNumber = modelNumber;
  stats.numChecks = 0;
  stats.numViews = 0;
  stats.preprocessingTime = 0.0;
}

CollisionChecker::CollisionChecker(const ModelTree* root) {
  init(0, root);
}

CollisionChecker::CollisionChecker(const int modelNumber, const ModelTree* root) {
  init(modelNumber, root);
}

void CollisionChecker::addDepthMap(const DepthMap* depthMap) {
  this->depthMaps.push_back(depthMap);
}

const DepthMap* CollisionChecker::getDepthMap(int i) {
  for(int j = 0; j < depthMaps.size(); j++) {
    if(depthMaps[j]->depthID == i) return depthMaps[j];
  }
  return NULL;
  //return this->depthMaps[i];
}

const DepthMap* CollisionChecker::getActiveDepthMap() {
  return depthMaps[0];
}

int CollisionChecker::numDepthMaps() const {
  return this->depthMaps.size();
}

void natToStr(const int i, char* s) {
  if(i < 10) {
    *s = (char)((int)'0' + i);
  } else if(i < 100) {
    *s = (char)((int)'0' + (i / 10));
    *(s+1) = (char)((int)'0' + (i % 10));
  } else if(i < 1000) {
    *s = (char)((int)'0' + (i / 100));
    *(s+1) = (char)((int)'0' + (i%100)/10);
    *(s+2) = (char)((int)'0' + i%10);
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

    // No default sphere-per-link
    //collisionMap[m->curr->name] = !collisionVec[sphereIndex++];

    // ASCII string munging.
    int offset = m->curr->name.length();
    char name[offset+6];
    memcpy(name, m->curr->name.c_str(), m->curr->name.length());
    name[offset] = '_';
    name[offset+1] = '_';
    name[offset+2] = 0;
    name[offset+3] = 0;
    name[offset+4] = 0;
    name[offset+5] = 0;
    
    for(int i = 0; i < m->curr->points.size(); i++) {
      //stringstream n;
      //n << m->curr->name << "__" << i;
      //collisionMap[n.str()] = collisionVec[sphereIndex++];
      natToStr(i, &name[offset+2]);
      collisionMap[name] = !collisionVec[sphereIndex++];
    }
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++) {
      q.push(*it);
    }
  }
}

//static vector<vector<pair<btTransform, const ModelTree*> > > perThreadQ(64);

// Check a posed model against an individual depth map.
void checkMap(const int threadId,
              const vector<const ModelTree*>& models,
              vector<bool>* possibleCollision,
              const DepthMap* const depth,
              const btTransform& robotFrame,
              const float sphereRadius,
              const vector<float>& jointAngles) {
  //const btVector3 origin = btVector3(0,0,0);
  const float* const dmap = depth->getMap(sphereRadius);
  const int w = depth->width;
  const int h = depth->height;
  const btTransform depthTrans = depth->trans;
  //const float focalLength = depth->focalLength;

#ifdef FISHEYE
  const float focalLengthX = 160.0f;
  const float focalLengthY = 120.0f;
#else
  const float focalLengthX = 320.0f;
  const float focalLengthY = 240.0f;
#endif

  const int halfW = w / 2;
  const int halfH = h / 2;

  // Pose the model
  int jointIndex = 0;
  int sphereIndex = -1;

  // Making the queue static should allow for less resizing on
  // successive checks.
  static vector<pair<btTransform, const ModelTree*> > q;
  //vector<pair<btTransform, const ModelTree*> >& q = perThreadQ[threadId];
  q.clear();
  q.push_back(make_pair(btTransform::getIdentity(), models[0]));

  int qHead = 0;
  int qRemaining = 1;
  const btTransform robotToCamera = depthTrans * robotFrame;
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
  
    if(j->points.size()) {
      const btTransform modelToCamera = robotToCamera * t;
      for(int i = 0; i < j->points.size(); i++) {
        sphereIndex++;
        btVector3 camSpace = modelToCamera(j->points[i]);
        // Can't say anything about a sphere behind the camera
        if(camSpace.z() + sphereRadius <= 0) {
          const float invZ = -1.0f / camSpace.z();
          const int screenX = (int)(camSpace.x() * focalLengthX * invZ) + halfW;
          const int screenY = (int)(-camSpace.y() * focalLengthY * invZ) + halfH;

          if(screenX >= 0 && screenX < w &&
             screenY >= 0 && screenY < h) {
            // Make a note if we have have evidence that a sphere is *not*
            // in collision.
            float observedDepth = *(dmap+screenY*w+screenX);
            
            // FIXME: This is the test we want to do...
            //if(observedDepth && sphereRadius - camSpace.z() < observedDepth)
            // But since we want to declare "no information" as a good
            // thing for testing purposes...
            if(!observedDepth || sphereRadius - camSpace.z() < observedDepth)
              (*possibleCollision)[sphereIndex] = false;
          }
        }
      }
    }

    for(ModelTree::child_iterator it = child->begin(); 
        it != child->end(); it++) {
      q.push_back(make_pair(t, *it));
    }
    qRemaining += child->numChildren();
  }
}

// The vector possibleCollision will contain "true" for every sphere
// not confirmed as free of collision after this function is called.
void CollisionChecker::getCollisionInfo(const btTransform& robotFrame,
                                        const float sphereRadius,
                                        const vector<float>& jointAngles,
                                        vector<bool>& possibleCollision) {
  possibleCollision.clear();
  possibleCollision.resize(this->numSpheres, true);

  // For a single pose, launching threads is worse than doing things
  // serially.

  // thread* t = new thread[depthMaps.size()];

  stats.numChecks++;
  for(int i = 0; i < depthMaps.size(); i++) {
    // Quick visibility feasibility test
    btVector3 test = depthMaps[i]->trans(robotFrame.getOrigin());
    if(test.z() > 0 || test.length2() > 100) continue;

    // Tighter FOV test
    //if(acos(btVector3(0,0,-1).dot(test) / test.length()) > 1.65) continue;

    stats.numViews++;

    checkMap(i, models, &possibleCollision, depthMaps[i], robotFrame, 
             sphereRadius, jointAngles);

    // Check if every sphere has been seen free of collision
    int j;
    for(j = 0; j < possibleCollision.size(); j++)
      if(possibleCollision[j]) break;
    if(j == possibleCollision.size()) {
      // Shuffle so depth map is checked first next time.

      // This is a rough heuristic that is often pointless (e.g. when
      // multiple views are needed to confirm collision freedom, the
      // order is swapped every time), but does capture the optimistic
      // common case of a single view being sufficient.
      if(i != 0) {
        const DepthMap* tmp = depthMaps[0];
        depthMaps[0] = depthMaps[i];
        depthMaps[i] = tmp;
      }
      break;
    }

    // t[i] = thread(checkMap, i, models, &possibleCollision, depthMaps[i], 
    //               robotFrame, sphereRadius, jointAngles);
  }

  // for(int i = 0; i < depthMaps.size(); i++) {
  //   t[i].join();
  // }
  // delete [] t;
}

/*
 * std::vector<double> &langles - 7 joint angles for the left
 * arm. {l_shoulder_pan_joint, l_shoulder_lift_joint,
 * l_upper_arm_roll_joint, l_elbow_flex_joint, l_forearm_roll_joint,
 * l_wrist_flex_joint, l_wrist_roll_joint}

 * std::vector<double> &rangles - 7 joint angles for the right
 * arm. {r_shoulder_pan_joint, r_shoulder_lift_joint,
 * r_upper_arm_roll_joint, r_elbow_flex_joint, r_forearm_roll_joint,
 * r_wrist_flex_joint, r_wrist_roll_joint}

 * BodyPose &pose - This is the position of the base in the map frame
 * and the torso height. This is a struct that you should include in
 * your own ROS package. I attached the file here.  

 * bool verbose - enables debug output unsigned char &dist - this is
 * the distance from the nearest obstacle in cells. you can ignore
 * this.

 * int &debug_code - I have error codes for debugging purposes. For
 * the sake of time - you can ignore them too.  

 * returns a bool - this is the money bool
 */
bool CollisionChecker::checkCollision(vector<double> &langles, 
                                      vector<double> &rangles, 
                                      BodyPose &pose, 
                                      bool verbose, 
                                      unsigned char &dist, 
                                      int &debug_code) {
  static vector<float> jointAngles(this->numJoints);
  static vector<bool> collisions;

  // Joint angle remapping
  //cout << "Joint angle remapping" << endl;
  for(int i = 0; i < 7; i++) {
    //cout << "remapping langle " << i << " to " << langleRemap[i] << endl;
    jointAngles[langleRemap[i]] = langles[i];
  }

  for(int i = 0; i < 7; i++) {
    jointAngles[rangleRemap[i]] = rangles[i];
  }

  //cout << "Setting up robotFrame" << endl;
  btTransform robotFrame = btTransform(btQuaternion(btVector3(0,0,1), pose.theta),
                                       btVector3(pose.x, pose.y, pose.z));
  getCollisionInfo(robotFrame, SPHERE_RADIUS*MODEL_SCALE, jointAngles, collisions);
  for(int i = 0; i < collisions.size(); i++) 
    if(collisions[i]) return false;
  return true;
}

void CollisionChecker::getCollisionSpheres(vector<double>& langles,
                                           vector<double>& rangles,
                                           BodyPose& pose,
                                           string group_name,
                                           vector<vector<double> >& spheres) {
  static vector<float> jointAngles(this->numJoints);
  static vector<bool> collisions;

  int oldSize = spheres.size();
  spheres.resize(this->numSpheres);
  for(int i = oldSize; i < this->numSpheres; i++) {
    spheres[i].resize(4);
  }

  // Joint angle remapping
  for(int i = 0; i < 7; i++) {
    jointAngles[langleRemap[i]] = langles[i];
  }

  for(int i = 0; i < 7; i++) {
    jointAngles[rangleRemap[i]] = rangles[i];
  }

  btTransform robotFrame = btTransform(btQuaternion(btVector3(0,0,1), pose.theta),
                                       btVector3(pose.x, pose.y, pose.z));

  // Model tree traversal is copied from checkMap
  static vector<pair<btTransform, const ModelTree*> > q;
  q.clear();
  q.push_back(make_pair(btTransform::getIdentity(), models[0]));
  int sphereIndex = -1;
  int jointIndex = 0;
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

    const btTransform modelToCamera = robotFrame * t;
    for(int i = 0; i < j->points.size(); i++) {
      const btVector3 spherePt = modelToCamera(j->points[i]);
      sphereIndex++;
      vector<double>& sphereVector = spheres[sphereIndex];
      sphereVector[0] = spherePt.x();
      sphereVector[1] = spherePt.y();
      sphereVector[2] = spherePt.z();
      sphereVector[3] = j->radius;
    }

    for(ModelTree::child_iterator it = child->begin(); 
        it != child->end(); it++) {
      q.push_back(make_pair(t, *it));
      qRemaining++;
    }
  }
}

extern btTransform parsePose(const char* filename);

// Convenience intialization for the Levine Hall data set
void CollisionChecker::levineInit() {
  DIR *dp;
  struct dirent *dirp;
#ifdef FISHEYE
  string depthMapDir("etc/levine/fisheye/");
#else
  string depthMapDir("etc/levine/");
#endif
  makePath(depthMapDir);
  if((dp = opendir(depthMapDir.c_str())) == NULL) {
    cout << "ERROR OPENING DEPTH MAP DIRECTORY" << endl;
    exit(-1);
  }
  vector<string> imgFiles;
  vector<string> poseFiles;
  while((dirp=readdir(dp)) != NULL) {
    string fname(dirp->d_name);
    if(fname.find(".bin") != string::npos) {
      imgFiles.push_back(string(fname));
      string pname(fname);
      pname = pname.substr(0,pname.size()-4);
      pname.append("pose.txt");
      poseFiles.push_back(pname);
    }
    // else if(fname.find("pose.txt") != string::npos) 
    //   poseFiles.push_back(string(fname));
  }
  if(imgFiles.size() != poseFiles.size()) {
    cout << "Didn't find matching number of depth images and poses" << endl;
    exit(-1);
  }
  for(int i = 0; i < imgFiles.size(); i++) {
    imgFiles[i].insert(0, depthMapDir);
    poseFiles[i].insert(0, depthMapDir);
    cout << "Image file: " << imgFiles[i] << endl;
    cout << "Pose file: " << poseFiles[i] << endl;
  }
  // const char* dmaps[] = {"etc/depths1.bin", "etc/depths1pose.txt",
  //                        "etc/depths2.bin", "etc/depths2pose.txt",
  //                        "etc/depths3.bin", "etc/depths3pose.txt",
  //                        "etc/depths4.bin", "etc/depths4pose.txt"};

  struct timeval start;
  struct timeval stop;
  gettimeofday(&start, NULL);
  for(int i = 0; i < imgFiles.size(); i++) {
    DepthMap* depth = new DepthMap();
    depth->depthID = this->numDepthMaps();
    depth->getKinectMapFromFile(FOCAL_LENGTH, imgFiles[i].c_str());
    
    btTransform t = parsePose(poseFiles[i].c_str());
    depth->camOrigin = t.getOrigin();
    depth->camOrientation = t.getRotation();
    depth->trans = btTransform(t.getRotation(),
                               btTransform(t.getRotation())(-1 * t.getOrigin()));
    depth->transInv = depth->trans.inverse();
    depth->addDilation(SPHERE_RADIUS*MODEL_SCALE);
    addDepthMap(depth);
  }
  gettimeofday(&stop, NULL);
  stats.preprocessingTime = (stop.tv_sec - start.tv_sec) + \
    0.000001 * (double)(stop.tv_usec - start.tv_usec);

  // Per-image time is the most useful number.
  stats.preprocessingTime /= (double)imgFiles.size();
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

// Return a vector of camera boses. The vector is n*7 elements long,
// where we represent each of n poses as translation and a rotation
// quaternion. Translation is a 3D vector with {X,Y,Z} components, and
// the quaternion is a 4D vector with {X,Y,Z,W} components.
vector<float> CollisionChecker::getDepthPoses() {
  vector<float> v(7*depthMaps.size());
  for(int i = 0; i < depthMaps.size(); i++) {
    const int o = i*7;
    const btVector3 t = depthMaps[i]->camOrigin;
    const btQuaternion r = depthMaps[i]->camOrientation;
    v[o] = t.x();
    v[o+1] = t.y();
    v[o+2] = t.z();
    v[o+3] = r.x();
    v[o+4] = r.y();
    v[o+5] = r.z();
    v[o+6] = r.w();
  }
  return v;
}

void CollisionChecker::resetStats() {
  stats.numChecks = 0;
  stats.numViews = 0;
}

string makeStatName(const string& prefix, const int suffix) {
  stringstream ss;
  ss << prefix << suffix;
  return ss.str();
}

void CollisionChecker::getStats(vector<string>& fieldNames, 
                                vector<double>& fieldValues) {
  fieldNames.clear();
  fieldValues.clear();

  fieldNames.push_back(makeStatName("num_checks", stats.modelNumber));
  fieldValues.push_back((double)stats.numChecks);

  fieldNames.push_back(makeStatName("num_views", stats.modelNumber));
  fieldValues.push_back((double)stats.numViews);

  fieldNames.push_back(makeStatName("preprocessing_per_image", 
                                    stats.modelNumber));
  fieldValues.push_back(stats.preprocessingTime);
}
