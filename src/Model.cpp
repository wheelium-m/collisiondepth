#include "Model.h"
#include <iostream>
#include <fstream>
#include <map>
#include <queue>
#include <algorithm>
//#include "StlFile.h"

using namespace std;

// Private initialization only to be called by the model function
// below.
Model* initModel() {
  Model* points = new btAlignedObjectArray<btVector3>;
  points->push_back(btVector3(8,0,0));
  points->push_back(btVector3(-8,0,0));
  return points;
}

// Our model data is a singleton object in the "Construct on First Use
// Idiom."
const Model& model() {
  static Model* points = initModel();
  return *points;
}

btVector3 readVector(std::istream& s) {
  double x, y, z;
  s.ignore(256, '(');
  s >> x;
  s.ignore(256, ',');
  s >> y;
  s.ignore(256, ',');
  s >> z;
  s.ignore(256, ')');
  return btVector3(x,y,z);
}

ModelTree* readDump(const char* fileName) {
  ifstream f(fileName, ios_base::in);
  int n;
  f >> dec >> n;
  f.ignore(256, '\n');

  // Read in named links
  vector<Joint> joints;
  vector<btVector3> originPoint;
  originPoint.push_back(btVector3(0,0,0));
  for(int i = 0; i < n; i++) {
    string s;
    getline(f, s, ':');
    btVector3 translation = readVector(f);
    btVector3 rpy = readVector(f);
    btVector3 axis = readVector(f);
    f.ignore(256, '\n');
    btTransform t(btQuaternion(rpy.z(), rpy.y(), rpy.x()), translation);
    joints.push_back(Joint(s, t, originPoint, axis, 0.1f, (const char *)NULL));
  }

  // Read in robot model graph arcs
  map<int,vector<int> > arcs;
  while(1) {
    int src, dst;
    f >> src;
    f.ignore(256, '>');
    f >> dst;
    f.ignore(256, '\n');
    arcs[src].push_back(dst);
    if(f.peek() == EOF) break;
  }
  f.close();

  //FIXME: Use shared_ptr?
  queue<pair<int, ModelTree*> > q;
  ModelTree* root = new ModelTree(new Joint(joints[0]));
  for(int i = 0; i < arcs[0].size(); i++) q.push(make_pair(arcs[0][i], root));
  cout << "Building model tree" << endl;
  while(!q.empty()) {
    int n = q.front().first;
    ModelTree* parent = q.front().second;
    q.pop();
    ModelTree* t = new ModelTree(new Joint(joints[n]));
    parent->add_child(t);
    for(int i = 0; i < arcs[n].size(); i++) q.push(make_pair(arcs[n][i], t));
  }
  cout << "Done building model tree" << endl;
  return root;
}

void makeSpheres(ModelTree *t) {
  double spacing = 0.1;
  for(ModelTree::child_iterator it = t->begin(); it!=t->end(); it++) {
    btVector3 loc = (*it)->curr->trans(btVector3(0.0,0.0,0.0));
    btVector3 step = (loc/loc.length())*spacing;
    for(btVector3 sphere; sphere.length() < loc.length(); sphere+=step) {
      t->curr->points.push_back(sphere);
    }
    makeSpheres(*it);
  }
}

ModelTree* initPR2() {
  return readDump("UrdfDumper/etc/pr2.txt");
}

const ModelTree& pr2() {
  static ModelTree* t = initPR2();
  makeSpheres(t);
  return *t;
}

ModelTree* initTestTree() {
  return readDump("UrdfDumper/etc/test.txt");
}

const ModelTree& testTree() {
  static ModelTree* t = initTestTree();
  return *t;
}

// Assign joint angles (angles given in radians).
ModelTree* poseModel(const ModelTree& root, 
                     const map<string,float>& jointAngles) {
  ModelTree* posed = new ModelTree(new Joint(*root.curr));
  queue<pair<ModelTree*, ModelTree*> > q;

  for(ModelTree::child_iterator it = root.begin();
      it != root.end();
      it++) {
    q.push(make_pair(posed, *it));
  }
  
  while(!q.empty()) {
    ModelTree* parent = q.front().first;
    ModelTree* n = q.front().second;
    q.pop();
    Joint* j = new Joint(*n->curr);
    const map<string,float>::const_iterator angle = 
      jointAngles.find(n->curr->name);
    if(angle != jointAngles.end()) {
      j->trans.setRotation(btQuaternion(j->axis, angle->second));
    }
    ModelTree* n2 = new ModelTree(j);
    parent->add_child(n2);

    for(ModelTree::child_iterator it = n->begin(); 
        it != n->end();
        it++) {
      q.push(make_pair(n2, *it));
    }
  }
  return posed;
}

// Posed (e.g. animated) models must be explicitly freed.
void freePosedModel(ModelTree* m) {
  queue<ModelTree*> q;
  q.push(m);
  while(!q.empty()) {
    ModelTree* n = q.front();
    q.pop();
    for(ModelTree::child_iterator it = n->begin();
        it != n->end();
        it++) {
      q.push(*it);
    }
    delete n->curr; // Delete the Joint
    delete n;       // Delete the ModelTree
  }
}

struct SphereCompare {
  bool operator()(CameraSphere a, CameraSphere b) { 
    return (a.center.z() - a.r) > (b.center.z() - b.r);
  }
} sphereCmp;

// Sort all sphere's in a model according to distance from camera.
void painterSort(const ModelTree& root, 
                 const btTransform& camera, 
                 vector<CameraSphere>& v) {
  btVector3 origin(0,0,0);
  queue<pair<ModelTree*, btTransform> > q;

  for(ModelTree::child_iterator it = root.begin();
      it != root.end();
      it++) {
    q.push(make_pair(*it, root.curr->trans));
  }
  v.push_back(CameraSphere(camera(root.curr->trans(origin)), root.curr->radius));
  while(!q.empty()) {
    ModelTree* m = q.front().first;
    btTransform t = q.front().second * m->curr->trans;
    q.pop();
    v.push_back(CameraSphere(camera(t(origin)), m->curr->radius));
    for(int i = 0; i < m->curr->points.size(); i++)
      v.push_back(CameraSphere(camera(t(m->curr->points[i])), 
                               m->curr->radius / 3.0));

    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++)
      q.push(make_pair(*it, t));
  }
  sort(v.begin(), v.end(), sphereCmp);
}
