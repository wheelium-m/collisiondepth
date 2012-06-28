#include "Model.h"
#include <iostream>
#include <fstream>
#include <map>
#include <queue>

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
    joints.push_back(Joint(s, t, originPoint, 0.1f));
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

ModelTree* initPR2() {
  return readDump("UrdfDumper/etc/pr2.txt");
}

const ModelTree& pr2() {
  static ModelTree* t = initPR2();
  return *t;
}

ModelTree* initTestTree() {
  return readDump("UrdfDumper/etc/test.txt");
}

const ModelTree& testTree() {
  static ModelTree* t = initTestTree();
  return *t;
}
