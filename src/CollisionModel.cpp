#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "CollisionModel.h"
#include <cstdlib>

using namespace std;

istream& operator >>(istream& ins, CollisionSphere& c) {
  ins.ignore(1);
  ins >> c.x;
  ins.ignore(1);
  ins >> c.y;
  ins.ignore(1);
  ins >> c.z;
  ins.ignore(1);
  ins >> c.r;
  ins.ignore(1);
  return ins;
}

ostream& operator <<(ostream& os, CollisionSphere& c) {
  os << "Sphere at (" << c.x << ", " << c.y << ", " << c.z << "), ";
  os << "r = " << c.r << endl;
  return os;
}

CollisionGeometry* pr2CollisionGeometry(const char* filename) {
  ifstream f(filename, ios_base::in);
  if(!f.is_open()) {
    cout << "Couldn't open collision model file: " << filename << endl;
    exit(-1);
  }
  CollisionGeometry* m = new CollisionGeometry();
  while(!f.eof()) {
    string s;
    getline(f, s);
    size_t i = s.find(" ");
    const string name = s.substr(0,i);
    s = s.substr(i+1);
    int n;
    stringstream(s) >> n;
    for(; n > 0; n--) {
      getline(f,s);
      stringstream sstream(s);
      CollisionSphere c;
      sstream >> c;
      (*m)[name].push_back(c);
    }
    // stringstream sstream(s);
    // while(!sstream.eof()) {
    //   CollisionSphere c;
    //   sstream >> c;
    //   (*m)[name].push_back(c);
    // }
  }
  return m;
}

/*
int main(int argc, char** argv) {
  CollisionGeometry* m = pr2CollisionGeometry("../BenYaml/etc/pr2_body.txt");
  cout << "Spheres attached to " << m->size() << " frames" << endl;
  int sum = 0;
  for(CollisionGeometry::const_iterator it = m->begin();
      it != m->end();
      it++) {
    sum += (*it).second.size();
  }
  cout << "Using " << sum << " spheres." << endl;
  return 0;
}
*/
