#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <btBulletDynamicsCommon.h>

using namespace std;

void tokenize(string s, vector<string>& tokens) {
  size_t i = s.find(" ");
  while(i != string::npos) {
    tokens.push_back(s.substr(0, i+1));
    s = s.substr(i+1);
    i = s.find(" ");
  }
  tokens.push_back(s.substr(0));
}

float stof(string& s) {
  stringstream ss(s);
  float f;
  if(ss.peek() == '(') ss.get();
  ss >> f;
  return f;
}

btTransform parsePose(const char* filename) {
  ifstream f(filename);
  vector<string> tokens;
  string s;

  // The pose text file format consists of two lines
  getline(f, s);
  tokenize(s, tokens);
  getline(f,s);
  tokenize(s, tokens);
  if(tokens.size() != 9) {
    cout << "Didn't find 9 space-separated tokens in a pose file!" << endl;
    exit(-1);
  }
  btQuaternion r = btQuaternion(stof(tokens[2]), stof(tokens[3]), 
                                stof(tokens[4]), stof(tokens[1]));
  btVector3 t = btVector3(stof(tokens[6]), stof(tokens[7]), stof(tokens[8]));
  return btTransform(r,t);
}

/*
int main(int argc, char** argv) {
  btTransform t = parsePose("/Users/acowley/Documents/Projects/CollisionDepth/etc/depths1pose.txt");
  btQuaternion q = t.getRotation();
  btVector3 v = t.getOrigin();
  cout << "Quaternion " << q.w() << ", " << q.x() << " " 
       << q.y() << " " << q.z() << endl;
  cout << "V3 " << v.x() << " " << v.y() << " " << v.z() << endl;
  return 0;
}
*/
