#include "Joint.h"
#include "Model.h"
#include <btBulletDynamicsCommon.h>
#include <iostream>
#define PI 3.14159265358979323

using namespace std;

int main(){
  btTransform trans(btQuaternion(btVector3(0.0,0.0,1.0),0.0), btVector3(0.0,0.0,0.0));
  btVector3 pts[]= {btVector3(0.0,0.0,0.0), btVector3(1.0,0.0,0.0)};
  int num = 2;
  float rad = .5;
  Joint j(trans, pts, num, rad);
  Model m(&j, 1);
  btVector3 v = m.joints[0].trans*m.joints[0].points[1];
  cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<endl;
  btQuaternion q(btVector3(0.0,0.0,1.0),PI/2.0);
  m.internalTransform(&q, 1);
  v = m.joints[0].trans*m.joints[0].points[1];
  cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<endl;
  return 0;
}
