#ifndef DEPTHMAP_H
#define DEPTHMAP_H
#include <btBulletDynamicsCommon.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

using namespace std;

/*For now, depth maps are just 480x640 arrays of floats. I like the
 * idea of using floats because it will be easier to use as part of
 * OpenGL's floating point texture format. */
class DepthMap{
public:
  int width, height;
  float * map;
  /* Specifies the camera transformation. */
  btTransform trans;
  DepthMap() 
    : width(0), height(0), map(NULL), trans(*(new btTransform())) {};
  DepthMap(int x, int y, float * m, btTransform t)
    : width(x), height(y), map(m), trans(t) {};
  DepthMap(const DepthMap &d)
    : width(d.width), height(d.height), map(d.map), trans(d.trans) {};
  
  // Returns true if there is no collision; false otherwise.
  // FIXME: Will: is that description right? It seems backwards...
  bool collides(int x, int y, float sphereDepth) const {
    if(*(map+(y*width)+x) >sphereDepth)
      return false;
    else return true;
  };
  
  void makeSimpleMap() {
    width = 640;
    height = 480;
    trans = btTransform::getIdentity();
    
    map = (float *)malloc(640*480*sizeof(float));
    for(int i = 0; i < (640*480); i++){
      *(map+i)=1.5;
    }
  }
};

#endif

