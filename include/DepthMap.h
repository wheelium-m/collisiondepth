#ifndef DEPTHMAP_H
#define DEPTHMAP_H
#include <btBulletDynamicsCommon.h>
/* #include <stdlib.h> */
/* #include <unistd.h> */
/* #include <iostream> */
/* #include <fstream> */

/*For now, depth maps are just 480x640 arrays of floats. I like the
 * idea of using floats because it will be easier to use as part of
# * OpenGL's floating point texture format. */
class DepthMap{
public:
  int width, height;
  float * map;
  /* Specifies the camera transformation. */
  btTransform trans;
  DepthMap() 
    : width(0), height(0), map(NULL), trans(btTransform::getIdentity()) {};
  DepthMap(int x, int y, float * m, btTransform t)
    : width(x), height(y), map(m), trans(t) {};
  DepthMap(const DepthMap &d)
    : width(d.width), height(d.height), map(d.map), trans(d.trans) {};

  // Returns true if there is a collision; false otherwise.
  
  inline bool collides(int x, int y, float sphereDepth) const {
    if(*(map+(y*width)+x) >sphereDepth)
      return true;
    else return false;
  }

  void makeSimpleMap();
  void getKinectMapFromFile(const char * filename);
  DepthMap* bloomDepths(const float focalLength, const float r);

};

#endif

