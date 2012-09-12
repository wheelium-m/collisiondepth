#ifndef DEPTHMAP_H
#define DEPTHMAP_H
#include <btBulletDynamicsCommon.h>
#include <map>
#include <vector>
/* #include <stdlib.h> */
/* #include <unistd.h> */
/* #include <iostream> */
/* #include <fstream> */

/*For now, depth maps are just 480x640 arrays of floats. I like the
 * idea of using floats because it will be easier to use as part of
# * OpenGL's floating point texture format. */
class DepthMap{
private:
  // We maintain a dictionary of depth maps indexed by a 3D dilation
  // radius. A radius of zero corresponds to the raw depth map.
  std::map<float, float*> dilatedMaps;
  typedef std::map<float,float*>::const_iterator map_it;
  void setRawMap(float*);
  float* bloomDepths(const float* map, const float r);
  float* bloomDepths2(const float* map, const float r);

  // We cache a normalized high resolution sphere rendering to use as
  // a reference during dilation.
  std::vector<float> dilationCache;
  int dilationCacheStride;
  void initCache(int r);

public:
  int width, height, focalLength;

  int depthID;

  /* Specifies the camera transformation. */
  btTransform trans;
  btTransform transInv;
  DepthMap() 
    : dilatedMaps(std::map<float,float*>()), width(0), height(0), 
    trans(btTransform::getIdentity()), transInv(btTransform::getIdentity()) {
    initCache(64);
  };
  /* DepthMap(int x, int y, float * m, btTransform t) */
  /*   : width(x), height(y), map(m), trans(t) {}; */
  DepthMap(const DepthMap &d)
    : dilatedMaps(d.dilatedMaps), width(d.width), height(d.height), 
    trans(d.trans), transInv(d.transInv) {
    initCache(64);
  };

  // Returns true if there is a collision; false otherwise.
  
  inline bool collides(const float* map, int x, int y, float sphereDepth) const {
    return *(map+(y*width)+x) >sphereDepth;
  }

  void makeSimpleMap(const float focalLength);
  void getKinectMapFromFile(const float focalLength, const char * filename);
  void addDilation(const float r);
  inline const float* getMap(const float r) const { 
    map_it it = dilatedMaps.find(r);
    if(it == dilatedMaps.end()) return (*dilatedMaps.begin()).second; //NULL;
    else return (*it).second;
  }
  inline const float* getRawMap() { return getMap(0.0f); }
};

#endif

