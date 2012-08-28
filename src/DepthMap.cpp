#include "DepthMap.h"
#include <fstream>
#include <iostream>
#include <stdint.h>
using namespace std;

void DepthMap::setRawMap(float* map) {
  map_it it = dilatedMaps.find(0);
  if(it != dilatedMaps.end()) delete (*it).second;
  dilatedMaps[0] = map;
}

void DepthMap::makeSimpleMap(const float focalLength) {
  width = 640;
  height = 480;
  this->focalLength = focalLength;
  trans.setIdentity();
  transInv.setIdentity();
    
  float* map = new float[640*480];
  for(int i = 0; i < (640*480); i++){
    *(map+i)=10.0;
  }
  for(int i = 120; i < 360; i++){
    for(int j = 160; j < 480; j++){
      *(map+i*640+j)=1.5;
    }
  }
  setRawMap(map);
}

void DepthMap::getKinectMapFromFile(const float focalLength, const char * filename) {
  ifstream file(filename, ios::in|ios::binary);
  width = 640;
  height = 480;
  this->focalLength = focalLength;
  trans.setIdentity();
  transInv.setIdentity();
  float* map = new float[640*480];
  file.read((char *)map, 640*480*4);
  // for(int i = 0; i < 640*480; i++)
  //   if(map[i] == 0.0f) map[i] = 12.0784;
  //if(map[i] == 0.0f) map[i] = 3156.33f;
  setRawMap(map);
}

void DepthMap::addDilation(const float r) {
  map_it it = dilatedMaps.find(r);
  if(it != dilatedMaps.end()) delete (*it).second;
  dilatedMaps[r] = bloomDepths(dilatedMaps[0], r);
}

// We want focalLength in pixels, and sphere radius in meters.
float* DepthMap::bloomDepths(const float* old, 
                             const float r) {
  float* dilated = new float[width*height];

  // Default depth is set to 3156.33m (69)
  // Default depth is set to 12.0784m (65)
  // 66 -> 48.5647m
  // 67 -> 195.263
  //memset((uint8_t*)dilated, 65, sizeof(float)*width*height);
  //memset((uint8_t*)dilated, 66, sizeof(float)*width*height);
  memset((uint8_t*)dilated, 0, sizeof(float)*width*height);
  const float rFocalLength = focalLength * r;

  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++, old++) {
      float dist = *old;
      //if(dist == 0.0f) continue; // Don't splat unknown points
      
      // Don't splat unknown points or those very close to the camera.
      if(dist < 0.2f) continue; 

      // Pixel radius
      const int pr = (int)(rFocalLength / dist);
      const int prsq = pr * pr;

      // Splat a projection of a sphere centered at the point of
      // interest onto the new depth map to find a conservative
      // minimum depth.
      for(int cy = -pr, rowOffset=(y-pr)*width; 
          cy < pr; 
          cy++, rowOffset += width) {
        if(cy + y >= height) break;
        if(cy + y < 0) continue;
        const int rowBound = (int)sqrt(prsq - cy*cy);
        int left = -rowBound;
        if(left + x >= width) continue;
        if(left + x < 0) left = -x;

        int right = rowBound;
        if(right + x < 0) continue;
        if(right + x >= width) right = width - 1 - x;

        for(int cx = left, colOffset = left+x; cx < right; cx++, colOffset++) {
          float* const dOld = &(dilated[rowOffset + colOffset]);
          if(dist < *dOld || *dOld == 0) *dOld = dist;
        }
      }
    }
  }
/*
  // Finish with a cleanup pass that sets 0-depth points to "infinite"
  // depth. This is to avoid holes in the depthmap from creating
  // collisions. FIXME: this is not conservative at all!
  float *p = dilated;
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++, p++) {
      if(*p == 0.0f) *p = 10.0f;
    }
  }
*/
  return dilated;
}
