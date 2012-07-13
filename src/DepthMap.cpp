#include "DepthMap.h"
#include <fstream>
#include <iostream>
using namespace std;

void DepthMap::makeSimpleMap() {
  width = 640;
  height = 480;
  trans = btTransform::getIdentity();
    
  map = (float *)malloc(640*480*sizeof(float));
  for(int i = 0; i < (640*480); i++){
    *(map+i)=10.0;
  }
  for(int i = 120; i < 360; i++){
    for(int j = 160; j < 480; j++){
      *(map+i*640+j)=1.5;
    }
  }
}

void DepthMap::getKinectMapFromFile(const char * filename) {
  ifstream file(filename, ios::in|ios::binary);
  width = 640;
  height = 480;
  trans = btTransform::getIdentity();
  map = (float *)malloc(640*480*4);
  file.read((char *)map, 640*480*4);
}

// We want focalLength in pixels, and sphere radius in meters.
DepthMap* DepthMap::bloomDepths(const float focalLength, const float r) {
  DepthMap* d = new DepthMap(width, height, NULL, trans);
  float* old = map;
  d->map = new float[width*height];

  // Default depth is set to 12.0784m
  memset((uint8_t*)d->map, 65, sizeof(float)*width*height);

  const float rFocalLength = focalLength * r;

  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++, old++) {
      float dist = *old;
      if(dist == 0.0f) continue; // Don't splat unknown points

      // Pixel radius
      const int pr = (int)(rFocalLength / dist);
      const int prsq = pr * pr;

      // Now sample from a projection of a sphere centered at the
      // point of interest onto the existing depth map to find the
      // minimum depth.
      for(int cy = -pr, rowOffset=(y-pr)*width; cy < pr; cy++, rowOffset += width) {
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
          float* const dOld = &d->map[rowOffset + colOffset];
          if(dist < *dOld) *dOld = dist;
        }
      }
    }
  }

  return d;
}
