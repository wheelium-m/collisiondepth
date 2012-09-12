#include "DepthMap.h"
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <sys/time.h>
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
  struct timeval start, stop;
  gettimeofday(&start,NULL);
  dilatedMaps[r] = bloomDepths2(dilatedMaps[0], r);
  gettimeofday(&stop,NULL);
  float seconds = (stop.tv_sec - start.tv_sec) + 0.000001f * (float)(stop.tv_usec - start.tv_usec);
  cout << "Dilation took " << seconds*1000.0f << "ms" << endl;
}

void DepthMap::initCache(int r) {
  dilationCache.resize(r*r*4);
  dilationCacheStride = 2*r;
  const int stride = r*2;
  const int rsq = r*r;
  const float ir = 1.0f / (float)r;
  for(int y = 0; y < stride; y++) {
    const int ysq = (y-r)*(y-r);
    const int tmp = rsq - ysq;
    const int bound = (int)sqrt(tmp);
    const int offset = y*stride+r;
    for(int x = -bound; x < bound; x++) {
      dilationCache[offset+x] = sqrt(tmp - x*x) * ir;
    }
  }
}

// This version of the depth map dilation technique more properly
// implements the idea of a sphere centered at an observed point. It
// makes use of a cached rendering of a unit sphere to avoid
// recomputing the depth for each pixel in each sphere rendering.
float* DepthMap::bloomDepths2(const float* old,
                              const float r) {
  // Denormalize the dilation cache
  for(int i = 0; i < dilationCache.size(); i++) dilationCache[i] *= r;

  float* dilated = new float[width*height];
  memset((uint8_t*)dilated, 0, sizeof(float)*width*height);
  const float rFocalLength = focalLength * r;
  const float stridef = (float)dilationCacheStride;
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++, old++) {
      float dist = *old;
      // Don't splat unknown points or those very close to the camera.
      if(dist < 0.2f) continue;
      const int pr = (int)(rFocalLength / dist); // Pixel radius
      int w = pr*2; // width of this sphere rendering
      float step = stridef / (float)w;
      int rowOffset = (y - pr) * width;
      const int prsq = pr*pr;
      float iy = 0.0f;
      for(int cy = 0; cy < w; cy++, rowOffset += width, iy += step) {
        const int yTmp = cy - pr;
        if(y + yTmp >= height) break;
        if(y + yTmp < 0) continue;
        
        const int rowBound = (int)sqrt(prsq - yTmp*yTmp);
        int left = -rowBound;
        if(left + x >= width) continue;
        if(left + x < 0) left = -x;

        int right = rowBound;
        if(right + x < 0) continue;
        if(right + x >= width) right = width - 1 - x;

        int offset = left+x+rowOffset;
        int stopcol = right+x+rowOffset;
        float i = (int)iy*dilationCacheStride + (float)(left + pr)*step;
        //for(int cx = left; cx < right; cx++, i += step, offset++) {
        for(; offset < stopcol; i += step, offset++) {
          const float dnew = dist - dilationCache[(int)i];//*r;
          const float dilatedDepth = dilated[offset];
          if(dilatedDepth == 0 || dnew < dilatedDepth) dilated[offset] = dnew;
        }
      }
    }
  }
  // Renormalize the dilation cache
  for(int i = 0; i < dilationCache.size(); i++) dilationCache[i] /= r;

  return dilated;
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
      float dist = *old - r; // Pad observed geometry by r
      
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
  return dilated;
}
