#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;

vector<float> hiRes;
int hiResStride;

void init(int r) {
  hiRes.resize(r*r*4);
  hiResStride = 2*r;
  const int stride = r*2;
  const int rsq = r*r;
  const float ir = 1.0f / (float)r;
  for(int y = 0; y < stride; y++) {
    const int ysq = (y-r)*(y-r);
    const int tmp = rsq - ysq;
    const int bound = (int)sqrt(tmp);
    const int offset = y*stride+r;
    for(int x = -bound; x < bound; x++) {
      hiRes[offset+x] = sqrt(tmp - x*x) * ir;
    }
  }
}

void savePGM(const char* filename, int width,
             const vector<float>& pixels) {
  ofstream f(filename);
  f << "P5" << endl << width << " " << width << " " << "255" << endl;
  int n = width*width;
  float step = (float)hiResStride / (float)width;
  for(int y = 0; y < width; y++) {
    float i = (int)(y*step)*hiResStride;
    for(int x = 0; x < width; x++, i+=step) {
      f.put((int)(pixels[(int)i]*255.0f));
    }
  }
}

int main(int argc, char** argv) {
  init(128);
  savePGM("test32.pgm", 53, hiRes);
  return 0;
}
