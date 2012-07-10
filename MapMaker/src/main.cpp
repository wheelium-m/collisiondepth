#include "MapMaker.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
  cout << "Starting..." << endl;
  MapMaker m;
  for(int i = 0; i < 30; i++)
   m.grabFrame();
  cout << "Ending." << endl;
  return 0;
}
