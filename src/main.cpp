#include <iostream>
#include <btBulletDynamicsCommon.h>
#include "SDLBackend.h"

#define DEG2RAD(x) (x*3.14159/180.0)

using namespace std;

extern btTransform parsePose(const char*);

/* Runs the application loop. */
void go(Renderer& renderer) {
  if(!renderer.init(640,480)) {
    cout << "Error initializing renderer" << endl;
    return;
  }

  //btTransform robotFrame(btQuaternion::getIdentity(), btVector3(0,0,3));

  // This is a candidate starting location for the robot
  // Quaternion (-0.16029344) (-0.9626429) 3.5844795e-2 0.21526611
  // V3 12.614223 3.6128845 2.0635848

  // Note that the LAB.pcd data set doesn't extend up exactly
  // perpendicularly to the XZ plane as we might hope, and has a Y
  // bias of 2-2.5m. Hence this funny starting location Y coordinate.
  btTransform robotFrame = btTransform(btQuaternion(btVector3(1,0,0),-3.14159*0.5), 
                                       btVector3(13.614223,-2.6128845,3.0635848));

  float zVel = 0.05;
  while(renderer.loop(robotFrame)) {
    robotFrame.setOrigin(robotFrame.getOrigin() + btVector3(0,0,zVel));
    if(robotFrame.getOrigin().getZ() > 6) {
      if(zVel > 0) zVel = -zVel;
    } else if(robotFrame.getOrigin().getZ() < 1) {
      if(zVel < 0) zVel = -zVel;
    }
  }
}

void goSDL() {
  // Allocate and initialize the SDL renderer.
  SDLBackend sdl;
  go(sdl);
}

int main(int argc, char** argv) {
  cout << "Starting..." << endl;
  goSDL();
  cout << "Ending." << endl;
  return 0;
}
