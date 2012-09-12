#include <iostream>
#include <btBulletDynamicsCommon.h>
#include "SDLBackend.h"

#define DEG2RAD(x) (x*3.14159/180.0)

using namespace std;

extern btTransform parsePose(const char*);

// Load depth map data (an array of floating point depths and the
// camera pose from which the view was captured) for the collision
// environment under consideration.
void loadEnvironment(Renderer& renderer) {
  // renderer.addDepthMap("etc/depths1.bin", "etc/depths1pose.txt");
  // renderer.addDepthMap("etc/depths2.bin", "etc/depths2pose.txt");
  // renderer.addDepthMap("etc/depths3.bin", "etc/depths3pose.txt");
  // renderer.addDepthMap("etc/depths4.bin", "etc/depths4pose.txt");
}

/* Runs the application loop. */
void go(Renderer& renderer) {
  if(!renderer.init(640,480)) {
    cout << "Error initializing renderer" << endl;
    return;
  }

  loadEnvironment(renderer);

  //btTransform robotFrame(btQuaternion::getIdentity(), btVector3(0,0,3));

  // This is a candidate starting location for the robot
  // Quaternion (-0.16029344) (-0.9626429) 3.5844795e-2 0.21526611
  // V3 12.614223 3.6128845 2.0635848

  // Note that the LAB.pcd data set doesn't extend up exactly
  // perpendicularly to the XZ plane as we might hope, and has a Y
  // bias of 2-2.5m. Hence this funny starting location Y coordinate.
  btQuaternion rot(btVector3(1,0,0),3.14159*0.5);
  rot = btQuaternion(btTransform(rot)(btVector3(0,0,1)), 3.14159*0.5) * rot;
  btVector3 trans(13.614223,4.6128845,3.0635848);
  btTransform robotFrame = btTransform(rot,trans);

  while(renderer.loop(robotFrame)) {
    btVector3 t = robotFrame.getOrigin();
    cout << "Robot at (" << t.x() << ", " << t.y() << ", ";
    cout << t.z() << ")" << endl;
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
