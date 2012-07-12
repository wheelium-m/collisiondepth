#include <iostream>
#include <btBulletDynamicsCommon.h>
#include "SDLBackend.h"

#define DEG2RAD(x) (x*3.14159/180.0)

using namespace std;

/* Runs the application loop. */
void go(Renderer& renderer) {
  if(!renderer.init(640,480)) {
    cout << "Error initializing renderer" << endl;
    return;
  }

  btTransform cam(btQuaternion::getIdentity(), btVector3(0,0,3));

  // Z-axis in new coordinate frame.
  btVector3 spinAxis = btTransform(cam.getRotation())(btVector3(0,0,1));

  // Rotate the camera about the Z-axis by 2 degrees-per-frame
  while(renderer.loop(cam)) {
    //cam = btTransform(btQuaternion(spinAxis, btScalar(DEG2RAD(2)))) * cam;
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
