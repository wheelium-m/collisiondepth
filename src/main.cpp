#include <iostream>
#include <btBulletDynamicsCommon.h>
#include "SDLBackend.h"

using namespace std;

void go(Renderer& renderer) {
  if(!renderer.init(640,480)) {
    cout << "Error initializing renderer" << endl;
    return;
  }

  btTransform cam(btQuaternion::getIdentity());
  while(renderer.loop(cam)) {
    cam *= btTransform(btQuaternion(btVector3(0,0,1), btScalar(2)));
  }
}

void goSDL() {
  SDLBackend sdl;
  go(sdl);
}

int main(int argc, char** argv) {
  cout << "Starting..." << endl;
  goSDL();
  cout << "Ending." << endl;
  return 0;
}
