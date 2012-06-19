#include "Model.h"
#include <iostream>
#include <SDL.h>
#include <SDL_gfxPrimitives.h>
#include <unistd.h>

#define DEG2RAD(x)(x*3.141592/180.0)

using namespace std;

void rotate(Model *m, int i){
  
  btTransform center(btQuaternion::getIdentity(), -1*m->points[i]);
  
  btTransform rotate(
		     btQuaternion(btVector3(0.0,0.0,1.0),
				  DEG2RAD(m->angles_per_sec[i])),
		     btVector3(0.0,0.0,0.0));
  
  btTransform undo(btQuaternion::getIdentity(), m->points[i]);
  
  for(; i < 3; i++){
    m->points[i] = undo(rotate(center(m->points[i])));
  }
  
}

int main(){
  Model m;
  SDL_Surface * surf;
  SDL_Init(SDL_INIT_EVERYTHING);
  surf = SDL_SetVideoMode(640, 480, 32, SDL_HWSURFACE|SDL_DOUBLEBUF);
  btTransform cam2screen(btQuaternion::getIdentity(), btVector3(320, 240, 0));
  Model n;
  for(int i = 0; i < 100; i++){
    rotate(&m, 0);
    rotate(&m, 1);
    
    for(int j = 0; j < 3; j++)
      n.points[j] = cam2screen(m.points[j]*100);
    
    SDL_FillRect(surf, NULL, 0);
    lineRGBA(surf, (Sint16)((n.points[0])[0]), (Sint16)((n.points[0])[1]), (Sint16)((n.points[1])[0]), (Sint16)((n.points[1])[1]), 255, 0, 0, 255);
    lineRGBA(surf, (n.points[1])[0], (n.points[1])[1], (n.points[2])[0], (n.points[2])[1], 255, 0, 0, 255);
    
  
      
    // cout<<endl;
  
    SDL_Flip(surf);
    usleep(30000);
  }

  return 0;
}

