#ifndef SDLBACKEND_H
#define SDLBACKEND_H

#include "Renderer.h"
#include <SDL.h>
#include <btBulletDynamicsCommon.h>
#include <SDL_framerate.h>

class SDLBackend : public Renderer {
public:
  SDLBackend();
  ~SDLBackend();
  virtual bool init(int width, int height);
  virtual bool loop(btTransform &camera);
private:
  btVector3 cameraToScreenTranslation;
  SDL_Surface* m_display;
  FPSmanager* m_fps;
  void render(btTransform &camera);
  btVector3 cameraToScreen(btVector3 pt);
};

#endif
