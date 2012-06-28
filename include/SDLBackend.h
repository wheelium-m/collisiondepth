#ifndef SDLBACKEND_H
#define SDLBACKEND_H

#include "Renderer.h"
#include "Model.h"
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
  void render(const btTransform& camera);
  void renderModel(const ModelTree& m, const btTransform& camera);
  void drawSphere(const btTransform& camera, btVector3 loc, float r);
  btVector3 cameraToScreen(btVector3 pt);
};

#endif
