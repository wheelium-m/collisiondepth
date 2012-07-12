#ifndef SDLBACKEND_H
#define SDLBACKEND_H

#include "Renderer.h"
#include "Model.h"
#include "DepthMap.h"
#include <list>
#include <SDL.h>
#include <btBulletDynamicsCommon.h>
#include <SDL_framerate.h>

typedef std::vector<std::list<std::pair<int,int> > > ScanlineIntervals;

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
  void drawAxis(const btTransform &camera);
  void render(const btTransform& camera);
  void renderModel(const ModelTree& m, const btTransform& camera);
  bool checkSphere(const DepthMap& depth,
                   const btVector3& camSpace, 
                   const btVector3& screenSpace, 
                   const float r,
                   ScanlineIntervals& spans);
  void drawSphere(ScanlineIntervals& spans, const CameraSphere&, DepthMap depth);
  btVector3 cameraToScreen(btVector3 pt);
  void drawDepthMap(DepthMap depth);
};

#endif
