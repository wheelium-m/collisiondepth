#include <stdio.h>
#include "SDLBackend.h"
#include <SDL_gfxPrimitives.h>
#include "Model.h"

SDLBackend::SDLBackend() : m_display(NULL) {}

/* Cleanup SDL. */
SDLBackend::~SDLBackend() {
  printf("SDL shutdown\n");
  SDL_Quit();
}

inline btVector3 SDLBackend::cameraToScreen(btVector3 pt) {
  // The scaling factor just zooms things in a bit. It is arbitrary.
  static const btVector3 scale = btVector3(10.0,10.0,1);
  btVector3 v = pt * scale + cameraToScreenTranslation;
  return v;
}

inline btVector3 project(btVector3 v) {
  if(v.z() != 0) return v * (1.0 / v.z());
  else return v;
}

/* Open a window, initialize a framerate manager, and clear the back
 * buffer. */
bool SDLBackend::init(int width, int height) {
  if(SDL_Init(SDL_INIT_EVERYTHING) < 0) return false;
  SDL_WM_SetCaption("Collision Depth", "Collision");
  SDL_initFramerate(m_fps);
  SDL_setFramerate(m_fps, 100);
  cameraToScreenTranslation = btVector3((float)width / 2.0, \
                                        (float)height / 2.0, \
                                        0.0);
  m_display = SDL_SetVideoMode(width, height, 32, SDL_HWSURFACE | SDL_DOUBLEBUF);
  if(!m_display) return false;
  SDL_FillRect(m_display, NULL, 0);
  return true;
}

/* Render the model (currently hard coded here) in the given camera's
 * coordinate frame. */
void SDLBackend::render(btTransform &camera) {
  SDL_FillRect(m_display, NULL, 0);
  Model m = model();
  for(int i = 0; i < m.size(); i++) {
    btVector3 pt = m[i];
    pt = cameraToScreen(project(camera(pt))); // Transform the point
    filledCircleRGBA(m_display, (Sint16)pt.x(), (Sint16)pt.y(), 5, \
                     255, 0, 0, 255);
  }
  SDL_Flip(m_display);
}

/* Check UI events, render the current frame, and return true if we
 * should continue looping and false if the user asked to quit. */
bool SDLBackend::loop(btTransform &camera) {
  SDL_Event ev;
  SDL_PollEvent(&ev);
  if(ev.type == SDL_KEYDOWN) return false;
  render(camera);
  SDL_framerateDelay(m_fps);
  return true;
}
