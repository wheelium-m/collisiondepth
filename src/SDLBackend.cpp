#include <stdio.h>
#include "SDLBackend.h"
#include <SDL_gfxPrimitives.h>

SDLBackend::SDLBackend() : m_display(NULL) {}

SDLBackend::~SDLBackend() {
  printf("SDL shutdown\n");
  SDL_Quit();
}

bool SDLBackend::init(int width, int height) {
  if(SDL_Init(SDL_INIT_EVERYTHING) < 0) return false;
  SDL_WM_SetCaption("Collision Depth", "Collision");
  SDL_initFramerate(m_fps);
  SDL_setFramerate(m_fps, 100);
  m_display = SDL_SetVideoMode(width, height, 32, SDL_HWSURFACE | SDL_DOUBLEBUF);
  if(!m_display) return false;
  SDL_FillRect(m_display, NULL, 0);
  return true;
}

void SDLBackend::render(btTransform &camera) {
  // Suppose we just have a point
  btVector3 pt(8,0,0);
  pt = camera(pt); // Transform the point
  Sint16 cx = (Sint16)(pt.x() * 10) + 320;
  Sint16 cy = (Sint16)(pt.y() * 10) + 240;
  filledCircleRGBA(m_display, cx, cy, 5, 255, 0, 0, 255);
  SDL_Flip(m_display);
}

bool SDLBackend::loop(btTransform &camera) {
  SDL_Event ev;
  SDL_PollEvent(&ev);
  if(ev.type == SDL_KEYDOWN) return false;
  render(camera);
  SDL_framerateDelay(m_fps);
  return true;
}

