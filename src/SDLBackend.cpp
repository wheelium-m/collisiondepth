#include <stdio.h>
#include "SDLBackend.h"
#include <SDL_gfxPrimitives.h>
#include <queue>
#include <iostream>
#include "DepthMap.h"

using namespace std;

#define ZOOM 180.0
#define PI 3.1415926535

SDLBackend::SDLBackend() : m_display(NULL) {}

/* Cleanup SDL. */
SDLBackend::~SDLBackend() {
  printf("SDL shutdown\n");
  SDL_Quit();
}

// Camera coordinate system to screen space
inline btVector3 SDLBackend::cameraToScreen(btVector3 pt) {
  // The scaling factor just zooms things in a bit. It is arbitrary.
  static const btVector3 scale = btVector3(ZOOM, ZOOM, 1);
  btVector3 v = pt * scale + cameraToScreenTranslation;
  return v;
}

// Perspective projection
inline btVector3 project(btVector3 v) {
  if(v.z() != 0) return v * (1.0 / v.z());
  else return v;
}

/* Draw the axis relative to the camera perspective */
void SDLBackend::drawAxis(const btTransform & camera){
  btVector3 orig(0.0,0.0,0.0);
  btVector3 x(1.0,0.0,0.0);
  btVector3 y(0.0,1.0,0.0);
  btVector3 z(0.0,0.0,1.0);
  orig = cameraToScreen(project(camera(orig)));
  x = cameraToScreen(project(camera(x)));
  y = cameraToScreen(project(camera(y)));
  z = cameraToScreen(project(camera(z)));
  
  lineRGBA(m_display, orig.x(), orig.y(), x.x(), x.y(), 255, 0, 0, 255);

  lineRGBA(m_display, orig.x(), orig.y(), y.x(), y.y(), 0,255,0,255);

  lineRGBA(m_display, orig.x(), orig.y(), z.x(), z.y(), 0,0,255,255);
  //void Draw_Line(SDL_Surface *super,
  //		 Sint16 x1, Sint16 y1, Sint16 x2, Sint16 y2,
  //		 Uint32 color);
  
}

bool SDLBackend::checkSphere(const DepthMap& depth,
                             const btVector3& camSpace, 
                             const btVector3& screenSpace, 
                             const float r) {
  uint8_t *p;
  uint8_t *endOfBuffer = (uint8_t*)m_display->pixels + \
                         m_display->pitch * m_display->h;
  // Perspective projection
  int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * ZOOM) : (int)r;
  if(bound < 1) bound = 1;

  int bpp = m_display->format->BytesPerPixel;
  int bsq = bound * bound; // screen-space radius squared
  float unproject = r / (float)bound;

  for(int y = -bound; y < bound; y++) {
    if(screenSpace.y() + y >= m_display->h || screenSpace.y() + y < 0) break;
    const int rowBound = (int)sqrt(bsq - y*y);
    const int ysq = y * y;
    p = (uint8_t*)m_display->pixels + \
        ((int)(screenSpace.y() - bound) + y) * m_display->pitch + \
        (int)(screenSpace.x() - rowBound)*bpp;
    if(p < m_display->pixels) continue;
    if(p >= endOfBuffer) break;

    for(int x = -rowBound; x < rowBound; x++, p += 4) {
      if(p >= endOfBuffer || screenSpace.x() >= m_display->w) break;
      if(screenSpace.x() + x < 0) continue;
      
      // We know that r^2 = x^2 + y^2 + z^2 in the sphere's coordinate
      // frame.  So we need to go from our screen space x' and y' to
      // sphere x and y. Or do we? Consider the equator of the
      // projected sphere: we know the x' coordinate will vary from
      // -bound to bound, the y' coordinate will be zero, and the z'
      // coordinate will vary from zero to bound and back to zero.  We
      // want a way of computing sphere-space z from screen space x'
      // and y'. We do this by computing z', then undoing the
      // perspective projection.

      float ptDepth = camSpace.z() + sqrt(bsq - x*x - y*y) * unproject;
      if(!depth.collides(screenSpace.x() + x, screenSpace.y() + y, ptDepth)) 
        return false;
    }
  }
  return true;
}

/* Draw a sphere with shading indicating depth. */
void SDLBackend::drawSphere(const btTransform &camera, btVector3 loc, float r) {
  drawAxis(camera);
  static DepthMap depth;
  if(!depth.map)
    depth.makeSimpleMap();
  btVector3 camSpace = camera(loc);
  camSpace = (depth.trans.inverse())(camSpace);
  /*This way, spheres behind the camera will not be drawn*/
  if(camSpace.z()<0.0) return;
  btVector3 pt = cameraToScreen(project(camSpace));

  // FIXME: the screen coordinate system scaling factor hack (ZOOM)
  // appears both here and in cameraToScreen.

  // Perspective projection
  int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * ZOOM) : (int)r;

  if(bound < 1) bound = 1;
  int bpp = m_display->format->BytesPerPixel;
  uint8_t *p;
  int colorScale = 128 / bound;
  int bsq = bound * bound; // radius squared
  bool sphereCollides = checkSphere(depth, camSpace, pt, r);

  // Pixel ordering is ARGB.
  uint8_t *endOfBuffer = (uint8_t*)m_display->pixels + \
                         m_display->pitch * m_display->h;
  for(int y = -bound; y < bound; y++) {
    if(pt.y() + y >= m_display->h || pt.y() + y < 0) break;
    const int rowBound = (int)sqrt(bsq - y*y);
    const int ysq = y * y;
    p = (uint8_t*)m_display->pixels + \
        ((int)(pt.y() - bound) + y) * m_display->pitch + \
        (int)(pt.x() - rowBound)*bpp;
    if(p < m_display->pixels) continue;
    if(p >= endOfBuffer) break;

    for(int x = -rowBound; x < rowBound; x++, p += 4) {
      if(p >= endOfBuffer || pt.x() + x >= m_display->w) break;
      if(pt.x() + x < 0) continue;
      int depthColor = sqrt(bsq - x*x - ysq) * colorScale;
      if(sphereCollides) {      
	p[1] = 0;
	p[2] = (uint8_t)(128 + depthColor < 255 ? 128 + depthColor : 255);
	p[3] = 0;
      }
      else{
	p[1] = (uint8_t)(128 + depthColor < 255 ? 128 + depthColor : 255);
	p[2] = 0;
	p[3] = 0;
      }
    }
  }
}

/* Open a window, initialize a framerate manager, and clear the back
 * buffer. */
bool SDLBackend::init(int width, int height) {
  if(SDL_Init(SDL_INIT_EVERYTHING) < 0) return false;
  SDL_WM_SetCaption("Collision Depth", "Collision");
  //SDL_initFramerate(m_fps);
  //SDL_setFramerate(m_fps, 100);
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
void SDLBackend::render(const btTransform &camera) {
  SDL_FillRect(m_display, NULL, 0);
  Model m = model();
  for(int i = 0; i < m.size(); i++) {
    btVector3 pt = m[i];
    pt = cameraToScreen(project(camera(pt))); // Transform the point
    filledCircleRGBA(m_display, (Sint16)pt.x(), (Sint16)pt.y(), 100, \
                     255, 0, 0, 255);
  }
  drawSphere(camera, btVector3(12,12,0), 4);
  SDL_Flip(m_display);
}



void SDLBackend::renderModel(const ModelTree& root, const btTransform &camera) {
  SDL_FillRect(m_display, NULL, 0);
  btVector3 origin(0,0,0);
  
  drawSphere(camera, root.curr->trans(origin), root.curr->radius);
  queue<pair<ModelTree*, btTransform> > q;
  for(vector<ModelTree*>::const_iterator it = root.begin();
      it != root.end();
      it++) {
    q.push(make_pair(*it, root.curr->trans));
  }
  while(!q.empty()) {
    ModelTree* m = q.front().first;
    btTransform t = q.front().second*m->curr->trans; 
    q.pop();
    drawSphere(camera, t(origin), m->curr->radius);
    for(int i = 0; i < m->curr->points.size();i++){
      drawSphere(camera, t(m->curr->points[i]), m->curr->radius/3.0);
    }
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++)
      q.push(make_pair(*it, t));
  }
  SDL_Flip(m_display);
}

/* Check UI events, render the current frame, and return true if we
 * should continue looping and false if the user asked to quit. */
bool SDLBackend::loop(btTransform &camera) {
  SDL_Event keyevent;    //The SDL event that we will poll to get events.
  static int moveFigure=0;
  static int moveLeft = 0;
  static int moveRight = 0;
  static int moveIn = 0;
  static int moveOut = 0;
  while (SDL_PollEvent(&keyevent))   //Poll our SDL key event for any keystrokes.
    {
      
      switch(keyevent.type){
	
      case SDL_KEYDOWN:
	switch(keyevent.key.keysym.sym){
        case SDLK_LEFT:
	  //camera=camera*(btTransform(btQuaternion(btVector3(0.0,0.0,1.0), 1.0*PI/18.0)));
	  
          moveLeft=1;
          break;
        case SDLK_RIGHT:
          //camera=camera*(btTransform(btQuaternion(btVector3(0.0,0.0,1.0), -1.0*PI/18.0)));
          moveRight=1;
	  break;
        case SDLK_UP:
          //camera=camera*(btTransform(btQuaternion(btVector3(0.0,1.0,0.0), PI/18.0)));
          moveIn=1;
	  break;
        case SDLK_DOWN:
          //camera=camera*(btTransform(btQuaternion(btVector3(0.0,1.0,0.0), -1.0*PI/18.0)));
          moveOut=1;
	  break;
	default:
          break;
	}
	break;
	
      case SDL_KEYUP:
	switch(keyevent.key.keysym.sym){
        case SDLK_LEFT:
	  
          moveLeft = 0;
          break;
        case SDLK_RIGHT:
          moveRight = 0;
          break;
        case SDLK_UP:
          moveIn = 0;
          break;
        case SDLK_DOWN:
          moveOut = 0;
          break;
        default:
          break;
	}
	break;
      case SDL_MOUSEMOTION:
	if(moveFigure==1){
	  
	  btVector3 motion(keyevent.motion.xrel, keyevent.motion.yrel,0.0);
	  btTransform rot(btQuaternion(btVector3(0.0,0.0,1.0), -1.0*PI/2.0), btVector3(0.0,0.0,0.0));
	  motion=rot(motion);
	  rot.setRotation(btQuaternion(motion, motion.length()/50.0));
	  camera=btTransform(btQuaternion::getIdentity(), camera.getOrigin())*(rot*btTransform(camera.getRotation(), btVector3(0.0,0.0,0.0)));
	}
	//printf("The motions are: %d, %d\n",keyevent.motion.xrel, keyevent.motion.yrel);
	//keyevent.motion.x, keyevent.motion.y
	break;
      case SDL_MOUSEBUTTONDOWN:
	
	moveFigure=1;
	break;
      case SDL_MOUSEBUTTONUP:
	
	moveFigure=0;
	break;

      }
    }
  
  if(keyevent.key.keysym.sym==SDLK_ESCAPE){
    return false;
  }
  
  if(moveLeft==1){
    
    camera.setOrigin(camera.getOrigin()+btVector3(0.01,0.0,0.0));
  }
  
  if(moveRight==1){
    camera.setOrigin(camera.getOrigin()+btVector3(-0.01,0.0,0.0));
  }
  
  if(moveIn==1){
    camera.setOrigin(camera.getOrigin()+btVector3(0.0,0.0,-0.01));
  }
  
  if(moveOut==1){
    camera.setOrigin(camera.getOrigin()+btVector3(0.0,0.0,0.01));
  }
  
  renderModel(pr2(), camera);
  //SDL_framerateDelay(m_fps);
  return true;
}
