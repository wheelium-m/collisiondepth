#include <stdio.h>
#include "SDLBackend.h"
#include <SDL_gfxPrimitives.h>
#include <queue>
#include <iostream>
#include "DepthMap.h"
#include "Intervals.h"
#include <algorithm>

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
                             const float r,
                             vector<list<pair<int,int> > >& spans) {
  // Perspective projection
  int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * ZOOM) : (int)r;
  if(bound < 1) bound = 1;

  int bsq = bound * bound; // screen-space radius squared
  float unproject = r / (float)bound;

  const int screenX = (int)screenSpace.x();
  const int screenY = (int)screenSpace.y();

  // FIXME: These are ambigous cases where the geometry is
  // off-screen. We need a three valued type rather than bool to
  // support this.
  if(screenY - bound >= m_display->h) return true;
  if(screenY + bound < 0) return true;

  for(int y = -bound; y < bound; y++) {
    if(screenY + y >= m_display->h) break;
    if(screenY + y < 0) continue;

    const int rowBound = (int)sqrt(bsq - y*y);
    const int ysq = y * y;
    int rowIndex = screenY + y;
    list<pair<int,int> >* row = &spans[rowIndex];
    listCIt rowCurr = row->begin();
    listCIt rowEnd = row->end();

    int left = -rowBound;
    if(left + screenX >= m_display->w) continue;
    if(screenX + left < 0) left = -screenX;

    int right = rowBound;
    if(right + screenX < 0) continue;
    if(screenX + right >= m_display->w) 
      right = m_display->w - 1 - screenX;

    if(left > right) continue;

    for(int x = left; x < right;) {
      // We know that r^2 = x^2 + y^2 + z^2 in the sphere's coordinate
      // frame.  So we need to go from our screen space x' and y' to
      // sphere x and y. Or do we? Consider the equator of the
      // projected sphere: we know the x' coordinate will vary from
      // -bound to bound, the y' coordinate will be zero, and the z'
      // coordinate will vary from zero to bound and back to zero.  We
      // want a way of computing sphere-space z from screen space x'
      // and y'. We do this by computing z', then undoing the
      // perspective projection.

      float ptDepth = camSpace.z() + sqrt(bsq - x*x - ysq) * unproject;
      if(!depth.collides(screenX + x, rowIndex, ptDepth)) return false;
      x = nextFreeIndex(rowEnd, rowCurr, x + screenX) - screenX;
    }
    //addInterval(screenSpace.x() + left, screenSpace.x() + right, row);
  }
  return true;
}

/* Draw a sphere with shading indicating depth. */
void SDLBackend::drawSphere(vector<list<pair<int,int> > >& rowIntervals, 
                            const CameraSphere& sphere, DepthMap depth) {
  
  btVector3 camSpace = sphere.center;
  float r = sphere.r;
  camSpace = (depth.trans.inverse())(camSpace);
  /* This way, spheres behind the camera will not be drawn. */
  if(camSpace.z() - r < 0.0) return;
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
  bool sphereCollides = checkSphere(depth, camSpace, pt, r, rowIntervals);

  const int screenX = (int)pt.x();
  const int screenY = (int)pt.y();

  // Geometry is completely off-screen
  if(screenY - bound >= m_display->h) return;
  if(screenY + bound < 0) return;

  // Pixel ordering is ARGB.
  for(int y = -bound; y < bound; y++) {
    if(screenY + y >= m_display->h) break;
    if(screenY + y < 0) continue;
    const int rowBound = (int)sqrt(bsq - y*y);
    const int ysq = y * y;
    list<pair<int,int> >* row = &rowIntervals[screenY + y];
    listCIt rowCurr = row->begin();
    listCIt rowEnd = row->end();

    int left = -rowBound;
    if(left + screenX >= m_display->w) continue;
    if(screenX + left < 0) left = -screenX;

    int right = rowBound;
    if(right + screenX < 0) continue;
    if(screenX + right >= m_display->w) right = m_display->w - 1 - screenX;

    if(left > right) continue;

    p = (uint8_t*)m_display->pixels + \
        (screenY + y) * m_display->pitch + \
        (screenX + left)*bpp;

    for(int x = left; x < right;) {
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

      int nxt = nextFreeIndex(rowEnd, rowCurr, x + screenX) - screenX;
      p += 4 * (nxt - x);
      x = nxt;
    }
    // Note that this means we only render backfaces which is good for
    // depth testing, but bad for visualization.
    //addInterval(screenX + left, screenX + right, row);
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

/* Render the model in the given camera's coordinate frame. */
void SDLBackend::render(const btTransform &camera) {
  SDL_FillRect(m_display, NULL, 0);
  Model m = model();
  for(int i = 0; i < m.size(); i++) {
    btVector3 pt = m[i];
    pt = cameraToScreen(project(camera(pt))); // Transform the point
    filledCircleRGBA(m_display, (Sint16)pt.x(), (Sint16)pt.y(), 100, \
                     255, 0, 0, 255);
  }
  ScanlineIntervals dummy(m_display->h);
  static DepthMap depth;
  if(!depth.map){
    //depth.getKinectMapFromFile("depth_texture.bin");
    depth.makeSimpleMap();
  }
  drawSphere(dummy, CameraSphere(camera(btVector3(12,12,0)), 4), depth);
  SDL_Flip(m_display);
}

void SDLBackend::renderModel(const ModelTree& rawRoot, const btTransform &camera) {
  SDL_FillRect(m_display, NULL, 0);
  btVector3 origin(0,0,0);

  map<string,float> posture;
  // Lift the PR2's left arm by 45 degrees and swing the right arm out.
  posture["l_shoulder_lift_link"] = -45.0f * PI / 180;
  posture["r_shoulder_pan_link"] = -45.0f * PI / 180;
  ModelTree* root = poseModel(rawRoot, posture);

  vector<CameraSphere> spheres;
  painterSort(*root, camera, spheres);

  vector<list<pair<int,int> > > rowIntervals(m_display->h);
  
  static DepthMap depth;
  if(!depth.map){
    //depth.getKinectMapFromFile("depth_texture.bin");
    depth.makeSimpleMap();
  }

  drawDepthMap(depth);

  for(int i = 0; i < spheres.size(); i++) drawSphere(rowIntervals, spheres[i], depth);

/*
  drawSphere(CameraSphere(camera(root->curr->trans(origin)), root->curr->radius));
  queue<pair<ModelTree*, btTransform> > q;
  for(vector<ModelTree*>::const_iterator it = root->begin();
      it != root->end();
      it++) {
    q.push(make_pair(*it, root->curr->trans));
  }
  while(!q.empty()) {
    ModelTree* m = q.front().first;
    btTransform t = q.front().second*m->curr->trans; 
    q.pop();
    drawSphere(CameraSphere(camera(t(origin)), m->curr->radius));
    for(int i = 0; i < m->curr->points.size();i++){
      drawSphere(CameraSphere(camera(t(m->curr->points[i])), m->curr->radius / 3.0));
    }
    for(ModelTree::child_iterator it = m->begin(); it != m->end(); it++)
      q.push(make_pair(*it, t));
  }
*/

  freePosedModel(root);
  drawAxis(camera);
  SDL_Flip(m_display);
}

void SDLBackend::drawDepthMap(DepthMap depth){
  int maxval = 0;
  int bpp = m_display->format->BytesPerPixel;
  uint8_t *p;
  for(int i = 0; i < depth.width*depth.height;i++){
    if(*(depth.map+i)>maxval)
      maxval = *(depth.map+i);
  }
  p = (uint8_t*)m_display->pixels;
  for(int i = 0; i < depth.width*depth.height;i++){
    (p+bpp*i)[1] = 255-(int)((*(depth.map+i)/maxval)*255);
    }
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
