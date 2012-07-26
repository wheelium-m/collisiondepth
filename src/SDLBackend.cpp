#include <stdio.h>
#include "SDLBackend.h"
#include <SDL_gfxPrimitives.h>
#include <queue>
#include <iostream>
#include "DepthMap.h"
#include "Intervals.h"
#include "HeatPalette.h"
#include <algorithm>
#include "CollisionChecker.h"
#include <sys/time.h>

using namespace std;

//#define ZOOM 180.0
#define FOCAL_LENGTH 594.0
#define PI 3.1415926535
#define SPHERE_RADIUS 0.1

SDLBackend::SDLBackend() : m_display(NULL) {
  cout << "SDLBackend constructor" << endl;
  checker = new CollisionChecker(&pr2());
  DepthMap* depth = new DepthMap();
  depth->getKinectMapFromFile(FOCAL_LENGTH, "depth_texture.bin");
  depth->addDilation(SPHERE_RADIUS);
  checker->addDepthMap(depth);
}

/* Cleanup SDL. */
SDLBackend::~SDLBackend() {
  printf("SDL shutdown\n");
  SDL_Quit();
}

// Camera coordinate system to screen space
inline btVector3 SDLBackend::cameraToScreen(btVector3 pt) {
  // The scaling factor just zooms things in a bit. It is arbitrary.
  static const btVector3 scale = btVector3(FOCAL_LENGTH, FOCAL_LENGTH, 1);
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
  // int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * FOCAL_LENGTH) : (int)r;
  // if(bound < 1) bound = 1;

  // int bsq = bound * bound; // screen-space radius squared
  // float unproject = r / (float)bound;

  const int screenX = (int)screenSpace.x();
  const int screenY = (int)screenSpace.y();

  const float* map = depth.getMap(SPHERE_RADIUS);
  if(screenX < 0 || screenX >= m_display->w) return false;
  if(screenY < 0 || screenY >= m_display->h) return false;
  float ptDepth = camSpace.z() + SPHERE_RADIUS;
  return depth.collides(map, screenX, screenY, ptDepth);
/*
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
      if(!depth.collides(map, screenX + x, rowIndex, ptDepth)) return false;
      x = nextFreeIndex(rowEnd, rowCurr, x + screenX) - screenX;
    }
    //addInterval(screenSpace.x() + left, screenSpace.x() + right, row);
  }
  return true;
*/
}

void SDLBackend::drawSphere(const bool sphereCollides,
                            vector<list<pair<int,int> > >& rowIntervals, 
                            const CameraSphere& sphere, DepthMap depth) {
  btVector3 camSpace = sphere.center;
  float r = sphere.r;
  camSpace = (depth.trans.inverse())(camSpace);
  /* This way, spheres behind the camera will not be drawn. */
  if(camSpace.z() - r < 0.0) return;
  btVector3 pt = cameraToScreen(project(camSpace));

  // FIXME: the screen coordinate system scaling factor hack (FOCAL_LENGTH)
  // appears both here and in cameraToScreen.

  // Perspective projection
  int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * FOCAL_LENGTH) : (int)r;

  if(bound < 1) bound = 1;
  int bpp = m_display->format->BytesPerPixel;
  uint8_t *p;
  int colorScale = 128 / bound;
  int bsq = bound * bound; // radius squared

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
      uint8_t shade = (uint8_t)(128 + depthColor < 255 ? 128 + depthColor : 255);
      if(sphereCollides) {      
        uint32_t col = SDL_MapRGB(m_display->format, 0, shade, 0);
        memcpy(p, &col, 4);
      }
      else{
        uint32_t col = SDL_MapRGB(m_display->format, shade, 0, 0);
        memcpy(p, &col, 4);
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

/* Draw a sphere with shading indicating depth. */
void SDLBackend::drawSphere(vector<list<pair<int,int> > >& rowIntervals,
                            const CameraSphere& sphere, DepthMap depth) {
  
  btVector3 camSpace = sphere.center;
  float r = sphere.r;
  camSpace = (depth.trans.inverse())(camSpace);
  /* This way, spheres behind the camera will not be drawn. */
  if(camSpace.z() - r < 0.0) return;
  btVector3 pt = cameraToScreen(project(camSpace));

  // FIXME: the screen coordinate system scaling factor hack (FOCAL_LENGTH)
  // appears both here and in cameraToScreen.

  // Perspective projection
  int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * FOCAL_LENGTH) : (int)r;

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
      uint8_t shade = (uint8_t)(128 + depthColor < 255 ? 128 + depthColor : 255);
      if(sphereCollides) {      
        uint32_t col = SDL_MapRGB(m_display->format, 0, shade, 0);
        memcpy(p, &col, 4);
      }
      else{
        uint32_t col = SDL_MapRGB(m_display->format, shade, 0, 0);
        memcpy(p, &col, 4);
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
  if(SDL_Init(SDL_INIT_EVERYTHING) < 0) {
    char* msg = SDL_GetError();
    cout << "SDL_Init error: " << msg << endl;
    return false;
  }
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
  if(!depth.getRawMap()){
    //depth.getKinectMapFromFile("depth_texture.bin");
    depth.makeSimpleMap(FOCAL_LENGTH);
  }
  drawSphere(dummy, CameraSphere(camera(btVector3(12,12,0)), 4, "dummy"), depth);
  SDL_Flip(m_display);
}

void SDLBackend::renderModel(const ModelTree& rawRoot, const btTransform &camera) {
  SDL_FillRect(m_display, NULL, 0);
  btVector3 origin(0,0,0);

  map<string,float> posture;
  // Lift the PR2's left arm by 45 degrees and swing the right arm out.
  posture["l_shoulder_lift_link"] = -45.0f * PI / 180;
  posture["r_shoulder_pan_link"] = -45.0f * PI / 180;

  map<string,bool> collisionInfo;
  
  struct timeval start;
  struct timeval stop;
  gettimeofday(&start, NULL);
  for(int i = 0; i < 100; i++) {
    checker->getCollisionInfo(camera, SPHERE_RADIUS, posture, collisionInfo);
  }
  gettimeofday(&stop, NULL);
  float seconds = (stop.tv_sec - start.tv_sec) + \
                  0.000001f * (float)(stop.tv_usec - start.tv_usec);
  seconds /= 100.0f;
  cout << "Collision detection took " << seconds*1000.0f << "ms" << endl;
  cout << "  with " << collisionInfo.size() << " components" << endl;

  ModelTree* root = poseModel(rawRoot, posture);

  vector<CameraSphere> spheres;
  painterSort(*root, camera, spheres);

  vector<list<pair<int,int> > > rowIntervals(m_display->h);
  
  static DepthMap depth;
  if(!depth.getRawMap()){
    cout << "Preparing depth map" << endl;
    depth.getKinectMapFromFile(FOCAL_LENGTH, "depth_texture.bin");
    //depth.makeSimpleMap(FOCAL_LENGTH);

    // If we use a uniform sphere size, then we can pass a windowed
    // min-filter over the depthmap and then sample a single pixel for
    // each sphere. The window is defined by the projection of a
    // sphere centered at each point represented by a depth map pixel.
    depth.addDilation(SPHERE_RADIUS);
  }

  drawDepthMap(depth, SPHERE_RADIUS);

  for(int i = 0; i < spheres.size(); i++) 
    drawSphere(collisionInfo[spheres[i].jointName], rowIntervals, spheres[i], depth);
    //drawSphere(rowIntervals, spheres[i], depth);

  freePosedModel(root);

  drawAxis(camera);
  SDL_Flip(m_display);
}

void SDLBackend::drawDepthMap(const DepthMap& depth, const float r){
  float maxval = 0;
  int bpp = m_display->format->BytesPerPixel;
  uint8_t *p;
  const float* map = depth.getMap(r);
  for(int i = 0; i < depth.width*depth.height;i++){
    if(*(map+i) > maxval)
      maxval = *(map+i);
  }
  p = (uint8_t*)m_display->pixels;
  for(int i = 0; i < depth.width*depth.height;i++){
    //(p+bpp*i)[1] = 255-(int)((*(depth.map+i)/maxval)*255);
    const uint8_t* col = getColor(min(1.0f, *(map+i) / maxval));
    uint32_t sdlCol = SDL_MapRGB(m_display->format, col[0], col[1], col[2]);
    memcpy(p+bpp*i, &sdlCol, 4);
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
