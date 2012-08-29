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

// Kinect focal length
//#define FOCAL_LENGTH 594.0

// When generating depthmaps from PCD files, we use a virtual camera
// with a focal length of 1.0.
#define FOCAL_LENGTH 320.0
#define FOCAL_LENGTH_X 320.0
#define FOCAL_LENGTH_Y 240.0

#define PI 3.1415926535
#define SPHERE_RADIUS 0.02

extern btTransform parsePose(const char* filename);

void SDLBackend::addDepthMap(const char* depthImg, const char* imgPose) {
  DepthMap* depth = new DepthMap();
  depth->getKinectMapFromFile(FOCAL_LENGTH, depthImg);
  btTransform t = parsePose(imgPose);
  depth->trans = btTransform(t.getRotation(), // .inverse(), 
                             btTransform(t.getRotation())(-1 * t.getOrigin()));
  depth->transInv = depth->trans.inverse();
  depth->addDilation(SPHERE_RADIUS);
  this->checker->addDepthMap(depth);
}

SDLBackend::SDLBackend() : m_display(NULL) {
  cout << "SDLBackend constructor" << endl;
  checker = new CollisionChecker(&pr2());
}

/* Cleanup SDL. */
SDLBackend::~SDLBackend() {
  printf("SDL shutdown\n");
  SDL_Quit();
}

/*
// Camera coordinate system to screen space
inline btVector3 SDLBackend::cameraToScreen(btVector3 pt) {
  // The scaling factor just zooms things in a bit. It is arbitrary.
  static const btVector3 scale = btVector3(FOCAL_LENGTH_X, FOCAL_LENGTH_Y, 1);
  btVector3 v = pt * scale + cameraToScreenTranslation;
  return v;
}
*/

// Perspective projection
inline btVector3 SDLBackend::project(btVector3 v) {
  //if(v.z() != 0) return v * (1.0 / -v.z());
  // Project and transform to screen space
  if(v.z() != 0) {
    float invZ = -1.0 / v.z();
    return (v * btVector3(FOCAL_LENGTH_X * invZ, -FOCAL_LENGTH_Y * invZ, invZ) + 
            cameraToScreenTranslation);
  }
  else return v;
}

/* Draw the axis relative to the camera perspective */
void SDLBackend::drawAxis(const btTransform & camera){
  btVector3 orig(0.0,0.0,0.0);
  btVector3 x(1.0,0.0,0.0);
  btVector3 y(0.0,1.0,0.0);
  btVector3 z(0.0,0.0,1.0);
  orig = project(camera(orig));
  x = project(camera(x));
  y = project(camera(y));
  z = project(camera(z));
  
  lineRGBA(m_display, orig.x(), orig.y(), x.x(), x.y(), 255, 0, 0, 255);

  lineRGBA(m_display, orig.x(), orig.y(), y.x(), y.y(), 0,255,0,255);

  lineRGBA(m_display, orig.x(), orig.y(), z.x(), z.y(), 0,0,255,255);
  //void Draw_Line(SDL_Surface *super,
  //		 Sint16 x1, Sint16 y1, Sint16 x2, Sint16 y2,
  //		 Uint32 color);
  
}

void SDLBackend::drawSphere(const bool sphereCollides,
                            vector<list<pair<int,int> > >& rowIntervals, 
                            const CameraSphere& sphere) {
  btVector3 camSpace = sphere.center;
  float r = sphere.r;

  /* This way, spheres behind the camera will not be drawn. */
  if(camSpace.z() + r > 0.0) return;
  btVector3 pt = project(camSpace);
  camSpace.setZ(-camSpace.getZ());

  // FIXME: the screen coordinate system scaling factor hack (FOCAL_LENGTH)
  // appears both here and in cameraToScreen.

  // Perspective projection
  int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * FOCAL_LENGTH_X) : (int)r;
  // int bound = camSpace.z() != 0 ? (int)((r / camSpace.z()) * FOCAL_LENGTH_X) : (int)r;

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

// FIXME: Gross hack
int bestView = 0;

void SDLBackend::renderModel(const ModelTree& rawRoot, 
                             const btTransform &robotFrame) {
  SDL_FillRect(m_display, NULL, 0);
  btVector3 origin(0,0,0);

  map<string,float> posture;
  // Lift the PR2's left arm by 45 degrees and swing the right arm out.
  posture["l_shoulder_lift_link"] = -45.0f * PI / 180;
  posture["r_shoulder_pan_link"] = -45.0f * PI / 180;
  
  vector<float> postureVec;
  checker->makeJointVector(posture, postureVec);

  vector<bool> collisionVec;
  map<string,bool> collisionInfo;

  // FIXME: Always drawing over the first depth map
  btTransform camera;
/*
  btVector3 vtmp;

  // A point starting out straight ahead (the negative Z-axis) is
  // rotated 90 degrees to lie on the negative X-axis. The rotated
  // point is then translated along the X-axis to arrive at (-1,0,0);
  vtmp = btTransform(btQuaternion(btVector3(0,1,0), 3.14159*0.5),
                     btVector3(1,0,0))(btVector3(0,0,-2));
  cout << "sanity1 " << vtmp.getX() << ", " << vtmp.getY() << ", " << vtmp.getZ() << endl;

  // A point starting out ahead and to the right is rotated by 90
  // degrees, taking it to left-ahead (-2,0,-1).
  vtmp = btTransform(btQuaternion(btVector3(0,1,0), 3.14159*0.5),
                     btVector3(0,0,0))(btVector3(1,0,-2));
  cout << "sanity2 " << vtmp.getX() << ", " << vtmp.getY() << ", " << vtmp.getZ() << endl;

  // Now we want to consider a camera that is moved along the positive
  // X-axis and rotated about its Y-axis to look to the left. A point
  // on the negative Z-axis should end up ahead-right.
  camera = btTransform(btQuaternion(btVector3(0,1,0), 3.14159*0.5));
  camera.setOrigin(camera(-1 * btVector3(1,0,0)));
  vtmp = camera(btVector3(0,0,-2));
  cout << "sanity3 " << vtmp.getX() << ", " << vtmp.getY() << ", " << vtmp.getZ() << endl;
*/

  //int bestView = 0;
  camera = checker->getDepthMap(bestView)->trans;
  camera = camera * robotFrame;
  
  struct timeval start;
  struct timeval stop;
  gettimeofday(&start, NULL);
  for(int i = 0; i < 1000; i++) {
    checker->getCollisionInfo(robotFrame, SPHERE_RADIUS, postureVec, collisionVec);
  }
  gettimeofday(&stop, NULL);
  checker->makeCollisionMap(collisionVec, collisionInfo);
  /*
  for(map<string,bool>::const_iterator it = collisionInfo.begin();
      it != collisionInfo.end();
      it++) {
    cout << it->first << " => " << it->second << endl;
  }
  */

  float seconds = (stop.tv_sec - start.tv_sec) + \
                  0.000001f * (float)(stop.tv_usec - start.tv_usec);
  seconds /= 1000.0f;
  cout << "Collision detection took " << seconds*1000.0f << "ms";
  cout << "(" << 1.0 / seconds << " Hz)" << endl;
  cout << "  with " << collisionInfo.size() << " components" << endl;

  ModelTree* root = poseModel(rawRoot, posture);

  vector<CameraSphere> spheres;
  painterSort(*root, camera, spheres);
  vector<list<pair<int,int> > > rowIntervals(m_display->h);
  
  const DepthMap* depth = this->checker->getDepthMap(bestView);

  drawDepthMap(depth, SPHERE_RADIUS);

  for(int i = 0; i < spheres.size(); i++) 
    drawSphere(collisionInfo[spheres[i].jointName], rowIntervals, 
               spheres[i]);
    
  freePosedModel(root);

  drawAxis(camera);
  SDL_Flip(m_display);
}

void SDLBackend::drawDepthMap(const DepthMap* depth, const float r){
  float maxval = 0;
  int bpp = m_display->format->BytesPerPixel;
  uint8_t *p;
  const float* map = depth->getMap(r);

  for(int i = 0; i < depth->width * depth->height; i++){
    if(*(map+i) > maxval)
      maxval = *(map+i);
  }

  p = (uint8_t*)m_display->pixels;
  for(int i = 0; i < depth->width * depth->height; i++){
    //(p+bpp*i)[1] = 255-(int)((*(depth.map+i)/maxval)*255);
    const uint8_t* col = getColor(min(1.0f, *(map+i) / maxval));
    uint32_t sdlCol = SDL_MapRGB(m_display->format, col[0], col[1], col[2]);
    memcpy(p+bpp*i, &sdlCol, 4);
  }
}

/* Check UI events, render the current frame, and return true if we
 * should continue looping and false if the user asked to quit. */
bool SDLBackend::loop(btTransform &robotFrame) {
  SDL_Event keyevent;    //The SDL event that we will poll to get events.
  static int moveFigure=0;
  static btVector3 translateRobot;
  while (SDL_PollEvent(&keyevent))   //Poll our SDL key event for any keystrokes.
    {
      
      switch(keyevent.type){
	
      case SDL_KEYDOWN:
	switch(keyevent.key.keysym.sym){
        case SDLK_LEFT:
	  translateRobot.setY(1);
          break;
        case SDLK_RIGHT:
          translateRobot.setY(-1);
	  break;
        case SDLK_UP:
          translateRobot.setX(1);
	  break;
        case SDLK_DOWN:
          translateRobot.setX(-1);
	  break;
        case SDLK_PAGEUP:
          translateRobot.setZ(1);
          break;
        case SDLK_PAGEDOWN:
          translateRobot.setZ(-1);
          break;
	default:
          break;
	}
	break;
	
      case SDL_KEYUP:
	switch(keyevent.key.keysym.sym){
        case SDLK_LEFT:
	  translateRobot.setY(0);
          break;
        case SDLK_RIGHT:
          translateRobot.setY(0);
          break;
        case SDLK_UP:
          translateRobot.setX(0);
          break;
        case SDLK_DOWN:
          translateRobot.setX(0);
          break;
        case SDLK_PAGEUP:
          translateRobot.setZ(0);
          break;
        case SDLK_PAGEDOWN:
          translateRobot.setZ(0);
        case SDLK_v:
          bestView = !bestView;
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
	  robotFrame = btTransform(btQuaternion::getIdentity(), 
                                   robotFrame.getOrigin()) *
                       (rot*btTransform(robotFrame.getRotation(), btVector3(0.0,0.0,0.0)));
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

  float speed = 0.04;

  btTransform rot(robotFrame.getRotation());
  robotFrame.setOrigin(robotFrame.getOrigin() + rot(speed*translateRobot));
  renderModel(pr2(), robotFrame);
  //SDL_framerateDelay(m_fps);
  return true;
}
