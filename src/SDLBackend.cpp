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
#define SPHERE_RADIUS 0.05

#define DEG2RAD(x) ((x) * (PI / 180))

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

// FIXME: Gross hack that allows us to cycle through available views
// with keyboard controls.
int bestView = 0;

const int numFrames = 70 + 70 + 90 + 70 + 70;

float interpolate(float start, float stop, float step) {
  return DEG2RAD(start + (stop - start)*step);
}

map<string,float>* ymca(int i) {
  map<string,float> *posture = new map<string,float>();
  float theta, step;
  theta = DEG2RAD(-90);
  (*posture)["l_shoulder_pan_link"] = -theta;
  (*posture)["r_shoulder_pan_link"] = theta;

  if(i < 70) {
    // Y
    step = (float)i / 70.0f;
    theta = interpolate(0, 70, step);
    (*posture)["l_shoulder_lift_link"] = -theta;
    (*posture)["r_shoulder_lift_link"] = -theta;
  } else if(i < 70 + 70) {
    // M
    theta = -DEG2RAD(70.0f);
    (*posture)["l_shoulder_lift_link"] = theta;
    (*posture)["r_shoulder_lift_link"] = theta;

    step = (float)(i - 70) / 70.0f;
    theta = interpolate(0, 70, step);
    (*posture)["l_elbow_flex_link"] = -theta;
    (*posture)["r_elbow_flex_link"] = -theta;
  } else if(i < 70 + 70 + 90) {
    // C
    theta = -DEG2RAD(70.0f);
    (*posture)["l_elbow_flex_link"] = theta;

    step = (float)(i - (70 + 70)) / 90.0f;
    theta = interpolate(70, -20, step);
    (*posture)["l_shoulder_lift_link"] = -theta;

    theta = -DEG2RAD(70.0f);
    (*posture)["r_shoulder_lift_link"] = theta;

    theta = interpolate(70, 35, step);
    (*posture)["r_elbow_flex_link"] = -theta;
  } else if(i < 70 + 70 + 90 + 70) {
    // A
    step = (float)(i - (70+70+90)) / 70.0f;
    theta = interpolate(70, 55, step);
    (*posture)["l_elbow_flex_link"] = -theta;

    theta = interpolate(-35.0f, -55.0f, step);
    (*posture)["r_elbow_flex_link"] = theta;

    theta = interpolate(-20,75,step);
    (*posture)["l_shoulder_lift_link"] = -theta;

    theta = interpolate(-70,-75,step);
    (*posture)["r_shoulder_lift_link"] = theta;
  } else if(i < 70 + 70 + 90 + 70 + 70) {
    // Back to start
    step = (float)(i - (70+70+90+70)) / 70.0f;
    theta = interpolate(55,0,step);
    (*posture)["l_elbow_flex_link"] = -theta;
    (*posture)["r_elbow_flex_link"] = -theta;

    theta = interpolate(75, 0, step);
    (*posture)["l_shoulder_lift_link"] = -theta;
    (*posture)["r_shoulder_lift_link"] = -theta;
  }
  return posture;
}

void SDLBackend::renderModel(const ModelTree& rawRoot, 
                             const btTransform &robotFrame) {
  SDL_FillRect(m_display, NULL, 0);
  btVector3 origin(0,0,0);

  static int currentFrame = 0;

/*
  map<string,float> posture;
  // Lift the PR2's left arm by 45 degrees and swing the right arm out.
  posture["l_shoulder_lift_link"] = -45.0f * PI / 180;
  posture["r_shoulder_pan_link"] = -45.0f * PI / 180;
*/
  map<string,float>* posture = ymca(currentFrame);
  currentFrame++;
  if(currentFrame >= numFrames) currentFrame = 0;
  
  vector<float> postureVec;
  checker->makeJointVector(*posture, postureVec);

  vector<bool> collisionVec;
  map<string,bool> collisionInfo;

  btTransform camera;

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

  ModelTree* root = poseModel(rawRoot, *posture);

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
          bestView++;
          if(bestView >= this->checker->numDepthMaps()) bestView = 0;
          break;
        default:
          break;
	}
	break;
      case SDL_MOUSEMOTION:
	if(moveFigure==1){
          btTransform robotRot = btTransform(robotFrame.getRotation());
          btQuaternion yaw = btQuaternion(robotRot(btVector3(0,0,1)), 
                                          keyevent.motion.xrel/50.0f);
          robotRot.setRotation(yaw * robotRot.getRotation());
          btQuaternion tilt = btQuaternion(robotRot(btVector3(0,-1,0)),
                                           keyevent.motion.yrel/50.0f);
          robotFrame.setRotation(tilt * robotRot.getRotation());
	}
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
