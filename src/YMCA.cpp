#include "YMCA.h"
#include <string>

#define DEG2RAD(x) ((x) * (PI / 180))
#define PI 3.1415926535
using namespace std;

int YMCA::nextFrame(int i) const {
  static int numFrames = 70 + 70 + 90 + 70 + 70;
  if(i < numFrames - 1) return ++i; else return 0;
}

int YMCA::numFrames() const {
  return 70 + 70 + 90 + 70 + 70;
}

const map<string,float>& YMCA::getJointAngles(int i) {
  static map<string,float> tempPosture;
  tempPosture.clear();
  float theta, step;
  theta = DEG2RAD(-90);
  tempPosture["l_shoulder_pan_link"] = -theta;
  tempPosture["r_shoulder_pan_link"] = theta;

  if(i < 70) {
    // Y
    step = (float)i / 70.0f;
    theta = interpolate(0, 70, step);
    tempPosture["l_shoulder_lift_link"] = -theta;
    tempPosture["r_shoulder_lift_link"] = -theta;
  } else if(i < 70 + 70) {
    // M
    theta = -DEG2RAD(70.0f);
    tempPosture["l_shoulder_lift_link"] = theta;
    tempPosture["r_shoulder_lift_link"] = theta;

    step = (float)(i - 70) / 70.0f;
    theta = interpolate(0, 70, step);
    tempPosture["l_elbow_flex_link"] = -theta;
    tempPosture["r_elbow_flex_link"] = -theta;
  } else if(i < 70 + 70 + 90) {
    // C
    theta = -DEG2RAD(70.0f);
    tempPosture["l_elbow_flex_link"] = theta;

    step = (float)(i - (70 + 70)) / 90.0f;
    theta = interpolate(70, -20, step);
    tempPosture["l_shoulder_lift_link"] = -theta;

    theta = -DEG2RAD(70.0f);
    tempPosture["r_shoulder_lift_link"] = theta;

    theta = interpolate(70, 35, step);
    tempPosture["r_elbow_flex_link"] = -theta;
  } else if(i < 70 + 70 + 90 + 70) {
    // A
    step = (float)(i - (70+70+90)) / 70.0f;
    theta = interpolate(70, 55, step);
    tempPosture["l_elbow_flex_link"] = -theta;

    theta = interpolate(-35.0f, -55.0f, step);
    tempPosture["r_elbow_flex_link"] = theta;

    theta = interpolate(-20,75,step);
    tempPosture["l_shoulder_lift_link"] = -theta;

    theta = interpolate(-70,-75,step);
    tempPosture["r_shoulder_lift_link"] = theta;
  } else if(i < 70 + 70 + 90 + 70 + 70) {
    // Back to start
    step = (float)(i - (70+70+90+70)) / 70.0f;
    theta = interpolate(55,0,step);
    tempPosture["l_elbow_flex_link"] = -theta;
    tempPosture["r_elbow_flex_link"] = -theta;

    theta = interpolate(75, 0, step);
    tempPosture["l_shoulder_lift_link"] = -theta;
    tempPosture["r_shoulder_lift_link"] = -theta;
  }
  return tempPosture;
}
