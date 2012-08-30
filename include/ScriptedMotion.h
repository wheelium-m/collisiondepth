#ifndef SCRIPTEDMOTION_H
#define SCRIPTEDMOTION_H
#include <map>

class ScriptedMotion {
protected:
  float interpolate(float start, float stop, float step) const {
    return (start + (stop - start)*step)*(3.14159/180.0f);
  }

public:
  virtual const std::map<std::string,float>& getJointAngles(int) = 0;
  virtual int nextFrame(int) const = 0;
  virtual int numFrames() const = 0;
};

#endif
