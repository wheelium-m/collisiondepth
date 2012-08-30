#ifndef YMCA_H
#define YMCA_H

#include "ScriptedMotion.h"

class YMCA : public ScriptedMotion {
public:
  YMCA() {};
  const std::map<std::string,float>& getJointAngles(int);
  int nextFrame(int) const;
  int numFrames() const;
};

#endif
