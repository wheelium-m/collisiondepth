#include <btBulletDynamicsCommon.h>

class Renderer {
public:
  virtual bool init(int width, int height) = 0;
  virtual bool loop(btTransform &camera) = 0;
};
