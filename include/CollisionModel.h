#ifndef COLLISION_MODEL_H
#define COLLISION_MODEL_H
#include <map>
#include <vector>

struct CollisionSphere {
  float x, y, z, r;
  CollisionSphere() {}
  CollisionSphere(float _x, float _y, float _z, float _r)
    : x(_x), y(_y), z(_z), r(_r) {}
  CollisionSphere(const CollisionSphere& o) 
    : x(o.x), y(o.y), z(o.z), r(o.r) {}
};

typedef std::map<const std::string, std::vector<CollisionSphere> > CollisionGeometry;
CollisionGeometry* pr2CollisionGeometry(const char*);

#endif

