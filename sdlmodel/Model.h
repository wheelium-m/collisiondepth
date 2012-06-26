#include <btBulletDynamicsCommon.h>

class Model{
 public:
  btVector3 *points;
  float *angles_per_sec;
  Model(){
    points = new btVector3[3];
    points[0] = btVector3(0.0,0.0,0.0);
    points[1] = btVector3(1.0,0.0,0.0);
    points[2] = btVector3(2.0,0.0,0.0);
  
    
    angles_per_sec = new float[2];
    angles_per_sec[0] = 2.0;
    angles_per_sec[1] = 4.0;
    
    
  }
  
};
