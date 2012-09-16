#include <vector>
#include "body_pose.h"
#include "ICRAScenarios.h"

using namespace std;
void kitchenStart(const CollisionChecker* c, 
                  vector<float>& postureVec, btTransform& robotFrame) {
  double la[7] = {-0.085, -0.192, 0.193, -0.159, 15.550, -1.611 -13.013};
  la[1] -= 0.08;
  vector<double> langles(la, la+8);
  
  double ra[7] = {0.154, -0.190, -0.666, -0.149, -2.381, -1.814, 0.139};
  ra[1] -= 0.08;
  vector<double> rangles(ra, ra+8);
  BodyPose body = BodyPose(2.283, -0.256, 0.0, -0.06);
  c->remapJointVector(langles, rangles, body, postureVec, robotFrame);
}

void kitchenGoal(const CollisionChecker* c, 
                 vector<float>& postureVec, btTransform& robotFrame) {
  double la[7] = {-0.085, -0.192, 0.193, -0.159, 15.550, -1.611 -13.013};
  la[1] -= 0.08;
  vector<double> langles(la, la+8);
  
  double ra[7] = {0.154, -0.190, -0.666, -0.149, -2.381, -1.814, 0.139};
  ra[1] -= 0.08;
  vector<double> rangles(ra, ra+8);
  BodyPose body = BodyPose(2.283, -0.256, 0.0, -0.06);
  c->remapJointVector(langles, rangles, body, postureVec, robotFrame);
}

void fountainStart(const CollisionChecker* c, 
                   vector<float>& postureVec, btTransform& robotFrame) {
  double la[7] = {-0.053, -0.312, 0.156, -0.149, 15.655, -1.782, -12.722};
  //la[1] -= 0.08;
  vector<double> langles(la, la+8);
  
  double ra[7] = {0.026, -0.313, -0.680, -0.149, -2.483, -1.819, 0.143};
  //ra[1] -= 0.08;
  vector<double> rangles(ra, ra+8);
  BodyPose body = BodyPose(3.479, 6.684, 0, -3.129);
  c->remapJointVector(langles, rangles, body, postureVec, robotFrame);
}
/*
void fountainGoal(const CollisionChecker* c, 
                  vector<float>& postureVec, btTransform& robotFrame) {
  double la[7] = {-0.053, -0.312, 0.156, -0.149, 15.655, -1.782, -12.722};
  //la[1] -= 0.08;
  vector<double> langles(la, la+8);
  
  double ra[7] = {0.026, -0.313, -0.680, -0.149, -2.483, -1.819, 0.143};
  //ra[1] -= 0.08;
  vector<double> rangles(ra, ra+8);
  BodyPose body = BodyPose(5.05, -0.30, 0, 3.13);
  c->remapJointVector(langles, rangles, body, postureVec, robotFrame);
}
*/
