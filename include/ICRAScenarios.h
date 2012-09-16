#ifndef ICRA_SCENARIOS_H
#define ICRA_SCENARIOS_H
#include <btBulletDynamicsCommon.h>
#include "CollisionChecker.h"

void kitchenStart(const CollisionChecker*,
                  std::vector<float>& postureVec, btTransform& robotFrame);
void kitchenGoal(const CollisionChecker*,
                 std::vector<float>& postureVec, btTransform& robotFrame);
void fountainStart(const CollisionChecker*,
                   std::vector<float>& postureVec, btTransform& robotFrame);
/* void fountainGoal(const CollisionChecker*, */
/*                   std::vector<float>& postureVec, btTransform& robotFrame); */

#endif
