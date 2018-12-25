#ifndef __KINEMATICS_H
#define __KINEMATICS_H


#include "crane_dh_inverse/link.h"
#include <stdlib.h>

#define LINK_NUM 4
#define LINK_NUM_GRIPPER 5
#define GRIPPER_ID 5

extern Link chain[LINK_NUM_GRIPPER];

void setChain();
void solveFK(std::vector<double> q);
MatrixXd calcJacobian();
bool solveIK(Vector3d goal_pos);

#endif
