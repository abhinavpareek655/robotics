#ifndef FK_H
#define FK_H

#include "joints.h"

/* Set link lengths for FK calculations (in meters)
 * base_height: height from base to shoulder axis
 * L1: shoulder to elbow length
 * L2: elbow to wrist length
 * L3: wrist to gripper tip length
 */
void fk_set_links(double base_height, double L1, double L2, double L3);

/* Calculate forward kinematics
 * Input: joints (angles in degrees)
 * Output: x, y, z (position in meters)
 * Returns: 0 on success, -1 on error
 */
int forward_kinematics(const RobotJoints *joints, double *x, double *y, double *z);

#endif /* FK_H */