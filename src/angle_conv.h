#ifndef ANGLE_CONV_H
#define ANGLE_CONV_H

#include "joints.h"

/* Set these from your calibration process */
void conv_init();

/* servo (0..180) -> math radians (or degrees depending on your FK/IK) */
void servo_to_math(const RobotJoints *servo, double *base_rad, double *shoulder_rad, double *elbow_rad, double *wrist_rad);

/* math radians -> servo angles (0..180) */
void math_to_servo(double base_rad, double shoulder_rad, double elbow_rad, double wrist_rad, RobotJoints *servo_out);

/* deg<->pulse for SSC-32 */
int deg_to_pulse(int deg);
int pulse_to_deg(int pulse);

#endif
