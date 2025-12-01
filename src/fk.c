#include <math.h>
#include "fk.h"

#define M_PI 3.14159265358979323846

/* AL5D Robot Arm Link Lengths (matching ik.c) */
static double FK_BASE_HEIGHT = 0.069;  // Base height (meters)
static double FK_L1 = 0.146;           // Shoulder to elbow (meters)
static double FK_L2 = 0.187;           // Elbow to wrist (meters)
static double FK_L3 = 0.100;           // Wrist to gripper tip (meters)

void fk_set_links(double base_height, double L1, double L2, double L3) {
    FK_BASE_HEIGHT = base_height;
    FK_L1 = L1;
    FK_L2 = L2;
    FK_L3 = L3;
}

static double deg2rad(double d) {
    return d * M_PI / 180.0;
}

int forward_kinematics(const RobotJoints *joints, double *x, double *y, double *z) {
    if (!joints || !x || !y || !z) return -1;

    // Convert joint angles from degrees to radians
    double theta_base = deg2rad(joints->base);
    double theta_shoulder = deg2rad(joints->shoulder);
    double theta_elbow = deg2rad(joints->elbow);
    double theta_wrist = deg2rad(joints->wrist);

    // Calculate position of elbow relative to shoulder
    double elbow_x = FK_L1 * cos(theta_shoulder);
    double elbow_z = FK_L1 * sin(theta_shoulder);

    // Calculate position of wrist relative to shoulder
    double wrist_angle = theta_shoulder + theta_elbow;
    double wrist_x = elbow_x + FK_L2 * cos(wrist_angle);
    double wrist_z = elbow_z + FK_L2 * sin(wrist_angle);

    // Calculate position of gripper tip relative to shoulder
    double gripper_angle = wrist_angle + theta_wrist;
    double gripper_x = wrist_x + FK_L3 * cos(gripper_angle);
    double gripper_z = wrist_z + FK_L3 * sin(gripper_angle);

    // Convert from shoulder-relative to base-relative coordinates
    // Rotate around base (theta_base) and add base height
    *x = gripper_x * cos(theta_base);
    *y = gripper_x * sin(theta_base);
    *z = gripper_z + FK_BASE_HEIGHT;

    return 0;
}