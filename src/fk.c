#include "fk.h"
#include <math.h>
#include <stdio.h>
#define M_PI 3.14159265358979323846

static double FK_BASE_HEIGHT = 0.04;   // meters (base height from ground to shoulder pivot)
static double FK_L1 = 0.108;            // meters (shoulder -> elbow)
static double FK_L2 = 0.107;            // meters (elbow -> wrist)
static double FK_L3 = 0.05;            // meters (wrist -> gripper tip)

void fk_set_links(double base_height, double L1, double L2, double L3) {
    FK_BASE_HEIGHT = base_height;
    FK_L1 = L1;
    FK_L2 = L2;
    FK_L3 = L3;
}

static double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

int forward_kinematics(const RobotJoints *j, double *x, double *y, double *z) {
    if (!j || !x || !y || !z) return -1;

    double base_deg     = (double) j->base;
    double shoulder_deg = (double) j->shoulder;
    double elbow_deg    = (double) j->elbow;
    double wrist_deg    = (double) j->wrist;

    double base_rad     = deg2rad(base_deg);
    double th1 = deg2rad(shoulder_deg);
    double th2 = deg2rad(elbow_deg);
    double th3 = deg2rad(wrist_deg);

    double r_plane = FK_L1 * cos(th1)
                   + FK_L2 * cos(th1 + th2)
                   + FK_L3 * cos(th1 + th2 + th3);

    double z_plane = FK_BASE_HEIGHT
                   + FK_L1 * sin(th1)
                   + FK_L2 * sin(th1 + th2)
                   + FK_L3 * sin(th1 + th2 + th3);

    *x = r_plane * cos(base_rad);
    *y = r_plane * sin(base_rad);
    *z = z_plane;

    return 0;
}
