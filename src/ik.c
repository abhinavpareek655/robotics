#include <math.h>
#include <stdio.h>
#include "ik.h"
#define M_PI 3.14159265358979323846

static double IK_BASE_HEIGHT = 0.04;
static double IK_L1 = 0.108;
static double IK_L2 = 0.107;
static double IK_L3 = 0.05;

void ik_set_links(double base_height, double L1, double L2, double L3) {
    IK_BASE_HEIGHT = base_height;
    IK_L1 = L1;
    IK_L2 = L2;
    IK_L3 = L3;
}

static double rad2deg(double r) {
    return r * 180.0 / M_PI;
}

int inverse_kinematics(double x, double y, double z, RobotJoints *out) {

    if (!out) return -1;

    double theta_base = atan2(y, x);

    double r = sqrt(x*x + y*y);
    double z2 = z - IK_BASE_HEIGHT;

    double D = sqrt(r*r + z2*z2);

    double maxReach = IK_L1 + IK_L2;
    double minReach = fabs(IK_L1 - IK_L2);

    if (D > maxReach) {
        printf("\nIK Error: target unreachable (too far).\n");
        return -2;
    }
    if (D < minReach) {
        printf("\nIK Error: target unreachable (too close).\n");
        return -3;
    }

    double cos_elbow = (IK_L1*IK_L1 + IK_L2*IK_L2 - D*D) / (2 * IK_L1 * IK_L2);

    if (cos_elbow < -1 || cos_elbow > 1) {
        printf("\nIK Error: numerical limit.\n");
        return -4;
    }

    double theta_elbow = acos(cos_elbow);

    double angle1 = atan2(z2, r);
    double angle2 = atan2(IK_L2 * sin(theta_elbow),
                          IK_L1 + IK_L2 * cos(theta_elbow));

    double theta_shoulder = angle1 - angle2;

    out->base     = (int)rad2deg(theta_base);
    out->shoulder = (int)rad2deg(theta_shoulder);
    out->elbow    = (int)rad2deg(theta_elbow);
    
    out->wrist = -(out->shoulder + out->elbow);

    return 0;
}
