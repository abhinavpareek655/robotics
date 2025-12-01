#include <math.h>
#include <stdio.h>
#include "ik.h"
#define M_PI 3.14159265358979323846

/* AL5D Robot Arm Link Lengths (in meters)
 * Base height: ~69mm = 0.069m (from base servo axis to shoulder servo axis)
 * L1 (Shoulder to Elbow): ~146mm = 0.146m
 * L2 (Elbow to Wrist): ~187mm = 0.187m  
 * L3 (Wrist to Gripper tip): ~100mm = 0.100m (approximate, depends on gripper)
 * 
 * Total reach: ~0.433m (43.3cm) from shoulder axis
 */

static double IK_BASE_HEIGHT = 0.069;  // Base height (meters)
static double IK_L1 = 0.146;           // Shoulder to elbow (meters)
static double IK_L2 = 0.187;           // Elbow to wrist (meters)
static double IK_L3 = 0.100;           // Wrist to gripper tip (meters)

void ik_set_links(double base_height, double L1, double L2, double L3) {
    IK_BASE_HEIGHT = base_height;
    IK_L1 = L1;
    IK_L2 = L2;
    IK_L3 = L3;
    printf("IK: Link lengths set to base=%.3f, L1=%.3f, L2=%.3f, L3=%.3f\n",
           base_height, L1, L2, L3);
}

static double rad2deg(double r) {
    return r * 180.0 / M_PI;
}

int inverse_kinematics(double x, double y, double z, RobotJoints *out) {

    if (!out) return -1;

    // 1. Calculate base rotation (theta_base)
    double theta_base = atan2(y, x);

    // 2. Calculate horizontal reach and adjusted z
    double r = sqrt(x*x + y*y);  // Horizontal distance from base center
    double z_adj = z - IK_BASE_HEIGHT;  // Adjust z relative to shoulder axis

    // 3. Account for wrist offset (L3) - project back toward shoulder
    // The wrist joint should reach (r - L3*cos(wrist_angle), z_adj - L3*sin(wrist_angle))
    // For simplicity, we'll subtract L3 from the horizontal reach
    double r_shoulder = r - IK_L3;  // Effective reach for 2-link IK
    
    if (r_shoulder < 0) {
        r_shoulder = 0;  // Clamp to prevent negative reach
    }

    // 4. Calculate distance from shoulder to target (2-link planar problem)
    double D = sqrt(r_shoulder*r_shoulder + z_adj*z_adj);

    // 5. Check reachability
    double maxReach = IK_L1 + IK_L2;
    double minReach = fabs(IK_L1 - IK_L2);

    printf("IK Debug: r=%.3f, z_adj=%.3f, r_shoulder=%.3f, D=%.3f, max=%.3f, min=%.3f\n",
           r, z_adj, r_shoulder, D, maxReach, minReach);

    if (D > maxReach + 0.001) {  // Small tolerance
        printf("IK Error: Target unreachable (too far). D=%.3f > max=%.3f\n", D, maxReach);
        return -2;
    }
    if (D < minReach - 0.001) {
        printf("IK Error: Target unreachable (too close). D=%.3f < min=%.3f\n", D, minReach);
        return -3;
    }

    // 6. Calculate elbow angle using law of cosines
    double cos_elbow = (IK_L1*IK_L1 + IK_L2*IK_L2 - D*D) / (2.0 * IK_L1 * IK_L2);

    // Clamp to valid range (numerical safety)
    if (cos_elbow < -1.0) cos_elbow = -1.0;
    if (cos_elbow > 1.0) cos_elbow = 1.0;

    double theta_elbow = acos(cos_elbow);  // Elbow angle (positive = bent)

    // 7. Calculate shoulder angle
    double alpha = atan2(z_adj, r_shoulder);  // Angle to target from shoulder
    
    double cos_beta = (IK_L1*IK_L1 + D*D - IK_L2*IK_L2) / (2.0 * IK_L1 * D);
    if (cos_beta < -1.0) cos_beta = -1.0;
    if (cos_beta > 1.0) cos_beta = 1.0;
    
    double beta = acos(cos_beta);  // Angle between L1 and line to target
    
    double theta_shoulder = alpha + beta;  // Shoulder angle from horizontal

    // 8. Calculate wrist angle to keep gripper horizontal
    // For horizontal gripper: wrist_angle = -(shoulder_angle + elbow_angle)
    double theta_wrist = -(theta_shoulder + theta_elbow);

    // 9. Convert to degrees
    out->base     = (int)(rad2deg(theta_base) + 0.5);
    out->shoulder = (int)(rad2deg(theta_shoulder) + 0.5);
    out->elbow    = (int)(rad2deg(theta_elbow) + 0.5);
    out->wrist    = (int)(rad2deg(theta_wrist) + 0.5);

    printf("IK Solution: base=%d°, shoulder=%d°, elbow=%d°, wrist=%d°\n",
           out->base, out->shoulder, out->elbow, out->wrist);

    return 0;
}