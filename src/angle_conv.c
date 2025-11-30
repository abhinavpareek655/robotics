#include "angle_conv.h"
#include <math.h>
#define M_PI 3.14159265358979323846

/* Per-joint calibration (example values). You *must* adjust these by measuring your robot. 
   offset_deg: add to math angle to get servo angle (or vice-versa depending on convention)
   direction: +1 or -1 to flip sign
*/
static int dir_base =  1;
static double offset_base_deg =  90.0;

static int dir_shoulder =  -1;
static double offset_shoulder_deg = 135.0;

static int dir_elbow =  1;      /* often elbow is inverted on AL5D */
static double offset_elbow_deg = 120.0;

static int dir_wrist =  -1;
static double offset_wrist_deg = 90.0;

void conv_init() {
    /* Optionally load these from a config file or set empirically */
    /* Keep defaults until you calibrate */
}

/* Helper: deg->rad and rad->deg */
static double deg2rad(double d){ return d * M_PI / 180.0; }
static double rad2deg(double r){ return r * 180.0 / M_PI; }

/* servo->math: servo is integer 0..180, math returned in radians */
void servo_to_math(const RobotJoints *servo, double *base_rad, double *shoulder_rad, double *elbow_rad, double *wrist_rad) {
    if (!servo) return;
    double b_deg = (double)servo->base;
    double s_deg = (double)servo->shoulder;
    double e_deg = (double)servo->elbow;
    double w_deg = (double)servo->wrist;

    /* convert using per-joint sign and offset:
       math_deg = (servo_deg - offset) * direction
       so if offset=90 and direction=1, servo 90 -> math 0
    */
    double base_math_deg = (b_deg - offset_base_deg) * dir_base;
    double shoulder_math_deg = (s_deg - offset_shoulder_deg) * dir_shoulder;
    double elbow_math_deg = (e_deg - offset_elbow_deg) * dir_elbow;
    double wrist_math_deg = (w_deg - offset_wrist_deg) * dir_wrist;

    *base_rad = deg2rad(base_math_deg);
    *shoulder_rad = deg2rad(shoulder_math_deg);
    *elbow_rad = deg2rad(elbow_math_deg);
    *wrist_rad = deg2rad(wrist_math_deg);
}

/* math->servo: take math radians and produce servo angles (0..180 ints) */
void math_to_servo(double base_rad, double shoulder_rad, double elbow_rad, double wrist_rad, RobotJoints *servo_out) {
    if (!servo_out) return;

    double base_deg_math = rad2deg(base_rad);
    double shoulder_deg_math = rad2deg(shoulder_rad);
    double elbow_deg_math = rad2deg(elbow_rad);
    double wrist_deg_math = rad2deg(wrist_rad);

    double servo_base = base_deg_math * dir_base + offset_base_deg;
    double servo_shoulder = shoulder_deg_math * dir_shoulder + offset_shoulder_deg;
    double servo_elbow = elbow_deg_math * dir_elbow + offset_elbow_deg;
    double servo_wrist = wrist_deg_math * dir_wrist + offset_wrist_deg;

    if (servo_base < 0) servo_base = 0; if (servo_base > 180) servo_base = 180;
    if (servo_shoulder < 0) servo_shoulder = 0; if (servo_shoulder > 180) servo_shoulder = 180;
    if (servo_elbow < 0) servo_elbow = 0; if (servo_elbow > 180) servo_elbow = 180;
    if (servo_wrist < 0) servo_wrist = 0; if (servo_wrist > 180) servo_wrist = 180;

    servo_out->base = (int)(servo_base + 0.5);
    servo_out->shoulder = (int)(servo_shoulder + 0.5);
    servo_out->elbow = (int)(servo_elbow + 0.5);
    servo_out->wrist = (int)(servo_wrist + 0.5);
}

/* SSC-32: 1500 uS center, approx 10 uS per degree (depends on servo). 
   Useful functions — check SSC-32 manual for exact mapping for your servos. */
int deg_to_pulse(int deg) {
    /* 1500uS is middle (90 deg in our servo-angle conv), 10uS per deg => pulse = 1500 + (deg - 90)*10 */
    return 1500 + (deg - 90) * 10;
}
int pulse_to_deg(int pulse) {
    return 90 + (pulse - 1500) / 10;
}
