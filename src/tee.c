#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include "joints.h"
#include "ik.h"
#include "serial.h"
#include "fk.h"
#include "fk_log.h"

#define M_PI 3.14159265358979323846

#define SERVO_MIN 0
#define SERVO_MAX 180

#define APPROACH_DELAY_MS 800
#define LOWER_DELAY_MS   400
#define GRIPPER_DELAY_MS 300
#define RETREAT_DELAY_MS 600

static const double approach_height = 0.05;
static const double gripper_offset = 0.02;
static const int gripper_servo_channel = 4;
static const int gripper_open_deg = 40;
static const int gripper_closed_deg = 115;

static int math_deg_to_servo(int math_deg, int servo_center) {
    return servo_center + math_deg;
}

static double deg2rad(double d) { 
    return d * M_PI / 180.0; 
}

static double rad2deg(double r) {
    return r * 180.0 / M_PI;
}

static int clamp_servo_angle(int a) {
    if (a < SERVO_MIN) return SERVO_MIN;
    if (a > SERVO_MAX) return SERVO_MAX;
    return a;
}

void sendServoJointAnglesSafe(const RobotJoints *math_joints) {
    if (!math_joints) return;
    
    // Convert math angles to servo angles
    // Standard AL5D mounting: servo 90° is typically the "zero" position
    RobotJoints servo;
    servo.base     = math_deg_to_servo(math_joints->base, 90);
    servo.shoulder = math_deg_to_servo(math_joints->shoulder, 90);
    servo.elbow    = math_deg_to_servo(math_joints->elbow, 90);
    servo.wrist    = math_deg_to_servo(math_joints->wrist, 90);
    
    // Clamp to safe range
    servo.base     = clamp_servo_angle(servo.base);
    servo.shoulder = clamp_servo_angle(servo.shoulder);
    servo.elbow    = clamp_servo_angle(servo.elbow);
    servo.wrist    = clamp_servo_angle(servo.wrist);

    sendJointAngles(servo);

    // Log FK for verification
    double x, y, z;
    if (forward_kinematics(math_joints, &x, &y, &z) == 0) {
        printf("FK: X=%.3f, Y=%.3f, Z=%.3f\n", x, y, z);
    }
    fk_log_step(&servo);
}

int move_to_xyz(double x, double y, double z) {
    printf("Moving to: X=%.3f, Y=%.3f, Z=%.3f\n", x, y, z);
    
    RobotJoints math_joints;
    int res = inverse_kinematics(x, y, z, &math_joints);
    if (res != 0) {
        printf("ERROR: IK failed for (%.3f, %.3f, %.3f) - code %d\n", x, y, z, res);
        return res;
    }

    printf("Math angles: base=%d°, shoulder=%d°, elbow=%d°, wrist=%d°\n",
           math_joints.base, math_joints.shoulder, math_joints.elbow, math_joints.wrist);

    sendServoJointAnglesSafe(&math_joints);
    return 0;
}

void set_gripper(int deg) {
    int sd = clamp_servo_angle(deg);
    int pulse = angleToPulse(sd);  // Use function from serial.c
    
    char cmd[64];
    sprintf(cmd, "#%d P%d T%d\r", gripper_servo_channel, pulse, GRIPPER_DELAY_MS);
    
#ifndef SIMULATION
    DWORD bytesWritten;
    extern HANDLE hSerial;
    WriteFile(hSerial, cmd, strlen(cmd), &bytesWritten, NULL);
#else
    printf("GRIPPER: %s\n", cmd);
#endif
    
    printf("Gripper: %d° (pulse %d)\n", sd, pulse);
    Sleep(GRIPPER_DELAY_MS);
}

int perform_tee_routine(double pickup_x, double pickup_y, double pickup_z,
                        double tee_x, double tee_y, double tee_z) {

    printf("\n=== Golf Ball Tee-ing Routine ===\n");
    printf("Pickup: (%.3f, %.3f, %.3f)\n", pickup_x, pickup_y, pickup_z);
    printf("Tee: (%.3f, %.3f, %.3f)\n\n", tee_x, tee_y, tee_z);

    // [1] Open gripper
    printf("[1] Opening gripper...\n");
    set_gripper(gripper_open_deg);

    // [2] Approach pickup
    printf("[2] Approaching pickup...\n");
    if (move_to_xyz(pickup_x, pickup_y, pickup_z + approach_height + gripper_offset) != 0)
        return -1;
    Sleep(APPROACH_DELAY_MS);

    // [3] Lower to pickup
    printf("[3] Lowering to pickup...\n");
    if (move_to_xyz(pickup_x, pickup_y, pickup_z + gripper_offset) != 0)
        return -2;
    Sleep(LOWER_DELAY_MS);

    // [4] Grab ball
    printf("[4] Grabbing ball...\n");
    set_gripper(gripper_closed_deg);

    // [5] Lift ball
    printf("[5] Lifting ball...\n");
    if (move_to_xyz(pickup_x, pickup_y, pickup_z + approach_height + gripper_offset) != 0)
        return -3;
    Sleep(RETREAT_DELAY_MS);

    // [6] Approach tee
    printf("[6] Approaching tee...\n");
    if (move_to_xyz(tee_x, tee_y, tee_z + approach_height + gripper_offset) != 0)
        return -4;
    Sleep(APPROACH_DELAY_MS);

    // [7] Lower to tee
    printf("[7] Lowering to tee...\n");
    if (move_to_xyz(tee_x, tee_y, tee_z + gripper_offset) != 0)
        return -5;
    Sleep(LOWER_DELAY_MS);

    // [8] Release ball
    printf("[8] Releasing ball...\n");
    set_gripper(gripper_open_deg);

    // [9] Retreat
    printf("[9] Retreating...\n");
    if (move_to_xyz(tee_x, tee_y, tee_z + approach_height + gripper_offset) != 0)
        return -6;
    Sleep(RETREAT_DELAY_MS);

    printf("\n=== Routine Complete! ===\n");
    return 0;
}