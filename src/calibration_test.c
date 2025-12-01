#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include "joints.h"
#include "ik.h"
#include "fk.h"
#include "angle_conv.h"
#include "serial.h"

#define M_PI 3.14159265358979323846

void print_menu() {
    printf("\n=== AL5D Calibration & Test Menu ===\n");
    printf("1. Test FK at current position\n");
    printf("2. Test IK for specific XYZ\n");
    printf("3. Move to safe home position\n");
    printf("4. Test reachability at multiple points\n");
    printf("5. Measure actual link lengths (manual)\n");
    printf("0. Exit\n");
    printf("Choice: ");
}

void test_fk() {
    RobotJoints servo;
    printf("\nEnter servo angles (0-180):\n");
    printf("Base: "); scanf("%d", &servo.base);
    printf("Shoulder: "); scanf("%d", &servo.shoulder);
    printf("Elbow: "); scanf("%d", &servo.elbow);
    printf("Wrist: "); scanf("%d", &servo.wrist);

    // Convert to math angles
    double base_rad, sh_rad, el_rad, wr_rad;
    servo_to_math(&servo, &base_rad, &sh_rad, &el_rad, &wr_rad);
    
    printf("\nMath angles (degrees): base=%.1f, shoulder=%.1f, elbow=%.1f, wrist=%.1f\n",
           base_rad * 180.0/M_PI, sh_rad * 180.0/M_PI, 
           el_rad * 180.0/M_PI, wr_rad * 180.0/M_PI);

    RobotJoints mathJ;
    mathJ.base = (int)(base_rad * 180.0/M_PI);
    mathJ.shoulder = (int)(sh_rad * 180.0/M_PI);
    mathJ.elbow = (int)(el_rad * 180.0/M_PI);
    mathJ.wrist = (int)(wr_rad * 180.0/M_PI);

    double x, y, z;
    if (forward_kinematics(&mathJ, &x, &y, &z) == 0) {
        printf("FK Result: X=%.4f m, Y=%.4f m, Z=%.4f m\n", x, y, z);
        printf("FK Result: X=%.1f cm, Y=%.1f cm, Z=%.1f cm\n", 
               x*100, y*100, z*100);
    } else {
        printf("FK calculation failed!\n");
    }
}

void test_ik() {
    double x, y, z;
    printf("\nEnter target coordinates (in meters):\n");
    printf("X: "); scanf("%lf", &x);
    printf("Y: "); scanf("%lf", &y);
    printf("Z: "); scanf("%lf", &z);

    RobotJoints ikMath;
    int result = inverse_kinematics(x, y, z, &ikMath);
    
    if (result == 0) {
        printf("\nIK Solution (math angles):\n");
        printf("  Base: %d°\n", ikMath.base);
        printf("  Shoulder: %d°\n", ikMath.shoulder);
        printf("  Elbow: %d°\n", ikMath.elbow);
        printf("  Wrist: %d°\n", ikMath.wrist);

        RobotJoints servo;
        math_to_servo(ikMath.base * M_PI/180.0,
                      ikMath.shoulder * M_PI/180.0,
                      ikMath.elbow * M_PI/180.0,
                      ikMath.wrist * M_PI/180.0,
                      &servo);

        printf("\nServo angles:\n");
        printf("  Base: %d\n", servo.base);
        printf("  Shoulder: %d\n", servo.shoulder);
        printf("  Elbow: %d\n", servo.elbow);
        printf("  Wrist: %d\n", servo.wrist);

        printf("\nSend to robot? (y/n): ");
        char c;
        scanf(" %c", &c);
        if (c == 'y' || c == 'Y') {
            sendJointAngles(servo);
            printf("Command sent!\n");
        }
    } else {
        printf("\nIK FAILED with code %d\n", result);
    }
}

void move_home() {
    printf("\nMoving to safe home position...\n");
    RobotJoints home = {90, 90, 90, 90};  // All servos at 90°
    sendJointAngles(home);
    printf("Home position sent.\n");
}

void test_reachability() {
    printf("\n=== Testing Reachability ===\n");
    
    // Test points in a grid
    double test_points[][3] = {
        {0.15, 0.0, 0.10},   // Front center, low
        {0.20, 0.0, 0.15},   // Front center, medium
        {0.25, 0.0, 0.20},   // Front center, high
        {0.15, 0.10, 0.10},  // Front right, low
        {0.15, -0.10, 0.10}, // Front left, low
        {0.30, 0.0, 0.10},   // Far front
        {0.10, 0.0, 0.10},   // Close front
    };
    
    int num_tests = sizeof(test_points) / sizeof(test_points[0]);
    int success_count = 0;
    
    for (int i = 0; i < num_tests; i++) {
        double x = test_points[i][0];
        double y = test_points[i][1];
        double z = test_points[i][2];
        
        RobotJoints ikMath;
        int result = inverse_kinematics(x, y, z, &ikMath);
        
        printf("\nPoint %d: (%.3f, %.3f, %.3f) => ", i+1, x, y, z);
        if (result == 0) {
            printf("REACHABLE ✓\n");
            printf("  Angles: base=%d°, shoulder=%d°, elbow=%d°, wrist=%d°\n",
                   ikMath.base, ikMath.shoulder, ikMath.elbow, ikMath.wrist);
            success_count++;
        } else {
            printf("UNREACHABLE ✗ (error code %d)\n", result);
        }
    }
    
    printf("\n%d / %d points reachable\n", success_count, num_tests);
}

void measure_links() {
    printf("\n=== Link Length Measurement Guide ===\n");
    printf("\nMeasure these distances on your robot:\n");
    printf("1. BASE_HEIGHT: From base rotation axis to shoulder rotation axis\n");
    printf("2. L1: From shoulder axis to elbow axis\n");
    printf("3. L2: From elbow axis to wrist axis\n");
    printf("4. L3: From wrist axis to gripper tip\n");
    printf("\nCurrent values (in meters):\n");
    printf("  BASE_HEIGHT = 0.069 m (69 mm)\n");
    printf("  L1 = 0.146 m (146 mm)\n");
    printf("  L2 = 0.187 m (187 mm)\n");
    printf("  L3 = 0.100 m (100 mm)\n");
    printf("\nTo update: Call ik_set_links() and fk_set_links() in your main code\n");
}

int main() {
    printf("=== AL5D Robot Arm Calibration Tool ===\n");
    
    // Initialize serial (simulation mode by default)
    initSerial("COM3");
    
    // Initialize kinematics with default AL5D measurements
    ik_set_links(0.069, 0.146, 0.187, 0.100);
    fk_set_links(0.069, 0.146, 0.187, 0.100);
    conv_init();
    
    int choice;
    do {
        print_menu();
        scanf("%d", &choice);
        
        switch(choice) {
            case 1: test_fk(); break;
            case 2: test_ik(); break;
            case 3: move_home(); break;
            case 4: test_reachability(); break;
            case 5: measure_links(); break;
            case 0: printf("Exiting...\n"); break;
            default: printf("Invalid choice!\n");
        }
    } while (choice != 0);
    
    return 0;
}