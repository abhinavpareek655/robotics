/src
 ├─ input.c         (keyboard handling)
 ├─ joints.c        (joint angle storage + limits)
 ├─ teleop.c        (teleoperation logic)
 ├─ serial.c        (placeholder for now)
 ├─ ui.c            (terminal display)
 └─ main.c          (application entry)

##teleoperation:
change this line 24 in teleop.c : initSerial("COM3");    // change COM port if needed
comment the line 1 in serial.c when connected to the actual hardware: #define SIMULATION  // Comment this line to enable real serial communication

Joint mapping (matches your teleop code):
base — rotation about vertical Z (degrees) (joint 0)
shoulder — first arm joint (degrees) (joint 1)
elbow — second arm joint (degrees) (joint 2)
wrist — third arm joint (degrees) (joint 3) — treated as an additional link rotation in the same plane for FK

Link lengths are configurable constants (example values included). You must replace them with your AL5D measured link lengths for accurate results.
to do this you must change these values in fk.c where they are mentioned using comments
Units: meters.

