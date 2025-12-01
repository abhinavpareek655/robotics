*A guide to understand, run, and modify the project.*

---

## **1. Project Structure (src/)**

```
src/
│
├── main.c           // Entry point – runs UI, teleop, or teach mode
│
├── input.c/h        // Keyboard input handling
├── ui.c/h           // Terminal display, menus, prompts
│
├── teleop.c/h       // Manual joint control + serial output
├── teach.c/h        // Record & replay joint sequences
│
├── joints.c/h       // Joint angle storage, limits, conversions
├── fk.c/h           // Forward kinematics (replace link lengths!)
├── ik.c/h           // Inverse kinematics solver
│
├── serial.c/h       // Serial communication (simulation or real hardware)
│
├── calibration_test.c   // Utility for testing servo mapping
├── fk_log.c, fk_log.h   // Logging support (optional)
│
└── recording.rec            // Teach-mode recordings
```

---

## **2. How to Run the Project**

### **Compile**

Use any GCC compiler:

```
gcc *.c -o robot.exe
```

### **Run**

```
./robot.exe
```

The UI allows you to choose:

* **Teleoperation**
* **Teach mode (Record/Replay)**
* **Kinematics tests**

---

## **3. Essential Settings to Update Before Using With Real Hardware**

### **3.1 Serial Port Selection (IMPORTANT)**

Update in **teleop.c** (line 36):

```c
initSerial("COM3");    // change this to your actual COM port
```

### **3.2 Disable Simulation Mode**

In **serial.c**, comment out line 1:

```c
// #define SIMULATION   // Comment this to enable real serial communication
```

This must be changed **when connecting to the actual AL5D controller**.

---

## **4. Robot Joint Mapping (Matches teleop.c)**

| Joint    | Function          | Index |
| -------- | ----------------- | ----- |
| Base     | Rotate around Z   | 0     |
| Shoulder | Main lifter joint | 1     |
| Elbow    | Middle arm        | 2     |
| Wrist    | End joint         | 3     |

These indices are used consistently in **teleop**, **teach mode**, **FK**, and **IK**.

---

## **5. Updating Kinematics for *Correct* AL5D Operation**

### **5.1 Replace Link Lengths in `fk.c`**

Replace link lengths with **measured AL5D link lengths**.

Look for comments like:
// TODO: replace with actual AL5D link lengths

Units **must be in meters**.

This ensures:

* Accurate forward kinematics
* Correct IK solutions
* Smooth positioning during replay

---

## **6. Teach Mode (recording.rec files)**

Teach mode records joint angles at fixed intervals into `recording.rec` file.

### **Record**

Teleop → Press the record key → Move joints → Save.

### **Replay**

```
Load recording → Robot follows saved joint path automatically
```

Used for your application (Golf Ball Tee-ing robot) as required. 

---

## **7. How to Operate the Robot Safely**

### Before running on real AL5D:

1. **Confirm COM port**
2. **Disable simulation** (`serial.c`)
3. **Reduce joint speed** first run
4. **Keep robot centered** to avoid hitting limits
5. **Test FK with low-speed increments**

---

## **8. What You Must Finalize for a “Perfect” Working System**

| Task                                   | File       |
| -------------------------------------- | ---------- |
| Update link lengths                    | `fk.c`     |
| Tune IK for reachable workspace        | `ik.c`     |
| Make sure teleop speed limits are safe | `teleop.c` |
| Confirm joint limits                   | `joints.c` |
| Set correct COM port                   | `teleop.c` |
| Disable SIMULATION                     | `serial.c` |

These are the only required fixes.
