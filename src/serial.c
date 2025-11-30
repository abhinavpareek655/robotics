#define SIMULATION  // Comment this line to enable real serial communication
#include <windows.h>
#include <stdio.h>
#include "serial.h"

HANDLE hSerial;

int angleToPulse(int angle) {
    return 500 + (angle * 2000 / 180);
}

void initSerial(const char *port) {
    hSerial = CreateFile(port, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Error opening serial port!\n");
        return;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hSerial, &dcbSerialParams);

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;
    SetCommState(hSerial, &dcbSerialParams);
}

void sendJointAngles(RobotJoints j) {
    char cmd[128];

    sprintf(cmd,
        "#0 P%d #1 P%d #2 P%d #3 P%d\r",
        angleToPulse(j.base),
        angleToPulse(j.shoulder),
        angleToPulse(j.elbow),
        angleToPulse(j.wrist)
    );

#ifndef SIMULATION
    DWORD bytesWritten;
    WriteFile(hSerial, cmd, strlen(cmd), &bytesWritten, NULL);
#else
    printf("\nSIMULATED OUTPUT: %s\n", cmd);
#endif
}

