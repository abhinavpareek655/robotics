#include <stdio.h>
#include <conio.h>

int clamp(int value) {
    if (value < 0) return 0;
    if (value > 180) return 180;
    return value;
}

int main() {
    int base = 90, shoulder = 90, elbow = 90, wrist = 90;
    char key;

    printf("Teleoperation Simulator\n");
    printf("Use Q/A, W/S, E/D, R/F to control joints.\n");

    while (1) {
        key = getch();

        if (key == 'x') break;

        switch(key) {
            case 'q': base += 2; break;
            case 'a': base -= 2; break;
            case 'w': shoulder += 2; break;
            case 's': shoulder -= 2; break;
            case 'e': elbow += 2; break;
            case 'd': elbow -= 2; break;
            case 'r': wrist += 2; break;
            case 'f': wrist -= 2; break;
        }

        // Apply limits
        base = clamp(base);
        shoulder = clamp(shoulder);
        elbow = clamp(elbow);
        wrist = clamp(wrist);

        // Simulated UI
        printf("\rBASE=%d  SHOULDER=%d  ELBOW=%d  WRIST=%d   ", 
               base, shoulder, elbow, wrist);
        fflush(stdout);
    }

    return 0;
}
