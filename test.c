#include <stdio.h>
#include <conio.h>  

int main() {
    char key;

    printf("Press keys to control robot. Press X to exit.\n");

    while (1) {
        key = getch();  

        if (key == 'x' || key == 'X') break;
        
        printf("You pressed: %c\n", key);
    }

    return 0;
}
