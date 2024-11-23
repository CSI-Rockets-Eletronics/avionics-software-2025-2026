#include <stdio.h>
#include "Arduino.h"

extern "C" void app_main()
{
    initArduino();

    printf("Hello, World!\n");
}
