#include <imu.h>

#define UART Serial
#define BAUD_RATE 115200

void setup() {
    UART.begin(BAUD_RATE);
    delay(250);

    pinMode(LED_BUILTIN, OUTPUT);

    imu_setup();
}

void loop() {
    imu_loop();

    UART.print("Roll:");
    UART.print(roll);
    UART.print(" Pitch:");
    UART.print(pitch);
    UART.print(" Yaw:");
    UART.print(yaw);
    UART.print(" gyroX:");
    UART.print(gyroX);
    UART.print(" gyroY:");
    UART.print(gyroY);
    UART.print(" gyroZ:");
    UART.println(gyroZ);
}