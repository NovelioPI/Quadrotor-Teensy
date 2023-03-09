#include <Arduino.h>
#include <imu.h>

#define UART Serial
#define BAUD_RATE 57600

#define USE_PID

uint32_t loop_timer = 0, debug_timer = 0;

void setup() {
  UART.begin(BAUD_RATE);
  delay(250);

  pinMode(LED_BUILTIN, OUTPUT);

  imu_setup();

  loop_timer = micros();
}

void loop() {
  imu_loop();

  UART.print("Roll:");
  UART.print(angle_roll);
  UART.print(" Pitch:");
  UART.println(angle_pitch);

  if (micros() - loop_timer > 4050) UART.println(micros() - loop_timer);                                      //Turn on the LED if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}