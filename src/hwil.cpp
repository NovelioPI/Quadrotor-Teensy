#define dt 0.002f

#include <communication.h>

uint32_t loop_timer = 0;

void setup() {
  Serial.begin(9600);
  Serial7.begin(57600);

  delay(250);

  loop_timer = micros();
}

void loop() {
  if (receive_message()) {
    Serial7.print(msg.sensors.sonar.distance);
    Serial7.print(" ");
  }
  Serial7.println(micros() - loop_timer);
  while (micros() - loop_timer < 1000000) {}
  loop_timer = micros();
}