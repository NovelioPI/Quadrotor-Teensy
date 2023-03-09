#include <Arduino.h>

#define SONAR_TRIG 1
#define SONAR_ECHO 0

float distance, velocity;
float last_distance = 0;
const float sonar_offset = 0.0357;

void sonar_setup() {
    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
}

void sonar_loop() {
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    float duration = pulseIn(SONAR_ECHO, HIGH);
    distance = duration * 0.017; // 0.034 / 2 (m)
    distance -= sonar_offset;
    distance = constrain(distance, 0, 400);

    // Velocity
    velocity = (distance - last_distance) / 0.01;

    last_distance = distance;
}