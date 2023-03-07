#include <Arduino.h>
#include <Servo.h>

#define MOTOR_1_PIN  3
#define MOTOR_2_PIN  4
#define MOTOR_3_PIN  5
#define MOTOR_4_PIN  6

Servo motor1, motor2, motor3, motor4;

void motor_setup() {
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);

    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    motor4.attach(MOTOR_4_PIN);
    
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    delay(1000);

    Serial.println("Motor setup complete");
}

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
}