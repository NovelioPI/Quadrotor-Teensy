#include <Arduino.h>

#define dt 0.002f

#include <remote.h>
#include <motor.h>
#include <imu.h>
#include <sonar.h>
#include <control.h>

#define UART Serial7
#define BAUD_RATE 57600 // 57600

uint32_t loop_timer = 0, debug_timer = 0;

int pwm1, pwm2, pwm3, pwm4;

void debug();
void tuning();

uint8_t* addSndBuf;

void setup()
{
  Serial.begin(115200);
  UART.begin(BAUD_RATE);
  
  delay(250);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  remote_setup();
  motor_setup();
  imu_setup();
  sonar_setup();

  digitalWrite(LED_BUILTIN, LOW);
  UART.println("Starting loop...");

  UART.println("roll pitch yaw alt");
  loop_timer = micros();
}

void loop()
{
  imu_loop();
  sonar_loop();

  roll_sensor = roll - roll_offset;
  pitch_sensor = pitch - pitch_offset;
  yaw_sensor = yaw - yaw_offset;
  alt_sensor = alt - alt_offset;

  roll_rate_sensor = gyroX;
  pitch_rate_sensor = gyroY;
  yaw_rate_sensor = gyroZ;
  alt_rate_sensor = alt_rate;

  remote_loop();

  set_control_ref();

  control_fsfb();

  if (arming)
  {
    pwm1 = (int)(omega2[0]) + ch_throttle; // Calculate the pulse for esc 1 (front-right - CCW).
    pwm2 = (int)(omega2[1]) + ch_throttle; // Calculate the pulse for esc 2 (rear-right - CW).
    pwm3 = (int)(omega2[2]) + ch_throttle; // Calculate the pulse for esc 3 (rear-left - CCW).
    pwm4 = (int)(omega2[3]) + ch_throttle; // Calculate the pulse for esc 4 (front-left - CW).
  }
  else
  {
    yaw_offset = yaw;

    pwm1 = 1000;
    pwm2 = 1000;
    pwm3 = 1000;
    pwm4 = 1000;
  }

  motor_loop(pwm1, pwm2 + 3, pwm3 + 1, pwm4 + 2);

  if (arming && micros() - debug_timer > 20000) {
    debug();
    debug_timer = micros();
  }

  while (micros() - loop_timer < 2000);
  Serial.println(micros() - loop_timer);
  loop_timer = micros();
}

void debug()
{
  UART.printf("%.3f %.3f %.3f %.3f\r\n", roll_sensor, pitch_sensor, yaw_sensor, alt_sensor);
}