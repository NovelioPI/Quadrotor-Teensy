#define dt 0.01f
#define HWIL

#include <remote.h>
#include <imu.h>
#include <sonar.h>
#include <control.h>
#include <motor.h>

uint32_t loop_timer = 0;

int pwm1, pwm2, pwm3, pwm4;

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(250);

  Serial.begin(6000000);
  Serial7.begin(57600);

  delay(250);

  remote_setup();
  alt_hold_mode = true;

  if (receive_message()) {
    if (msg.system_state == HWIL_msg_SystemState_POWERUP) {
      k_alt = msg.gain.z
      k_alt_rate = msg.gain.vz
      k_roll = msg.gain.roll
      k_roll_rate = msg.gain.p
      k_pitch = msg.gain.pitch
      k_pitch_rate = msg.gain.q
      k_yaw = msg.gain.yaw
      k_yaw_rate = msg.gain.r
  }

  digitalWrite(13, HIGH);
  loop_timer = micros();
}

void loop() {
  if (receive_message()) {
    imu_loop();
    sonar_loop();

    alt_sensor = alt;
    roll_sensor = roll;
    pitch_sensor = pitch;
    yaw_sensor = yaw;

    if (yaw_sensor > 180.0f) {
      yaw_sensor -= 360.0f;
    } else if (yaw_sensor < -180.0f) {
      yaw_sensor += 360.0f;
    }

    alt_rate_sensor = alt_rate;
    roll_rate_sensor = gyroX;
    pitch_rate_sensor = gyroY;
    yaw_rate_sensor = gyroZ;

    remote_loop();

    set_control_ref();

    control_fsfb();

    msg = HWIL_msg_init_zero;

    msg.command.u1 = u1;
    msg.command.u2 = u2;
    msg.command.u3 = u3;
    msg.command.u4 = u4;
    msg.command.motor1.pwm = (uint32_t) constrain(omega2[0] / 5.0f, 0, 250);
    msg.command.motor2.pwm = (uint32_t) constrain(omega2[1] / 5.0f, 0, 250);
    msg.command.motor3.pwm = (uint32_t) constrain(omega2[2] / 5.0f, 0, 250);
    msg.command.motor4.pwm = (uint32_t) constrain(omega2[3] / 5.0f, 0, 250);
    msg.command.has_u1 = true;
    msg.command.has_u2 = true;
    msg.command.has_u3 = true;
    msg.command.has_u4 = true;
    msg.has_command = true;

    msg.state.roll = roll_sensor;
    msg.state.pitch = pitch_sensor;
    msg.state.yaw = yaw_sensor;
    msg.state.z = alt_sensor;
    msg.state.has_roll = true;
    msg.state.has_pitch = true;
    msg.state.has_yaw = true;
    msg.state.has_z = true;
    msg.has_state = true;

    send_message(msg);
  }

  while (micros() - loop_timer < 10000) {}
  loop_timer = micros();
}