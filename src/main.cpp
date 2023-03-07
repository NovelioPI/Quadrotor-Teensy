#include <Arduino.h>
#include <imu.h>
#include <remote.h>
#include <control.h>
#include <motor.h>

#define UART Serial7
#define BAUD_RATE 57600

#define USE_PID

uint32_t loop_timer = 0, debug_timer = 0;

int pwm1, pwm2, pwm3, pwm4;

void debug(uint32_t time);
void tuning();

void setup() {
  UART.begin(BAUD_RATE);
  delay(250);

  pinMode(LED_BUILTIN, OUTPUT);

  remote_setup();
  imu_setup();
  motor_setup();

  digitalWrite(LED_BUILTIN, LOW);
  UART.println("Starting loop...");
  #ifdef USE_PID
    UART.println("Using PID");
  #else
    UART.println("Using FSFB");
  #endif
  loop_timer = micros();
}

void loop() {
  remote_loop();
  imu_loop();

  if (!arming) {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;

    #ifdef USE_PID
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
    #endif
  }

  #ifdef USE_PID
    //The PID set point in degrees per second is determined by the roll receiver input.
    //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_roll_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (ch_roll > 1508)pid_roll_setpoint = ch_roll - 1508;
    else if (ch_roll < 1492)pid_roll_setpoint = ch_roll - 1492;

    pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;
    roll_input = gyro_roll_input;
    
    //The PID set point in degrees per second is determined by the pitch receiver input.
    //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_pitch_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (ch_pitch > 1508)pid_pitch_setpoint = ch_pitch - 1508;
    else if (ch_pitch < 1492)pid_pitch_setpoint = ch_pitch - 1492;

    pid_pitch_setpoint -= pitch_level_adjust;                                       //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                                                      //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.
    pitch_input = gyro_pitch_input;

    //The PID set point in degrees per second is determined by the yaw receiver input.
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_yaw_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (ch_throttle > 1050) { //Do not yaw when turning off the motors.
      if (ch_yaw > 1508)pid_yaw_setpoint = (ch_yaw - 1508) / 3.0;
      else if (ch_yaw < 1492)pid_yaw_setpoint = (ch_yaw - 1492) / 3.0;            //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
    }
    yaw_input = gyro_yaw_input;

    control_pid();
  #else
    
    control_fsfb();
  #endif

  if (arming) {
    if (ch_throttle > 1800) ch_throttle = 1800; //Limit the throttle to 1800us.
    #ifdef USE_PID
      pwm1 = ch_throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
      pwm2 = ch_throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
      pwm3 = ch_throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
      pwm4 = ch_throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

      pwm1 = constrain(pwm1, 1100, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
      pwm2 = constrain(pwm2, 1100, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
      pwm3 = constrain(pwm3, 1100, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
      pwm4 = constrain(pwm4, 1100, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
    #else
      pwm1 = (int) omega2[0] + ch_throttle;                                                  //Calculate the pulse for esc 1 (front-right - CCW).
      pwm2 = (int) omega2[1] + ch_throttle;                                                  //Calculate the pulse for esc 2 (rear-right - CW).
      pwm3 = (int) omega2[2] + ch_throttle;                                                  //Calculate the pulse for esc 3 (rear-left - CCW).
      pwm4 = (int) omega2[3] + ch_throttle;                                                  //Calculate the pulse for esc 4 (front-left - CW).

      pwm1 = constrain(pwm1, 1000, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
      pwm2 = constrain(pwm2, 1000, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
      pwm3 = constrain(pwm3, 1000, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
      pwm4 = constrain(pwm4, 1000, 2000);                                                    //Constrain the pulse between 1000 and 2000us.
    #endif
  }
  else {
    pwm1 = 1000;
    pwm2 = 1000;
    pwm3 = 1000;
    pwm4 = 1000;
  }

  motor_loop(pwm1, pwm2, pwm3, pwm4);


  if (micros() - debug_timer > 100000) {
    tuning();
    // debug(loop_timer);
    // debug_timer = micros();
  }

  if (micros() - loop_timer > 4050) UART.println(micros() - loop_timer);                                      //Turn on the LED if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}

void debug(uint32_t time) {
  String str = "";
  str += "Roll:";
  str += angle_roll;
  str += " Pitch:";
  str += angle_pitch;
  str += " Yaw:";
  str += gyro_yaw_input;
  str += " Throttle:";
  str += ch_throttle;
  str += " PWM:";
  str += pwm1;
  str += " ";
  str += pwm2;
  str += " ";
  str += pwm3;
  str += " ";
  str += pwm4;
  str += " Kroll:";
  str += pid_p_gain_roll;
  str += " ";
  str += pid_i_gain_roll;
  str += " ";
  str += pid_d_gain_roll;
  str += " Kpitch:";
  str += pid_p_gain_pitch;
  str += " ";
  str += pid_i_gain_pitch;
  str += " ";
  str += pid_d_gain_pitch;
  str += " Kyaw:";
  str += pid_p_gain_yaw;
  str += " ";
  str += pid_i_gain_yaw;
  str += " ";
  str += pid_d_gain_yaw;
  
  UART.println(str);
}

void tuning() {
  if (UART.available()) {
    char inChar = (char)UART.read();
    switch (inChar)
    {
    case 'a':
      pid_p_gain_roll += 0.1;
      UART.print("kp roll = "); UART.println(pid_p_gain_roll);
      break;
    case 'z':
      pid_p_gain_roll -= 0.1;
      UART.print("kp roll = "); UART.println(pid_p_gain_roll);
      break;
    case 's':
      pid_i_gain_roll += 0.01;
      UART.print("ki roll = "); UART.println(pid_i_gain_roll);
      break;
    case 'x':
      pid_i_gain_roll -= 0.01;
      UART.print("ki roll = "); UART.println(pid_i_gain_roll);
      break;
    case 'd':
      pid_d_gain_roll += 0.1;
      UART.print("kd roll = "); UART.println(pid_d_gain_roll);
      break;
    case 'c':
      pid_d_gain_roll -= 0.1;
      UART.print("kd roll = "); UART.println(pid_d_gain_roll);
      break;
    case 'f':
      pid_p_gain_pitch += 0.1;
      UART.print("kp pitch = "); UART.println(pid_p_gain_pitch);
      break;
    case 'v':
      pid_p_gain_pitch -= 0.1;
      UART.print("kp pitch = "); UART.println(pid_p_gain_pitch);
      break;
    case 'g':
      pid_i_gain_pitch += 0.01;
      UART.print("ki pitch = "); UART.println(pid_i_gain_pitch);
      break;
    case 'b':
      pid_i_gain_pitch -= 0.01;
      UART.print("ki pitch = "); UART.println(pid_i_gain_pitch);
      break;
    case 'h':
      pid_d_gain_pitch += 0.1;
      UART.print("kd pitch = "); UART.println(pid_d_gain_pitch);
      break;
    case 'n':
      pid_d_gain_pitch -= 0.1;
      UART.print("kd pitch = "); UART.println(pid_d_gain_pitch);
      break;
    case 'j':
      pid_p_gain_yaw += 0.1;
      UART.print("kp yaw = "); UART.println(pid_p_gain_yaw);
      break;
    case 'm':
      pid_p_gain_yaw -= 0.1;
      UART.print("kp yaw = "); UART.println(pid_p_gain_yaw);
      break;
    case 'k':
      pid_i_gain_yaw += 0.01;
      UART.print("ki yaw = "); UART.println(pid_i_gain_yaw);
      break;
    case ',':
      pid_i_gain_yaw -= 0.01;
      UART.print("ki yaw = "); UART.println(pid_i_gain_yaw);
      break;
    case 'l':
      pid_d_gain_yaw += 0.1;
      UART.print("kd yaw = "); UART.println(pid_d_gain_yaw);
      break;
    case '.':
      pid_d_gain_yaw -= 0.1;
      UART.print("kd yaw = "); UART.println(pid_d_gain_yaw);
      break;
    default:
      break;
    }
  }
}