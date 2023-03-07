#include <Arduino.h>

float pid_p_gain_roll = 1.0;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.0;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 6.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, yaw_input, pid_output_yaw, pid_last_yaw_d_error;

void control_pid() {
    //Roll calculations
    pid_error_temp = roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
    else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
    else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

    pid_last_roll_d_error = pid_error_temp;

    //Pitch calculations
    pid_error_temp = pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
    else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
    else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

    pid_last_pitch_d_error = pid_error_temp;

    //Yaw calculations
    pid_error_temp = yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
    else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
    else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

    pid_last_yaw_d_error = pid_error_temp;
}


// ================================  FSFB  ================================

float k_alt         = 0.0;
float k_z_vel       = 1.0f;

float k_roll        = 37.0f;
float k_roll_rate   = 380.0f;
float k_i_roll      = 0.04f;

float k_pitch       = 37.0f; 
float k_pitch_rate  = 380.0f; 
float k_i_pitch     = 0.04f;

float k_yaw         = 600.0f;
float k_yaw_rate    = 0.0f;
float k_i_yaw       = 0.06f;

float roll_ref, pitch_ref, yaw_ref;
float roll_sensor, pitch_sensor, yaw_sensor, roll_rate_sensor, pitch_rate_sensor, yaw_rate_sensor;
float heading_now, last_heading;
float u1, u2, u3, u4;

const double A_invers[4][4] = {{-292600, 1300300, 1300300, 6283300},
                               {-292600, 1300300, -1300300, -6283300},
                               {-292600, -1300300, -1300300, 6283300},
                               {-292600, -1300300, 1300300, -6283300}};

float omega2[4];

void control_fsfb() {
    float target = heading_now - last_heading;
    if(heading_now < last_heading){
      if (target < 360.0f-target){
        yaw_sensor = target;
      } else if (360.0f-target < target){
        yaw_sensor = -(360.0f-target);
      }
    } else if(heading_now > last_heading){
      if(target < 360.0f-target){
        yaw_sensor = -(target);
      } else if(target > 360.0f-target){
        yaw_sensor = 360.0f-target;
      }
    } else {
      yaw_sensor = target;
    }

    u1 = /*(-gain.k_alt*(alt_target/1.000f) + (-gain.k_z_vel*(z_velocity/100.0f)))*/0.0/*-gain.k_alt*(rc_copter.channel_throttle->get_radio_in()/A_invers[0][0])*M_CONST*/;
    u2 = (-k_roll * (roll_sensor-roll_ref) / 10'000'000.0f) + (-k_roll_rate * (roll_rate_sensor) / 10'000'000.0f);
    u3 = (-k_pitch * (pitch_sensor-pitch_ref) / 10'000'000.0f) + (-k_pitch_rate * (pitch_rate_sensor) / 10'000'000.0f);
    u4 = (-k_yaw * (yaw_sensor-yaw_ref) / 10'000'000.0f) + (-k_yaw_rate * (yaw_rate_sensor) / 10'000'000.0f);

    omega2[0] = (A_invers[0][0]*u1 + A_invers[0][1]*u2 + A_invers[0][2]*u3 + A_invers[0][3]*u4);
    omega2[1] = (A_invers[1][0]*u1 + A_invers[1][1]*u2 + A_invers[1][2]*u3 + A_invers[1][3]*u4);
    omega2[2] = (A_invers[2][0]*u1 + A_invers[2][1]*u2 + A_invers[2][2]*u3 + A_invers[2][3]*u4);
    omega2[3] = (A_invers[3][0]*u1 + A_invers[3][1]*u2 + A_invers[3][2]*u3 + A_invers[3][3]*u4);

    last_heading = heading_now;
}