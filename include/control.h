#include <Arduino.h>

#define INTEGRAL_LIMIT 10.0f

float k_alt = 10.0f;
float k_alt_rate = 3.0f;

float k_roll = 30.0f;
float k_roll_rate = 33.0;
float k_roll_i = 100.0f;

float k_pitch = 30.0f;
float k_pitch_rate = 33.0f;
float k_pitch_i = k_roll_i;

float k_yaw = 22.0f;
float k_yaw_rate = 9.0f;

float roll_ref, pitch_ref, yaw_ref, alt_ref = 75;
bool yaw_move = false, alt_move = false;
float roll_sensor, pitch_sensor, yaw_sensor, alt_sensor;
float roll_rate_sensor, pitch_rate_sensor, yaw_rate_sensor, alt_rate_sensor;
float roll_offset = 1.0f, pitch_offset = 1.0f, yaw_offset = 0, alt_offset = 0;
float roll_integrator = 0, pitch_integrator = 0;
int thrust = 1500;
float u1, u2, u3, u4;

const double A_invers[4][4] = {{292600, -1300300, 1300300, 6283300},
                               {292600, -1300300, -1300300, -6283300},
                               {292600, 1300300, -1300300, 6283300},
                               {292600, 1300300, 1300300, -6283300}};

float omega2[4];

void control_fsfb()
{
    float alt_error = alt_sensor - alt_ref;
    float roll_error = roll_sensor - roll_ref;
    float pitch_error = pitch_sensor - pitch_ref;
    float yaw_error = yaw_sensor;

    roll_integrator += roll_error * dt;
    pitch_integrator += pitch_error * dt;
    roll_integrator = constrain(roll_integrator, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    pitch_integrator = constrain(pitch_integrator, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    if (ch_throttle < 1100) {
        roll_integrator = 0;
        pitch_integrator = 0;
    }

    if (alt_hold_mode) {
        u1 = ((-k_alt * alt_error) + (-k_alt_rate * alt_rate_sensor)) / 1'000'000.0f;
        u1 = constrain(u1, -0.001367f, 0.001367f);
    } else {
        u1 = 0.0f;
    }

    
    u2 = ((-k_roll * roll_error) + (-k_roll_rate * roll_rate_sensor) + (-k_roll_i + roll_integrator)) / 10'000'000.0f;
    u3 = ((-k_pitch * pitch_error) + (-k_pitch_rate * pitch_rate_sensor) + (-k_pitch_i + pitch_integrator)) / 10'000'000.0f;
    u4 = ((-k_yaw * yaw_error) + (-k_yaw_rate * yaw_rate_sensor)) / 10'000'000.0f;

    if (ch_throttle > 1800)
        ch_throttle = 1800; // Limit the throttle to 1800us.

    omega2[0] = (A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4);
    omega2[1] = (A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4);
    omega2[2] = (A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4);
    omega2[3] = (A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4);
}

void set_control_ref()
{
    roll_ref = (ch_roll - 1500) / 20.0f; 
    pitch_ref = (ch_pitch - 1500) / 20.0f;
}