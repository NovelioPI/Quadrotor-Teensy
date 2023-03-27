#include <Arduino.h>

float k_alt = 3.0f;
float k_alt_rate = 1.0f;

float k_roll = 22.0f;
float k_roll_rate = 27.0;

float k_pitch = 20.0f;
float k_pitch_rate = 25.0f;

float k_yaw = 22.0f;
float k_yaw_rate = 9.0f;

float roll_ref, pitch_ref, yaw_ref, alt_ref = 75;
bool yaw_move = false, alt_move = false;
float roll_sensor, pitch_sensor, yaw_sensor, alt_sensor;
float roll_rate_sensor, pitch_rate_sensor, yaw_rate_sensor, alt_rate_sensor;
float roll_offset = 0.9, pitch_offset = -2.60, yaw_offset = 0, alt_offset = 0;
int thrust = 1500;
float u1, u2, u3, u4;

const double A_invers[4][4] = {{292600, -1300300, 1300300, 6283300},
                               {292600, -1300300, -1300300, -6283300},
                               {292600, 1300300, -1300300, 6283300},
                               {292600, 1300300, 1300300, -6283300}};

float omega2[4];

void control_fsfb()
{
    if (alt_hold_mode)
    {
        // dynamic altitude gain
        // float alt_error = abs(alt_sensor - alt_ref);
        // if (alt_error > 10.0f) {
        //     k_alt = (alt_error - 10.0f) / 20.0f;
        //     if (k_alt > 3.0f) k_alt = 3.0f;
        // }

        u1 = (-k_alt * (alt_sensor - alt_ref) / 1'000'000.0f) + (-k_alt_rate * (alt_rate_sensor) / 1'000'000.0f);
        u1 = constrain(u1, -0.001367f, 0.001367f);
        
        u2 = (-k_roll * (roll_sensor - roll_ref) / 10'000'000.0f) + (-k_roll_rate * (roll_rate_sensor) / 10'000'000.0f);
        u3 = (-k_pitch * (pitch_sensor - pitch_ref) / 10'000'000.0f) + (-k_pitch_rate * (pitch_rate_sensor) / 10'000'000.0f);
        u4 = (-k_yaw * (yaw_sensor - yaw_ref) / 10'000'000.0f) + (-k_yaw_rate * (yaw_rate_sensor) / 10'000'000.0f);


        omega2[0] = (A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4) + ch_throttle;
        omega2[1] = (A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4) + ch_throttle;
        omega2[2] = (A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4) + ch_throttle;
        omega2[3] = (A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4) + ch_throttle;
    }
    else
    {
        u1 = 0.0f;
        u2 = (-k_roll * (roll_sensor - roll_ref) / 10'000'000.0f) + (-k_roll_rate * (roll_rate_sensor) / 10'000'000.0f);
        u3 = (-k_pitch * (pitch_sensor - pitch_ref) / 10'000'000.0f) + (-k_pitch_rate * (pitch_rate_sensor) / 10'000'000.0f);
        u4 = (-k_yaw * (yaw_sensor - yaw_ref) / 10'000'000.0f) + (-k_yaw_rate * (yaw_rate_sensor) / 10'000'000.0f);

        if (ch_throttle > 1800)
            ch_throttle = 1800; // Limit the throttle to 1800us.

        omega2[0] = (A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4) + ch_throttle;
        omega2[1] = (A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4) + ch_throttle;
        omega2[2] = (A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4) + ch_throttle;
        omega2[3] = (A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4) + ch_throttle;
    }
}

void set_control_ref()
{
    roll_ref = (ch_roll - 1500) / 10.0f;   // limit roll_ref -50 to 50 deg
    pitch_ref = (ch_pitch - 1500) / 10.0f; // limit pitch_ref -50 to 50 deg

    if (ch_yaw > 1600)
    {
        yaw_ref = (ch_yaw - 1600) / 40.0f; // limit yaw_ref 0 to 10 deg
        yaw_offset = yaw;
    }
    else if (ch_yaw < 1400)
    {
        yaw_ref = (ch_yaw - 1400) / 40.0f; // limit yaw_ref -10 to 0 deg
        yaw_offset = yaw;
    }

    // if (alt_hold_mode)
    // {
    //     if (ch_throttle > 1600)
    //     {
    //         alt_ref = (ch_throttle - 1600) / 4.0f; // limit alt_ref 0 to 10 cm
    //         alt_offset = alt;
    //     }
    //     else if (ch_throttle < 1400)
    //     {
    //         alt_ref = (ch_throttle - 1400) / 4.0f; // limit alt_ref -10 to 0 cm
    //         alt_offset = alt;
    //     }
    // }
}