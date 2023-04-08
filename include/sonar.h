#include <Arduino.h>

#define SONAR_TRIG 1
#define SONAR_ECHO 0
#define SOUND_SPEED 344.0f // m/s

uint32_t srf_timer = 0;
float echo_signal = 0;

float alt, alt_slow, alt_fast;
float alt_rate, alt_rate_slow, alt_rate_fast;
float prev_alt = 0;
const float sonar_offset = 5;
const int num_samples = 20;
float rotating_avg[num_samples][2] = {0}, total_avg[2] = {0};
int rotating_location[2] = {0};

void echo_rising();
void echo_falling();

void sonar_setup()
{
    pinMode(SONAR_ECHO, INPUT);
    pinMode(SONAR_TRIG, OUTPUT);

    srf_timer = micros();

    attachInterrupt(SONAR_ECHO, echo_rising, RISING);
    
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);

    prev_alt = (SOUND_SPEED * echo_signal * 0.0001) / 2.0f; // cm
}

void sonar_loop()
{
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);

    float alt_temp = (SOUND_SPEED * echo_signal * 0.0001) / 2.0f; // cm
    alt_temp -= sonar_offset;

    if (abs(alt_temp - prev_alt) > 10) alt_temp = prev_alt;

    // rolling average
    total_avg[0] -= rotating_avg[rotating_location[0]][0];
    rotating_avg[rotating_location[0]][0] = alt_temp;
    total_avg[0] += rotating_avg[rotating_location[0]][0];
    rotating_location[0]++;
    if (rotating_location[0] >= num_samples) rotating_location[0] = 0;
    alt_fast = total_avg[0] / (float)num_samples;

    alt_slow = alt_slow * 0.99f + alt_fast * 0.01f;
    float alt_diff = alt_slow - alt_fast;
    alt_diff = constrain(alt_diff, -10.0f, 10.0f);
    if (alt_diff > 1 || alt_diff < -1) alt_slow -= alt_diff;
    alt = alt_slow;

    alt = constrain(alt, 0.0f, 200.0f);

    // rolling average
    float alt_rate_temp = (alt - prev_alt) / dt;
    prev_alt = alt;

    total_avg[1] -= rotating_avg[rotating_location[1]][1];
    rotating_avg[rotating_location[1]][1] = alt_rate_temp;
    total_avg[1] += rotating_avg[rotating_location[1]][1];
    rotating_location[1]++;
    if (rotating_location[1] >= num_samples) rotating_location[1] = 0;
    alt_rate_fast = total_avg[1] / (float)num_samples;

    alt_rate_slow = alt_rate_slow * 0.98f + alt_rate_fast * 0.02f;
    alt_rate = alt_rate_slow;
}

void echo_rising()
{
    detachInterrupt(SONAR_ECHO);
    srf_timer = micros();
    attachInterrupt(SONAR_ECHO, echo_falling, FALLING);
}

void echo_falling()
{
    detachInterrupt(SONAR_ECHO);
    echo_signal = micros() - srf_timer;
    attachInterrupt(SONAR_ECHO, echo_rising, RISING);
}