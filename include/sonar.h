#include <Arduino.h>

#define SONAR_TRIG 1
#define SONAR_ECHO 0
#define SOUND_SPEED 344.0f // m/s

uint32_t srf_timer = 0;
float echo_signal = 0;

float alt, alt_rate, alt_rate_slow, alt_rate_fast;
float prev_alt = 0;
const float sonar_offset = 5;
const int num_samples = 20;
float rotating_avg[num_samples] = {0}, total_avg = 0;
int rotating_location = 0;

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

    alt = (SOUND_SPEED * echo_signal * 0.0001) / 2.0f; // cm
    alt -= sonar_offset;

    if (alt < 0)
    {
        alt = 0;
    }
    else if (alt > 200)
    {
        alt = 200;
    }

    // rotating average
    float alt_rate_temp = (alt - prev_alt) / 0.002f;
    prev_alt = alt;

    total_avg -= rotating_avg[rotating_location];
    rotating_avg[rotating_location] = alt_rate_temp;
    total_avg += rotating_avg[rotating_location];
    rotating_location++;
    if (rotating_location >= num_samples) rotating_location = 0;
    alt_rate_fast = total_avg / (float)num_samples;

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