#include <remote.h>
#include <motor.h>

bool use_remote = true;
int throttle = 1000;

char c;

void setup() {
    Serial.begin(115200);
    motor_setup();
    remote_setup();

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

void loop() {
    remote_loop();

    c = '0';
    if (Serial.available() > 0) {
        c = Serial.read();
    }

    if (use_remote) {
        throttle = ch_throttle;
    } else {
        if (c != '0') {
            if (c == 'w') {
                throttle += 1;
            } else if (c == 's') {
                throttle -= 1;
            } else if (c == ' ') {
                throttle = 1000;
            }
        }
    }

    if (c != '0') {
        if (c == '1') {
            use_remote = false;
        } else if (c == '2') {
            use_remote = true;
        }
    }

    motor_loop(throttle, throttle+3, throttle+1, throttle+2);

    Serial.print("Throttle: ");
    Serial.println(throttle);

    delay(100);
}