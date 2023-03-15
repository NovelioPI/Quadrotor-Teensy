#include <sonar.h>

uint32_t loop_timer = 0;

void setup() {
    Serial.begin(115200);
    sonar_setup();
}

void loop() {
    sonar_loop();
    Serial.print("Distance:");
    Serial.print(alt, 4);
    Serial.print(" Velocity:");
    Serial.println(alt_rate, 4);
    
    if (micros() - loop_timer > 4050) Serial.println(micros() - loop_timer);                                      //Turn on the LED if the loop time exceeds 4050us.
    while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
    loop_timer = micros();   
}