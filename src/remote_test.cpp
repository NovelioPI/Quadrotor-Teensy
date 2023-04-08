#include <remote.h>

void setup() {
    Serial.begin(115200);

    remote_setup();

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

void loop() {
    remote_loop();
    
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    Serial.println();

    delay(100);
}