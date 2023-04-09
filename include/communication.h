#include <Arduino.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "hwil.pb.h"

#define STX 0xFE
#define MAX_MESSAGE_SIZE (HWIL_msg_size + 4)

HWIL_msg msg = HWIL_msg_init_zero;

// Buffer Format: STX, message_size, message, checksum

void send_message(HWIL_msg &msg) {
    uint8_t buffer[MAX_MESSAGE_SIZE];
    buffer[0] = STX; // Start of message
    buffer[1] = 0;
    buffer[2] = 0;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer + 3, HWIL_msg_size);
    if (!pb_encode(&stream, HWIL_msg_fields, &msg)) {
        Serial.println("Failed to encode message");
        return;
    }


    uint8_t checksum = 0;
    for (uint8_t i = 0; i < stream.bytes_written; i++) {
        checksum ^= buffer[i + 3];
    }
    buffer[stream.bytes_written + 3] = checksum;
    
    buffer[1] = stream.bytes_written >> 8; // Message size
    buffer[2] = stream.bytes_written & 0xFF; // Message size

    Serial.write(buffer, stream.bytes_written + 4);
}

bool receive_message() {
    uint8_t buffer[MAX_MESSAGE_SIZE];
    uint8_t checksum = 0;
    uint8_t message_size = 0;

    while (Serial.available() > 0) {
        uint8_t byte = Serial.read();
        if (byte == STX) {
            message_size = Serial.read() << 8 | Serial.read();
            if (message_size > HWIL_msg_size) {
                Serial.println("Message too large");
                return false;
            }
            for (uint8_t i = 0; i < message_size; i++) {
                buffer[i] = Serial.read();
                checksum ^= buffer[i];
            }
            if (checksum != Serial.read()) {
                Serial.println("Checksum failed");
                return false;
            }
            break;
        }
    }

    pb_istream_t stream = pb_istream_from_buffer(buffer, message_size);
    msg = HWIL_msg_init_zero;
    if (!pb_decode(&stream, HWIL_msg_fields, &msg)) {
        Serial.println("Failed to decode message");
        return false;
    }

    return true;
}