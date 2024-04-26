#include <util/crc16.h>

void setup() {
    Serial.begin(9600);
    char data[] = "1000/4/115200/5555/1000000"; // Change to character array
    size_t data_len = strlen(data);
    uint16_t crc = computeCRC(data, data_len);
    Serial.println(crc, HEX); // Print the CRC-16 checksum in hexadecimal
}

void loop() {}

uint16_t computeCRC(const char * data, size_t data_len) {
    uint16_t crc = 0xFFFF; // Initial value

    for (size_t i = 0; i < data_len; i++) {
        crc = _crc16_update(crc, data[i]); // Update CRC with next byte
    }

    return crc;
}
