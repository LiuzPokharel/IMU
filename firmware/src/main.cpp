#include <Arduino.h>
#include <Wire.h>


void setup() {
    Serial.begin(115200);

    while (!Serial){
        delay(10);
    }

    Wire.begin();

}

void loop() {
    int counter = 0; 
    
    for (uint8_t addr = 0x08; addr <= 0x77; addr++){
        Wire.beginTransmission(addr);
        uint8_t result = Wire.endTransmission();
            if (result == 0){
                Serial.printf("Found device at 0x%02X\n", addr);
                counter++;
            }
    }
    Serial.printf("Devices found: %d\n", counter);
    delay(2000);
}
