#include <Arduino.h>
#include <Adafruit_BNO08x.h>
Adafruit_BNO08x bno;

void setup() {
    Serial.begin(115200);
    
    while (!Serial){
        delay(10);
    }

    if(!bno.begin_I2C()){
        Serial.println("BNO085 not found");
        while (1) delay(10);
    }

    if (!bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000)){
        Serial.println("Could not enable uncalibrated gyro report");
        while (1) delay(10);
    }

    if (!bno.enableReport(SH2_ACCELEROMETER, 10000)){
        Serial.println("Could not enable accelerometer report");
        while (1) delay(10);
    }
}

void loop() {
    sh2_SensorValue_t value;
    if (!bno.getSensorEvent(&value)) return;

    // The Adafruit library never records when it read the report, so value.timestamp is only
    // "sample time minus read time": a few ms, negative. Adding micros() (≈ read time) gives the sample time.
    unsigned long t_us = micros() + (int32_t)value.timestamp;

    if (value.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
        Serial.printf("%lu,g,%.6f,%.6f,%.6f\n", t_us,
                      value.un.gyroscopeUncal.x, value.un.gyroscopeUncal.y, value.un.gyroscopeUncal.z);
    }else if(value.sensorId == SH2_ACCELEROMETER){
        Serial.printf("%lu,a,%.6f,%.6f,%.6f\n", t_us,
                      value.un.accelerometer.x, value.un.accelerometer.y, value.un.accelerometer.z);
    }
}
