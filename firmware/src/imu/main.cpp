#include <Arduino.h>
#include <Adafruit_BNO08x.h>
Adafruit_BNO08x bno;

void setup() {
    Serial.begin(115200);

    // Wait up to 3 s for a computer to open the port, then start anyway. Without the timeout the
    // board sits frozen until something connects, so it can't start streaming from power-on.
    while (!Serial && millis() < 3000) delay(10);

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

    // The BNO085's own fused orientation, from the same gyro + accelerometer and no magnetometer:
    // the reference my filters are scored against.
    if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)){
        Serial.println("Could not enable game rotation vector report");
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
    }else if(value.sensorId == SH2_GAME_ROTATION_VECTOR){
        // w first, to match the Python side's [w, x, y, z]
        Serial.printf("%lu,q,%.6f,%.6f,%.6f,%.6f\n", t_us,
                      value.un.gameRotationVector.real, value.un.gameRotationVector.i,
                      value.un.gameRotationVector.j, value.un.gameRotationVector.k);
    }
}
