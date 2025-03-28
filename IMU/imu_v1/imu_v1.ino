#include <Wire.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno085;
sh2_SensorValue_t sensorValue;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Starting BNO085 IMU...");

    if (!bno085.begin_I2C()) { 
        Serial.println("Failed to initialize BNO085! Check connections.");
        while (1);
    }

    Serial.println("BNO085 Initialized.");

    // Enable reports for Accelerometer and Rotation Vector (Quaternion for Yaw, Pitch, Roll)
    if (!bno085.enableReport(SH2_ACCELEROMETER)) {
        Serial.println("Failed to enable Accelerometer.");
    }
    if (!bno085.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("Failed to enable Rotation Vector.");
    }
}

void loop() {
    while (bno085.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                //Serial.print("Accel (m/s^2): ");
                //Serial.print(sensorValue.un.accelerometer.x, 2);
                //Serial.print(", ");
                //Serial.print(sensorValue.un.accelerometer.y, 2);
                //Serial.print(", ");
                //Serial.println(sensorValue.un.accelerometer.z, 2);
                break;

            case SH2_ROTATION_VECTOR: {
                float qW = sensorValue.un.rotationVector.real;
                float qX = sensorValue.un.rotationVector.i;
                float qY = sensorValue.un.rotationVector.j;
                float qZ = sensorValue.un.rotationVector.k;

                // Convert quaternion to Yaw, Pitch, Roll
                float yaw   = atan2(2.0 * (qW * qZ + qX * qY), 1.0 - 2.0 * (qY * qY + qZ * qZ)) * 180.0 / PI;
                float pitch = asin(2.0 * (qW * qY - qZ * qX)) * 180.0 / PI;
                float roll  = atan2(2.0 * (qW * qX + qY * qZ), 1.0 - 2.0 * (qX * qX + qY * qY)) * 180.0 / PI;

                Serial.print(" Yaw: "); Serial.print(yaw, 2);
                Serial.print(", Pitch: "); Serial.print(pitch, 2);
                Serial.print(", Roll: "); Serial.println(roll, 2);
                break;
            }
        }
    }
    
    delay(500); // Adjust delay as needed
}
