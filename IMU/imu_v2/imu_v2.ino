#include <Wire.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno085;
sh2_SensorValue_t sensorValue;

// Variables for integration
float velocityX = 0.0;  // Velocity in X direction (m/s)
float positionX = 0.0;  // Distance in X direction (m)
unsigned long prevTime = 0;  // Timestamp for integration

const float ACCEL_THRESHOLD = 0.85; // Ignore small noise

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Starting BNO085 IMU...");

    if (!bno085.begin_I2C()) { 
        Serial.println("Failed to initialize BNO085! Check connections.");
        while (1);
    }

    Serial.println("BNO085 Initialized.");

    // Enable Accelerometer
    if (!bno085.enableReport(SH2_ACCELEROMETER)) {
        Serial.println("Failed to enable Accelerometer.");
    }

    prevTime = millis(); // Initialize time
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    while (bno085.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ACCELEROMETER) {
            float accelX = sensorValue.un.accelerometer.x; // Acceleration in X (m/s^2)

            // Remove small noise
            if (fabs(accelX) < ACCEL_THRESHOLD) {
                accelX = 0.0;
            }

            // Integrate acceleration to get velocity
            velocityX += accelX * dt;

            // Integrate velocity to get position (distance)
            positionX += velocityX * dt;

            // Print results
            Serial.print("Accel X: "); Serial.print(accelX, 4);
            Serial.print(" m/s², Velocity X: "); Serial.print(velocityX, 4);
            Serial.print(" m/s, Distance X: "); Serial.print(positionX, 4);
            Serial.println(" m");
        }
    }

    delay(100); // Adjust sampling rate
}
