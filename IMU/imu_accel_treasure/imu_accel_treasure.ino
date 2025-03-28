#include <Wire.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Offsets for accelerometer
float accel_offset_x = 0.2596, accel_offset_y = 0.0487, accel_offset_z = -0.0163;
float gravity_ref = 9.81;  // Reference gravity value

// Thresholds
const float threshold_x = 0.05;
const float threshold_y = 0.05;
const float threshold_z = 9.89;

void setup() {
  Serial.begin(115200);
  
  if (!bno08x.begin_I2C()) { 
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }

  Serial.println("BNO08x Found!");

  // Enable Accelerometer Report
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable Accelerometer");
    while (1);
  }

  Serial.println("Accelerometer Enabled");
}

void loop() {
  if (bno08x.getSensorEvent(&sensorValue)) {  
    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      // Read raw values
      float raw_x = sensorValue.un.accelerometer.x;
      float raw_y = sensorValue.un.accelerometer.y;
      float raw_z = sensorValue.un.accelerometer.z;

      // Apply zero-referencing correction
      float accel_x = raw_x - accel_offset_x;
      float accel_y = raw_y - accel_offset_y;
      float accel_z = raw_z - accel_offset_z;

      // Apply thresholds
      if (fabs(accel_x) < threshold_x) accel_x = 0.0;
      if (fabs(accel_y) < threshold_y) accel_y = 0.0;
      if (fabs(accel_z) < threshold_z) accel_z = 0.0;

      Serial.print("Accel X: "); Serial.print(accel_x, 4);
      Serial.print(" Y: "); Serial.print(accel_y, 4);
      Serial.print(" Z: "); Serial.println(accel_z, 4);
    }
  }
  
  delay(250);  
}
