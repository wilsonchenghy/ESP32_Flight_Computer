#include <Adafruit_BNO08x.h>

/*
  BNO085 Sensor Connection (SPI Mode):

  VCC  -> 3.3V
  GND  -> GND
  SCL  -> D18
  SDA  -> D19
  AD0  -> D23
  CS   -> D5
  INT  -> D22
  RST  -> D4
  PS1  -> 3.3V
  PS0  -> 3.3V
*/

#define BNO08X_CS     5
#define BNO08X_INT    22
#define BNO08X_RESET  4

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float current_roll = 0, current_pitch = 0, current_yaw = 0;
float roll_offset = 0, pitch_offset = 0, yaw_offset = 0;
bool isCalibrated = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Starting BNO085...");

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }

  Serial.println("BNO085 Found!");

  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector!");
    while (1) delay(10);
  }

  delay(200);  // Give time to stabilize
}

void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset. Re-enabling reports...");
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
    isCalibrated = false;
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    float w = sensorValue.un.gameRotationVector.real;
    float x = sensorValue.un.gameRotationVector.i;
    float y = sensorValue.un.gameRotationVector.j;
    float z = sensorValue.un.gameRotationVector.k;

    // Convert Quaternion to Euler angles (degrees)
    current_roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * 180.0f / PI;
    current_pitch = asinf (2.0f * (w * y - z * x)) * 180.0f / PI;
    current_yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 180.0f / PI;

    if (!isCalibrated) {
      calibrateZero();
      isCalibrated = true;
    }

    float roll_zeroed  = wrapAngle(current_roll  - roll_offset);
    float pitch_zeroed = wrapAngle(current_pitch - pitch_offset);
    float yaw_zeroed   = wrapAngle(current_yaw   - yaw_offset);

    Serial.print("Zeroed Euler angles - Roll: ");
    Serial.print(roll_zeroed, 2);
    Serial.print("  Pitch: ");
    Serial.print(pitch_zeroed, 2);
    Serial.print("  Yaw: ");
    Serial.println(yaw_zeroed, 2);
  }
}

void calibrateZero() {
  roll_offset = current_roll;
  pitch_offset = current_pitch;
  yaw_offset = current_yaw;
  Serial.println("Orientation zeroed.");
}

float wrapAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}
