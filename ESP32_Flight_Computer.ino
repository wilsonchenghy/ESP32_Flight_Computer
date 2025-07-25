#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
// #include <Wire.h>
// #include <Adafruit_BMP085.h>
// #include <SPI.h>
// #include <ArduinoJson.h>

// #define seaLevelPressure_hPa 975.2 // hPa, hPa*100 to Pa

/*
  BNO085 Sensor Connection (SPI Mode):

  VCC  -> 3.3V
  GND  -> GND
  SCL  -> D18
  SDA  -> D19
  AD0  -> D23
  CS   -> D5
  INT  -> D16
  RST  -> D4
  PS1  -> 3.3V
  PS0  -> 3.3V
*/

#define BNO08X_CS 5
#define BNO08X_INT 16
#define BNO08X_RESET 4
#define FLASH_CS 2
#define BUZZER_PIN 14
#define BUTTON_PIN 13
#define PITCH_SERVO_PIN 27
#define ROLL_SERVO_PIN 26
#define PARACHUTE_SERVO_PIN 25
#define GREEN_LED_PIN 32
#define RED_LED_PIN 33
#define MOSFET_PIN 12

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct Quaternion {
  float w, x, y, z;
};

Quaternion q_calib_inv = {1, 0, 0, 0};
bool isCalibrated = false;

// Quaternion multiplication
Quaternion multiplyQuaternions(const Quaternion& q1, const Quaternion& q2) {
  Quaternion r;
  r.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  r.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  r.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  r.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
  return r;
}

// Convert quaternion to Euler angles (degrees)
void quaternionToEuler(const Quaternion& q, float &roll, float &pitch, float &yaw) {
  roll  = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * 180.0f / PI;
  float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (sinp > 1.0f) sinp = 1.0f;
  else if (sinp < -1.0f) sinp = -1.0f;
  pitch = asinf(sinp) * 180.0f / PI;
  yaw   = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 180.0f / PI;
}

// WiFi
const char* ssid = "BELL422";
const char* password = "17F94429DE2F";

WebServer server(80);

float tx_roll = 0.0, tx_pitch = 0.0, tx_yaw = 0.0;
float tx_qw = 1.0, tx_qx = 0.0, tx_qy = 0.0, tx_qz = 0.0;

// Servo
Servo pitchServo;
Servo rollServo;
Servo parachuteServo;

void moveServo() {
  // 0 to 90 degrees
  for (int pos = 0; pos <= 90; pos += 1) {
    pitchServo.write(pos);
    delay(15);
  }
  delay(500);

  // 90 to 0 degrees
  for (int pos = 90; pos >= 0; pos -= 1) {
    pitchServo.write(pos);
    delay(15);
  }
  delay(500);
}

// BMP180
// Adafruit_BMP085 bmp;
// float altitude = 0;
// float filteredAltitude = 0;
// float filterAlpha = 0.1;

// Flash Chip
// SPIClass spi = SPIClass(VSPI);

// #define CMD_WRITE_ENABLE  0x06
// #define CMD_PAGE_PROGRAM  0x02
// #define CMD_READ_DATA     0x03

// void flashWriteEnable() {
//   digitalWrite(FLASH_CS, LOW);
//   spi.transfer(CMD_WRITE_ENABLE);
//   digitalWrite(FLASH_CS, HIGH);
// }

// void flashWriteBytes(uint32_t address, const uint8_t* data, size_t length) {
//   flashWriteEnable();
//   digitalWrite(FLASH_CS, LOW);

//   spi.transfer(CMD_PAGE_PROGRAM);
//   spi.transfer((address >> 16) & 0xFF);
//   spi.transfer((address >> 8) & 0xFF);
//   spi.transfer(address & 0xFF);

//   for (size_t i = 0; i < length; i++) {
//     spi.transfer(data[i]);
//   }

//   digitalWrite(FLASH_CS, HIGH);
//   delay(5); // Wait for write to complete
// }

// void flashReadBytes(uint32_t address, uint8_t* buffer, size_t length) {
//   digitalWrite(FLASH_CS, LOW);

//   spi.transfer(CMD_READ_DATA);
//   spi.transfer((address >> 16) & 0xFF);
//   spi.transfer((address >> 8) & 0xFF);
//   spi.transfer(address & 0xFF);

//   for (size_t i = 0; i < length; i++) {
//     buffer[i] = spi.transfer(0x00);
//   }

//   digitalWrite(FLASH_CS, HIGH);
// }


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // LED
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);

  // Button
  pinMode(BUTTON_PIN, INPUT);  // button with external pull-down

  // MOSFET
  pinMode(MOSFET_PIN, OUTPUT);

  // Servo
  pitchServo.attach(PITCH_SERVO_PIN);
  rollServo.attach(ROLL_SERVO_PIN);
  parachuteServo.attach(PARACHUTE_SERVO_PIN);

  pitchServo.write(90);
  rollServo.write(90);
  parachuteServo.write(90);

  // BNO085
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

  // BMP180
  // if (!bmp.begin()) {
  //   Serial.println("Failed to find BMP180 chip");
  //   while (1) delay(10);
  // }
  // Serial.println("BMP180 Sensor Initialized");

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
  // Serial.print("ESP32 IP address: ");
  // Serial.println(WiFi.localIP());

  server.on("/euler", []() {
    String json = "{";
    json += "\"roll\":" + String(tx_roll, 2) + ",";
    json += "\"pitch\":" + String(tx_pitch, 2) + ",";
    json += "\"yaw\":" + String(tx_yaw, 2);
    json += "}";

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
  });

  // server.on("/quaternion", []() {
  //   String json = "{";
  //   json += "\"w\":" + String(tx_qw, 6) + ",";
  //   json += "\"x\":" + String(tx_qx, 6) + ",";
  //   json += "\"y\":" + String(tx_qy, 6) + ",";
  //   json += "\"z\":" + String(tx_qz, 6);
  //   json += "}";

  //   server.sendHeader("Access-Control-Allow-Origin", "*");
  //   server.send(200, "application/json", json);
  // });

  server.begin();

  // Flash Chip
  // spi.begin();
  // pinMode(FLASH_CS, OUTPUT);
  // digitalWrite(FLASH_CS, HIGH);  // Deselect flash chip
}


void loop() {
  delay(10);

  if (digitalRead(BUTTON_PIN) == HIGH) {
    Serial.println("Button Pressed");

    moveServo();

    // tone(BUZZER_PIN, 1000); // 1000 Hz tone
    // delay(200); // Beep duration
    // noTone(BUZZER_PIN);

    // delay(100);

    // // Beep 2
    // tone(BUZZER_PIN, 1000);
    // delay(200); // Beep duration
    // noTone(BUZZER_PIN);
  }

  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset. Re-enabling reports...");
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
    isCalibrated = false;
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    Quaternion q_current;
    q_current.w = sensorValue.un.gameRotationVector.real;
    q_current.x = sensorValue.un.gameRotationVector.i;
    q_current.y = sensorValue.un.gameRotationVector.j;
    q_current.z = sensorValue.un.gameRotationVector.k;

    if (!isCalibrated) {
      q_calib_inv.w = q_current.w;
      q_calib_inv.x = -q_current.x;
      q_calib_inv.y = -q_current.y;
      q_calib_inv.z = -q_current.z;
      isCalibrated = true;
      Serial.println("Orientation zeroed using quaternion calibration.");
    }

    Quaternion q_zeroed = multiplyQuaternions(q_calib_inv, q_current);

    float roll, pitch, yaw;
    quaternionToEuler(q_zeroed, roll, pitch, yaw);

    // Normalize angles to (-180, 180)
    roll = wrapAngle(roll);
    pitch = wrapAngle(pitch);
    yaw = wrapAngle(yaw);

    // Copy to tx euler angles
    tx_roll = roll;
    tx_pitch = pitch;
    tx_yaw = yaw;

    server.handleClient();

    // Serial.print("Zeroed Euler angles - Roll: ");
    // Serial.print(roll, 2);
    // Serial.print("  Pitch: ");
    // Serial.print(pitch, 2);
    // Serial.print("  Yaw: ");
    // Serial.println(yaw, 2);

    // String serialJson = "{";
    // serialJson += "\"roll\":" + String(roll, 2) + ",";
    // serialJson += "\"pitch\":" + String(pitch, 2) + ",";
    // serialJson += "\"yaw\":" + String(yaw, 2);
    // serialJson += "}";

    // Serial.println(serialJson);

    // tx_qw = q_zeroed.w;
    // tx_qx = q_zeroed.x;
    // tx_qy = q_zeroed.y;
    // tx_qz = q_zeroed.z;

    // server.handleClient();

    // BMP180
    // altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
    // filteredAltitude = filterAlpha * altitude + (1 - filterAlpha) * filteredAltitude;

    // Print to Serial as JSON
    String serialJson = "{";
    serialJson += "\"w\":" + String(q_zeroed.w, 6) + ",";
    serialJson += "\"x\":" + String(q_zeroed.x, 6) + ",";
    serialJson += "\"y\":" + String(q_zeroed.y, 6) + ",";
    serialJson += "\"z\":" + String(q_zeroed.z, 6);
    // serialJson += "\"z\":" + String(q_zeroed.z, 6) + ",";
    // serialJson += "\"altitude\":" + String(altitude, 2) + ",";
    // serialJson += "\"filteredAltitude\":" + String(filteredAltitude, 2);
    serialJson += "}";

    Serial.println(serialJson);

    // Flash Chip
    // StaticJsonDocument<128> doc;
    // doc["w"] = q_zeroed.w;
    // doc["x"] = q_zeroed.x;
    // doc["y"] = q_zeroed.y;
    // doc["z"] = q_zeroed.z;

    // char jsonBuffer[128];
    // size_t jsonLen = serializeJson(doc, jsonBuffer);

    // // --- Write JSON to flash ---
    // uint32_t flashAddr = 0x000000; // Should change this to allow multiple logs and don't always write to the same address
    // flashWriteBytes(flashAddr, (uint8_t*)jsonBuffer, jsonLen);

    // // --- Read back from flash for verification ---
    // uint8_t readBack[128] = {0};
    // flashReadBytes(flashAddr, readBack, jsonLen);

    // Serial.print("Read back from flash: ");
    // Serial.write(readBack, jsonLen);
    // Serial.println();
  }
}

float wrapAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}
