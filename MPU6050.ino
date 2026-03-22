#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


const float bx = 0.40956;
const float by = -0.13782;
const float bz = 0.285869;

const float sx = 1.003873;
const float sy = 0.993239;
const float sz = 0.971155;


const float gx_bias = -2.2323914;
const float gy_bias = -0.2915797;
const float gz_bias =  0.25525971;

bool isRunning = false;
unsigned long startTime = 0;
unsigned long lastSampleTime = 0;
const unsigned long runDuration = 20000;


float roll = 0;
float pitch = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  Serial.println("Type 'start' to begin 3 min auto run.");
  Serial.println("time_ms,roll,pitch");
}

void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "start" && !isRunning) {
      isRunning = true;
      startTime = millis();
      lastSampleTime = millis();

     
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float ax = (a.acceleration.x - bx) * sx;
      float ay = (a.acceleration.y - by) * sy;
      float az = (a.acceleration.z - bz) * sz;

      roll  = atan2(ay, az) * 180.0 / PI;
      pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

      Serial.println("START (20s auto)");
    }
  }

  if (!isRunning) return;

  unsigned long currentTime = millis();

  if (currentTime - startTime >= runDuration) {
    isRunning = false;
    Serial.println("AUTO STOP (20s reached)");
    return;
  }

  if (currentTime - lastSampleTime >= 20) {

    float dt = (currentTime - lastSampleTime) / 1000.0;
    lastSampleTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    
    float ax = (a.acceleration.x - bx) * sx;
    float ay = (a.acceleration.y - by) * sy;
    float az = (a.acceleration.z - bz) * sz;

    
    float gx = g.gyro.x * 180.0 / PI - gx_bias;
    float gy = g.gyro.y * 180.0 / PI - gy_bias;

   
    float roll_acc  = atan2(ay, az) * 180.0 / PI;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

   
    const float alpha = 0.97;

    roll  = alpha * (roll + gx * dt) + (1 - alpha) * roll_acc;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc;

    unsigned long time_ms = currentTime - startTime;

    Serial.print(time_ms); Serial.print(",");
    Serial.print(roll, 3); Serial.print(",");
    Serial.println(pitch, 3);
  }
}