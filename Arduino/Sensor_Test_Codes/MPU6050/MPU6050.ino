#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("MPU6050 during startup...");

  mpu.initialize();
  Serial.println("MPU6050 initialization complete");

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection success");
  } else {
    Serial.println("MPU6050 connection failure");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az; // 加速度(X, Y, Z軸)
  int16_t gx, gy, gz; // 角速度(X, Y, Z軸)

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az);
  Serial.print(" | Gyro: ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);

  delay(500);
}
