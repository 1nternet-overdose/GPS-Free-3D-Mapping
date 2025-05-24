#include <Wire.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("QMC5883L during startup...");

  compass.init();
  Serial.println("QMC5883L initialization complete");
}

void loop() {
  compass.read();                     // データ更新

  int x = compass.getX();             // x軸磁場
  int y = compass.getY();             // y軸磁場
  int z = compass.getZ();             // z軸磁場
  int heading = compass.getAzimuth(); // 方位角(0～359度)

  Serial.print("X: "); Serial.print(x);
  Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Z: "); Serial.print(z);
  Serial.print(" | 方位(度): "); Serial.println(heading);

  delay(500);
}
