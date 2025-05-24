#include <Wire.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();

  Serial.println("QMC5883L 初期化完了");
}

void loop() {
  compass.read();

  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();
  int heading = compass.getAzimuth();

  Serial.print("X: "); Serial.print(x);
  Serial.print(" Y: "); Serial.print(y);
  Serial.print(" Z: "); Serial.print(z);
  Serial.print(" 方位（度）: "); Serial.println(heading);

  delay(500);
}
