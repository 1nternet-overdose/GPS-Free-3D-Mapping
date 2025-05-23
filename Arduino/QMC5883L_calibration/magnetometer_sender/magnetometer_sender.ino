#include <Wire.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
}

void loop() {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();

  // カンマ区切りで送信
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);

  delay(50);
}
