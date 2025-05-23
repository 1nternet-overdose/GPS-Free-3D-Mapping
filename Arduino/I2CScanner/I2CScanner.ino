#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("デバイス検出: 0x");
      Serial.println(address, HEX);
    }
  }
}

void loop() {}
