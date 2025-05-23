import processing.serial.*;

Serial myPort;
String[] values;
int x = 0, y = 0;

void setup() {
  size(600, 600);
  background(0);

  // シリアルポート自動選択（必要に応じて修正）
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.bufferUntil('\n');

  // 原点中央
  translate(width/2, height/2);
}

void draw() {
  // 原点中央に移動
  translate(width / 2, height / 2);
  
  // 少しずつ背景を薄く消す
  fill(0, 10);
  noStroke();
  rect(-width/2, -height/2, width, height);

  // プロット（点描）
  stroke(0, 255, 0);
  strokeWeight(3);
  point(x / 10.0, -y / 10.0);  // 反転はY軸を上向きに
}

void serialEvent(Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  if (inString != null) {
    inString = trim(inString);
    values = split(inString, ',');
    if (values.length == 2) {
      try {
        x = int(values[0]);
        y = int(values[1]);
      } catch (Exception e) {
        println("データ変換エラー: " + e);
      }
    }
  }
}
