import processing.serial.*;

Serial myPort;
String[] sensorData;
String[] latestData;  // 最新データを保持
String filename = "sensor_log_" + year() + "_" + month() + "_" + day() + "_" + hour() + "_" + minute() + ".csv";
PrintWriter output;

void setup() {
  size(800, 400);
  printArray(Serial.list());

  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');

  output = createWriter(filename);

  background(0);
  fill(255);
  textSize(16);
}

void draw() {
  background(0);
  fill(255);
  textSize(16);

  if (latestData != null && latestData.length == 14) {
    text("Time = " + latestData[0] + " ms", 50, 25);

    for (int i = 1; i <= 3; i++) {
      text("Acc[" + (i - 1) + "] = " + latestData[i], 50, 25 + i * 25);
    }

    for (int i = 4; i <= 6; i++) {
      text("Gyro[" + (i - 4) + "] = " + latestData[i], 250, (i - 3) * 25);
    }

    for (int i = 7; i <= 9; i++) {
      text("Mag[" + (i - 7) + "] = " + latestData[i], 450, (i - 6) * 25);
    }

    text("ToF = " + latestData[10] + " mm", 50, 160);
    text("Pres = " + latestData[11] + " Pa", 50, 190);
    text("Temp = " + latestData[12] + " °C", 50, 220);
    text("Humid = " + latestData[13] + " %", 50, 250);
  } else {
    text("Waiting for sensor data...", 50, 50);
  }
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;

  line = trim(line);
  sensorData = split(line, ',');

  if (sensorData.length == 14) {
    output.println(line);
    println(line);
    latestData = sensorData;  // 最新データを更新
  } else {
    println("Invalid data length (" + sensorData.length + "): " + line);
  }
}

void keyPressed() {
  if (key == 'q' || key == 'Q') {
    output.flush();
    output.close();
    exit();
  }
}
