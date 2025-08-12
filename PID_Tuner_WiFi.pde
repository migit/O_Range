// Processing GUI for Self-Balancing Robot IMU Calibration and PID Tuning (WiFi Version)
// Visualizes raw tilt, smoothed tilt, PID output with a sci-fi themed interface
// Features: Neon color palette, grid-based graphs, status texts in left panel
// Updates: Uses TCP over WiFi instead of Serial, maintains original functionality
// Communicates with ESP32 via TCP (BalancingRobotWiFi.ino)
// Requires controlP5 and processing.net libraries, Orbitron font (or fallback to Consolas)
import controlP5.*;
import processing.net.*;
ControlP5 cp5;
Client client;
PFont sciFiFont;
boolean clientInitialized = false;
float tilt = 0.0, rawTilt = 0.0, pidOutput = 0.0, tiltOffset = 0.0;
float[] tiltHistory = new float[400], rawTiltHistory = new float[400], pidHistory = new float[400];
int historyIndex = 0;
long lastDataTime = 0;
String lastRawInput = "";
float lastTilt = 0.0;
int oscillationCount = 0;
float oscillationThreshold = 1.0;
color dataIndicatorColor = color(255, 64, 64);
String esp32IP = "192.168.1.100";  // Replace with your ESP32's IP address
int esp32Port = 80;

void setup() {
  size(1000, 700);
  cp5 = new ControlP5(this);
  
  sciFiFont = createFont("Orbitron", 12, true);
  if (sciFiFont == null) sciFiFont = createFont("Consolas", 12, true);
  textFont(sciFiFont);
  
  // Attempt to connect to ESP32 via TCP
  println("Attempting to connect to ESP32 at " + esp32IP + ":" + esp32Port);
  for (int i = 0; i < 5; i++) {
    try {
      client = new Client(this, esp32IP, esp32Port);
      if (client.active()) {
        clientInitialized = true;
        println("Connected to ESP32 at " + esp32IP);
        break;
      }
    } catch (Exception e) {
      println("Attempt " + (i + 1) + ": Error connecting to ESP32: " + e.getMessage());
      if (i < 4) delay(2000); else println("Failed after 5 attempts.");
    }
  }
  
  // Set up sliders
  cp5.addSlider("Kp").setPosition(20, 50).setSize(200, 20).setRange(0, 50).setValue(10.0)
     .setLabel("Kp").setColorForeground(color(0, 255, 255)).setColorBackground(color(50, 60, 90))
     .setColorActive(color(0, 128, 255)).setColorLabel(color(200, 220, 255));
  cp5.addSlider("Ki").setPosition(20, 80).setSize(200, 20).setRange(0, 1).setValue(0.0)
     .setLabel("Ki").setColorForeground(color(0, 255, 255)).setColorBackground(color(50, 60, 90))
     .setColorActive(color(0, 128, 255)).setColorLabel(color(200, 220, 255));
  cp5.addSlider("Kd").setPosition(20, 110).setSize(200, 20).setRange(0, 10).setValue(5.0)
     .setLabel("Kd").setColorForeground(color(0, 255, 255)).setColorBackground(color(50, 60, 90))
     .setColorActive(color(0, 128, 255)).setColorLabel(color(200, 220, 255));
  
  // Initialize history arrays
  for (int i = 0; i < 400; i++) {
    tiltHistory[i] = rawTiltHistory[i] = pidHistory[i] = 0.0;
  }
}

void draw() {
  background(20, 24,36);
  fill(30, 36, 54, 200);
  noStroke();
  rect(10, 10, 240, 680, 10);
  fill(200, 220, 255);
  textAlign(LEFT);
  text(String.format("PID:\nKp=%.2f\nKi=%.2f\nKd=%.2f\nOffset=%.2f",
                     cp5.getController("Kp").getValue(),
                     cp5.getController("Ki").getValue(),
                     cp5.getController("Kd").getValue(),
                     tiltOffset), 20, 200);
  text("Connection: " + (clientInitialized && client.active() ? "Connected" : "Disconnected"), 20, 300);
  String lastData = lastRawInput.length() > 20 ? lastRawInput.substring(0, 17) + "..." : lastRawInput;
  textSize(10);
  text(String.format("Last: %s\nTime: %d ms", lastData, millis() - lastDataTime), 20, 310);
  textSize(12);
  fill(abs(tilt) < 2.0 ? color(0, 255, 128) : color(255, 64, 64));
  rect(20, 350, 30, 20, 5);
  text("CoG: " + (abs(tilt) < 2.0 ? "Stable" : "Adjust"), 60, 365);
  text("Wheels: " + (tilt > 0 ? "Backward" : tilt < 0 ? "Forward" : "Stopped"), 20, 400);
  fill(oscillationCount > 50 ? color(255, 64, 64) : color(0, 255, 128));
  rect(20, 410, 30, 20, 5);
  text("Jitter: " + (oscillationCount > 50 ? "High (Tune)" : "Stable"), 60, 425);
  if (oscillationCount > 50) text("Tip: Lower Kp, inc Kd", 20, 445);
  fill(dataIndicatorColor);
  rect(20, 470, 30, 20, 5);
  fill(200, 220, 255);
  text("Data: " + (lastDataTime > 0 ? "Real" : "None"), 60, 485);
  drawGraph(300, 20, 670, 200, rawTiltHistory, color(0, 255, 255), "Raw Tilt", rawTilt, -90, 90);
  drawGraph(300, 250, 670, 200, tiltHistory, color(0, 128, 255), "Smoothed Tilt", tilt, -90, 90);
  drawGraph(300, 450, 670, 100, pidHistory, color(255, 0, 255), "PID Output", pidOutput, -100, 100);
  
  // Check for incoming data
  if (clientInitialized && client.active() && client.available() > 0) {
    processClientData();
  }
}

void drawGraph(float x, float y, float w, float h, float[] data, color lineColor, String title, float currentValue, float minVal, float maxVal) {
  fill(30, 36, 54, 200);
  stroke(50, 60, 90, 150);
  strokeWeight(1);
  rect(x, y, w, h, 10);
  int gridLines = 5;
  float yStep = h / gridLines, valStep = (maxVal - minVal) / gridLines;
  for (int i = 0; i <= gridLines; i++) {
    float gy = y + i * yStep;
    line(x, gy, x + w, gy);
    fill(200, 220, 255);
    textAlign(RIGHT);
    text(String.format("%.1f", maxVal - i * valStep), x - 5, gy + 5);
  }
  noFill();
  stroke(lineColor, 150);
  strokeWeight(3);
  beginShape();
  for (int i = 0; i < 400; i++) {
    int idx = (historyIndex + i) % 400;
    float val = data[idx], gy = map(val, minVal, maxVal, y + h, y);
    if (!Float.isNaN(gy)) vertex(x + i * w / 400, gy);
  }
  endShape();
  stroke(lineColor);
  strokeWeight(1);
  beginShape();
  for (int i = 0; i < 400; i++) {
    int idx = (historyIndex + i) % 400;
    float val = data[idx], gy = map(val, minVal, maxVal, y + h, y);
    if (!Float.isNaN(gy)) vertex(x + i * w / 400, gy);
  }
  endShape();
  fill(200, 220, 255);
  textAlign(LEFT);
  text(title, x, y - 10);
  text(String.format("Val: %.2f", currentValue), x, y + h + 20);
}

void processClientData() {
  try {
    String input = client.readStringUntil('\n');
    if (input == null || input.trim().isEmpty()) return;
    input = input.trim();
    lastRawInput = input;
    lastDataTime = millis();
    println("Received: [" + input + "]");
    if (input.startsWith("DATA")) {
      String[] data = input.split(",");
      if (data.length >= 3) {
        try {
          tilt = parseFloatSafe(data[1], 0.0);
          pidOutput = parseFloatSafe(data[2], 0.0);
          rawTilt = (data.length >= 4) ? parseFloatSafe(data[3], 0.0) : 0.0;
          dataIndicatorColor = color(0, 255, 128);
          if (abs(tilt - lastTilt) > oscillationThreshold) oscillationCount++;
          else oscillationCount = max(0, oscillationCount - 1);
          lastTilt = tilt;
          tiltHistory[historyIndex] = tilt;
          rawTiltHistory[historyIndex] = rawTilt;
          pidHistory[historyIndex] = pidOutput;
          historyIndex = (historyIndex + 1) % 400;
          if (data.length < 4) println("Note: rawTilt missing, defaulting to " + rawTilt);
        } catch (Exception e) {
          println("Parse error: [" + input + "], " + e.getMessage());
          dataIndicatorColor = color(255, 64, 64);
        }
      }
    } else if (input.startsWith("Calibration Complete")) {
      try {
        tiltOffset = parseFloatSafe(input.split(":")[1].split(" ")[1], 0.0);
        println("Tilt Offset: " + tiltOffset);
      } catch (Exception e) {
        println("Calibration error: [" + input + "], " + e.getMessage());
      }
    } else if (input.startsWith("Updated PID")) {
      println("PID updated: [" + input + "]");
    }
  } catch (Exception e) {
    println("Client error: " + e.getMessage());
    dataIndicatorColor = color(255, 64, 64);
    clientInitialized = false;
    // Attempt to reconnect
    try {
      client = new Client(this, esp32IP, esp32Port);
      if (client.active()) clientInitialized = true;
    } catch (Exception reconnectEx) {
      println("Reconnect failed: " + reconnectEx.getMessage());
    }
  }
}

float parseFloatSafe(String s, float fallback) {
  try {
    return Float.parseFloat(s.trim());
  } catch (NumberFormatException e) {
    println("FloatទFloat error: [" + s + "], using " + fallback);
    return fallback;
  }
}

void controlEvent(ControlEvent event) {
  if (!clientInitialized || !client.active() || !event.isController()) return;
  float kp = cp5.getController("Kp").getValue();
  float ki = cp5.getController("Ki").getValue();
  float kd = cp5.getController("Kd").getValue();
  client.write(String.format("PID,%.2f,%.2f,%.2f\n", kp, ki, kd));
  println("Sent PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
}
