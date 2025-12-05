/*Mikrollere@Hackster.io*/
#include <WiFi.h>
#include <MPU6050.h>
#include <PID_v1.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";  // Replace with your hotspot name
const char* password = "YOUR_WIFI_PASSWORD";  // Replace with yout wifi passwords

// TCP server
WiFiServer server(80);
WiFiClient client;

// MPU6050 and PID
MPU6050 mpu;
double setpoint = 0;
double input, output;
double Kp = 10, Ki = 0, Kd = 5;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// BTS7960 pins
const int motor1_RPWM = 25;
const int motor1_LPWM = 26;
const int motor1_EN = 33;
const int motor2_RPWM = 27;
const int motor2_LPWM = 14;
const int motor2_EN = 32;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);

  pinMode(motor1_RPWM, OUTPUT);
  pinMode(motor1_LPWM, OUTPUT);
  pinMode(motor1_EN, OUTPUT);
  pinMode(motor2_RPWM, OUTPUT);
  pinMode(motor2_LPWM, OUTPUT);
  pinMode(motor2_EN, OUTPUT);
  digitalWrite(motor1_EN, HIGH);
  digitalWrite(motor2_EN, HIGH);

  ledcSetup(0, 1000, 8); ledcAttachPin(motor1_RPWM, 0);
  ledcSetup(1, 1000, 8); ledcAttachPin(motor1_LPWM, 1);
  ledcSetup(2, 1000, 8); ledcAttachPin(motor2_RPWM, 2);
  ledcSetup(3, 1000, 8); ledcAttachPin(motor2_LPWM, 3);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(1000);
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  if (!client.connected()) client = server.available();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  input = atan2(ay, az) * 180 / PI;

  myPID.Compute();
  driveMotors(output);

  if (client.connected()) {
    client.print("DATA,");
    client.print(input);
    client.print(",");
    client.print(output);
    client.print(",");
    client.print(input);  // Raw tilt 
    client.println();
  }

// Receive PID values from client
  if (client.available()) {
    String data = client.readStringUntil('\n');
    if (data.startsWith("PID")) {
      int comma1 = data.indexOf(",");
      int comma2 = data.indexOf(",", comma1 + 1);
      int comma3 = data.indexOf(",", comma2 + 1);
      if (comma1 != -1 && comma2 != -1 && comma3 != -1) {
        Kp = data.substring(comma1 + 1, comma2).toFloat();
        Ki = data.substring(comma2 + 1, comma3).toFloat();
        Kd = data.substring(comma3 + 1).toFloat();
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.println("Updated PID: Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
        if (client.connected()) {
          client.println("Updated PID");
        }
      }
    }
  }

  delay(10);
}

void driveMotors(double speed) {
  int pwm = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    ledcWrite(0, pwm); ledcWrite(1, 0);
    ledcWrite(2, pwm); ledcWrite(3, 0);
  } else if (speed < 0) {
    ledcWrite(0, 0); ledcWrite(1, pwm);
    ledcWrite(2, 0); ledcWrite(3, pwm);
  } else {
    ledcWrite(0, 0); ledcWrite(1, 0);
    ledcWrite(2, 0); ledcWrite(3, 0);
  }
}
