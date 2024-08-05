#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include "DHT.h"

const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASSWORD";
AsyncWebServer server(80);

BluetoothSerial serialBT;
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27
#define ENA 26
#define ENB 25
#define Speed 200
#define DHTPIN 33
char value;
String data = "Idle";
String patil = "Camera at center"; // Initially at center

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define SERVO_PIN 32
Servo myservo;
int servoPos = 90; // Initial position
int targetServoPos = 90;
int servoStep = 1;
unsigned long previousMillis = 0;
const long interval = 10; // Interval for servo updates in milliseconds

void setup() {
  serialBT.begin("Esp32-BT");
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  dht.begin();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
  server.on("/", HTTP_GET, handleRoot);
  server.begin();
  
  myservo.attach(SERVO_PIN);
  myservo.write(servoPos); // Set initial position
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    updateServo();
  }
  
  if (serialBT.available()) {
    value = serialBT.read();
    Serial.println(value);
    if (value == 'U') {
      Forward();
      data = "Forward";
    } else if (value == 'D') {
      Backward();
      data = "Backward";
    } else if (value == 'S') {
      Stop();
      data = "Stop";
    } else if (value == 'L') {
      Left();
      data = "Left";
    } else if (value == 'R') {
      Right();
      data = "Right";
    } else if (value == '1') {
      targetServoPos = 180;
      patil = "Left Turn";
    } else if (value == '2') {
      targetServoPos = 90;
    } else if (value == '3') {
      targetServoPos = 0;
      patil = "Right Turn";
    } else if (value == '4') {
      targetServoPos = 90;
    }
  }
}

void updateServo() {
  if (servoPos < targetServoPos) {
    servoPos += servoStep;
    if (servoPos > targetServoPos) {
      servoPos = targetServoPos;
    }
    myservo.write(servoPos);
    //delay(25);
  } else if (servoPos > targetServoPos) {
    servoPos -= servoStep;
    if (servoPos < targetServoPos) {
      servoPos = targetServoPos;
    }
    myservo.write(servoPos);
    //delay(25);
  }
  
  if (servoPos == 90) {
    patil = "Camera at center";
  } else if (servoPos == 180) {
    patil = "Left Turn";
  } else if (servoPos == 0) {
    patil = "Right Turn";
  }
}

void Forward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Backward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void Left() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Right() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void handleRoot(AsyncWebServerRequest *request) {
  int Humidity = dht.readHumidity();
  float Temperature = dht.readTemperature();

  if (isnan(Humidity) || isnan(Temperature)) {
    request->send(200, "text/html", "Failed to read from DHT sensor!");
    return;
  }

  String html = "<html><body><h1> BOT for Real-Time Fault Detection in Restricted Areas with Image Processing </h1><h1>Humidity: " + String(Humidity) + " %</h1><h1>Temperature: " + String(Temperature) + " &deg;C</h1><h1>BOT Running Status: " + data + "</h1><h1>BOT Camera Status: " + patil + "</h1></body></html>";
  request->send(200, "text/html", html);
}
