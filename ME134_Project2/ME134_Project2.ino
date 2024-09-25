#include <WiFi.h>  
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

// Define pins for the two steppers
#define STEPPER1_STEP_PIN 36
#define STEPPER1_DIR_PIN 39
#define STEPPER2_STEP_PIN 34
#define STEPPER2_DIR_PIN 35

// Define LED pins
#define WIFI_LED_PIN 23  // Change to your LED pin
#define MQTT_LED_PIN 22  // Change to your LED pin
#define TRAJECTORY_LED_PIN 21  // Change to your LED pin
#define ERROR_LED_PIN 19  // Change to your LED pin

int stepsPerRevolution = 200;

// Create AccelStepper objects for each motor
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// Create MultiStepper object
MultiStepper steppers;

const char* ssid = "Tufts_Robot";
const char* password = "";

// MQTT Broker details
const char* mqtt_broker = ""; 
const char* topic = "ME134/motor";  
const int mqtt_port = 1883;

// WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  
  Serial.println("Connected to Wi-Fi");
  digitalWrite(WIFI_LED_PIN, HIGH);  // Turn on Wi-Fi LED
}

// Function to move motors through an array of angles
void moveMotors(float** angles, int numAngles) {
  long targetPositions[2];

  for (int currentAngleIndex = 0; currentAngleIndex < numAngles; currentAngleIndex++) {
    targetPositions[0] = lround(angles[0][currentAngleIndex] * stepsPerRevolution / 360.0);  
    targetPositions[1] = lround(angles[1][currentAngleIndex] * stepsPerRevolution / 360.0);  

    steppers.moveTo(targetPositions);

    while (steppers.run()) {
      // Continue running until the movement is completed
    }
    delay(10);  // take a short pause
  }
  digitalWrite(TRAJECTORY_LED_PIN, HIGH);  // Turn on Trajectory LED
}

// Function to parse the MQTT message into angles
void parseAndMoveMotors(char* message, unsigned int length) {
  int numAngles = 0;
  for (int i = 0; i < length; i++) {
    if (message[i] == ';') {
      numAngles++;
    }
  }
  numAngles++;  

  float** angles = new float*[2];
  angles[0] = new float[numAngles];  
  angles[1] = new float[numAngles];  
  
  char* token = strtok(message, ";");
  int index = 0;
  
  while (token != NULL && index < numAngles) {
    char* angleToken = strtok(token, ",");
    if (angleToken != NULL) {
      angles[0][index] = atof(angleToken);  
      angleToken = strtok(NULL, ",");
      if (angleToken != NULL) {
        angles[1][index] = atof(angleToken);  
      }
    }
    token = strtok(NULL, ";");
    index++;
  }

  moveMotors(angles, numAngles);

  delete[] angles[0];
  delete[] angles[1];
  delete[] angles;
}

// Callback function to handle incoming messages
void MessageRecieved(char* topic, byte* message, unsigned int length) {
  Serial.print("Message received in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    msg[i] = (char)message[i];
  }
  msg[length] = '\0';  

  Serial.print("Message: ");
  Serial.println(msg);

  parseAndMoveMotors(msg, length);
  digitalWrite(MQTT_LED_PIN, HIGH);  // Turn on MQTT LED
}

// Reconnect function
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP32_Client")) {
      Serial.println("Connected");
      client.subscribe(topic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on Error LED
      delay(5000);
    }
  }
  digitalWrite(ERROR_LED_PIN, LOW);  // Turn off Error LED on successful connection
}

// MAIN SETUP CODE //
void setup() {
  Serial.begin(115200); // establish serial connection
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(MQTT_LED_PIN, OUTPUT);
  pinMode(TRAJECTORY_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  
  setup_wifi(); 
  
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(MessageRecieved);

  stepper1.setMaxSpeed(200); 
  stepper1.setAcceleration(200);
  
  stepper2.setMaxSpeed(200);
  stepper2.setAcceleration(200);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}

// MAIN LOOPING CODE: "While True" //
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  
}
