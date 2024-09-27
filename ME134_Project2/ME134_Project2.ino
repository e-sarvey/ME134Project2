#include <WiFi.h>  // Internet connection baby!
#include <PubSubClient.h> // MQTT client subscribe and publish library for ESP (https://www.arduino.cc/reference/en/libraries/pubsubclient/)
#include <AccelStepper.h> // Simplifies advanced stepper control including duel steppers in sync (https://www.airspayce.com/mikem/arduino/AccelStepper/)
#include <MultiStepper.h> // Sub class of Accel Stepper for running multiple steppers
#include <math.h> // so we can calculate things of course
#include <mysecrets.h> // git ignored file with IP address for MQTT and WIFI credentials

// Define pins for the two steppers
#define STEPPER1_STEP_PIN 4
#define STEPPER1_DIR_PIN 0
#define STEPPER2_STEP_PIN 12
#define STEPPER2_DIR_PIN 14

// Define LED pins for debugging. Each turns on depending on secton of code running or complted.
// LEDS: WFI CONNECTED, MQTT BROKER CONNECTION, TRAJEACTORY COMPLETED, ERROR OCCURED
#define WIFI_LED_PIN 35 
#define MQTT_LED_PIN 34
#define TRAJECTORY_LED_PIN 39
#define ERROR_LED_PIN 36

/// Define the initial offset for the steppers (in degrees)
// We wll want to update this with a homing sequence so we know the starting position of the arm more accurately for the IK results to be more accurate
// We should also update comments to explain which stepper is #1 vs #2 ...
const float INITIAL_OFFSET_1 = -90.0;  // Offset for Stepper 1
const float INITIAL_OFFSET_2 = -90.0;  // Offset for Stepper 2

int stepsPerRevolution = 200; // to calculate angles

// Create AccelStepper objects for each motor
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// Create MultiStepper object from the two single stepper objects
MultiStepper steppers;

// WIFI parameters (imported from mysecrets.h)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PSWD;

// MQTT Broker parameters (IP imported from mysecrets.h)
const char* mqtt_broker = MQTT_IP; 
const char* topic = "ME134/motors";
const int mqtt_port = 1883;

// WiFi and MQTT client objects
WiFiClient espClient;
PubSubClient client(espClient);


void setup_wifi() {
  // This function connects the ESP to the wifi network specified by ssid and password included previously
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  
  Serial.println("Connected to Wi-Fi");
  digitalWrite(WIFI_LED_PIN, HIGH);  // Turn on Wi-Fi LED when connected
}


void moveMotors(float** angles, int numAngles) {
  // This function moves both stepper motors through an array of angles input.
  // Input: 2xN array of angle pairs to move through, int, N number of angle pairs to pass through (required for dynamic array with proper memory allocation)
  long targetPositions[2];
  
  for (int currentAngleIndex = 0; currentAngleIndex < numAngles; currentAngleIndex++) {
    // Apply the offset to the angles
    float adjustedAngle1 = angles[0][currentAngleIndex] + INITIAL_OFFSET_1; // offset angles by initial position
    float adjustedAngle2 = angles[1][currentAngleIndex] + INITIAL_OFFSET_2;

    targetPositions[0] = lround(adjustedAngle1 * stepsPerRevolution / 360.0);  // conversion from angles to steps
    targetPositions[1] = lround(adjustedAngle2 * stepsPerRevolution / 360.0);  
    // Serial printing for debugging
    Serial.print("Stepper 1 Target Position (steps): ");
    Serial.println(targetPositions[0]);
    Serial.print("Stepper 2 Target Position (steps): ");
    Serial.println(targetPositions[1]);
    
    steppers.moveTo(targetPositions);  // Move both motors simultaneously
    steppers.runSpeedToPosition();     // Blocking function, waits until both motors reach target. (There is a non-blocking function which is just run() which returns a boolean true while running if we need that functionality)

    delay(2000);  // Give a little room for error in the motor timing. Not sure how accurate the library is...
  }
  digitalWrite(TRAJECTORY_LED_PIN, HIGH);  // Turn on Trajectory Finished LED
}

void parseAndMoveMotors(char* message, unsigned int length) {
  // This function parses the character message recieved via MQTT and creates a 2xN angle array, then calls moveMotors() with that array.
  // ASSUMES ANGLE PAIRS LISTED LIKE AN Nx2 MATLAB ARRAY: ex. (90.0,-90.0),(-30.0,180.0) would be sent and parsed as "90.0,-90.0;-30.0,180.0"
  // Takes in the message and message length for memory allocation.
  int numAngles = 0; // initialize

  // Make a copy of the message to avoid modifying the original
  char* messageCopy = new char[length + 1];
  strncpy(messageCopy, message, length);
  messageCopy[length] = '\0';

  // Count the number of semicolons to determine how many angle pairs there are
  for (int i = 0; i < length; i++) {
    if (message[i] == ';') {
      numAngles++;
    }
  }
  numAngles++;  // One more than the number of semicolons since semicolons before 

  // Allocate memory for angles
  float** angles = new float*[2];
  angles[0] = new float[numAngles];  
  angles[1] = new float[numAngles];  

  // Use a single strtok loop to tokenize and parse the angle pairs
  char* pairToken = strtok(messageCopy, ";");
  int index = 0;

  while (pairToken != NULL && index < numAngles) {
    // Use sscanf to parse both angles from the pair at once
    float angle1, angle2;
    if (sscanf(pairToken, "%f,%f", &angle1, &angle2) == 2) {
      angles[0][index] = angle1;
      angles[1][index] = angle2;
    } else {
      Serial.println("Error parsing angles!");
    }

    pairToken = strtok(NULL, ";");
    index++;
  }

  // Debugging: Print parsed angles to make sure the angles parsed match the message recieved
  Serial.print("Parsed Motor 1 Angles: ");
  for (int i = 0; i < numAngles; i++) {
    Serial.print(angles[0][i]);
    if (i < numAngles - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();

  Serial.print("Parsed Motor 2 Angles: ");
  for (int i = 0; i < numAngles; i++) {
    Serial.print(angles[1][i]);
    if (i < numAngles - 1) {
      Serial.print(", ");
    }
  }
  // DEBUGGING CODE COMMENTED OUT. HAD TO MAKE SURE CORRECT NUMBER OF ANGLES WERE BEING PARSED
  //Serial.println();

  //Serial.print("Total Parsed Angle Pairs: ");
 // Serial.println(numAngles);

  moveMotors(angles, numAngles); // now use moveMotor() func to execute trajectory on steppers

  // Clean up for dynamoc memory managment. This way we can recieve messages, and control motors several times without filling up memory
  delete[] angles[0];
  delete[] angles[1];
  delete[] angles;
  delete[] messageCopy;
}

void MessageRecieved(char* topic, byte* message, unsigned int length) {
  // MQTT callback to recieve messages from broker
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

void reconnect() {
// This function handles dropped wifi connections and reconnects ESP to internet and MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP32_Client")) {
      Serial.println("Connected");
      client.subscribe(topic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds"); // Repeats until connected
      digitalWrite(ERROR_LED_PIN, HIGH);  // Turn on Error LED if inital connection fails
      delay(5000);
    }
  }
  digitalWrite(ERROR_LED_PIN, LOW);  // Turn off Error LED on successful connection
}

// MAIN SETUP CODE //
void setup() {
  Serial.begin(115200); // establish serial connection

// SETUP LEDS
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(MQTT_LED_PIN, OUTPUT);
  pinMode(TRAJECTORY_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  
  setup_wifi();
  
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(MessageRecieved);

  stepper1.setMaxSpeed(20); 
  stepper1.setAcceleration(10);
  
  stepper2.setMaxSpeed(20);
  stepper2.setAcceleration(10);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}

// MAIN LOOPING CODE: "While True" //
void loop() {
// really simple loop that just makes sure we are connected to wifi since the callback handles the rest of the control. 
// It's probably not best practice to use the callback to call function to parse the message but its ok for now since we should only recieve one at a time but if we had multiple callbacks I'm not sure how calling the other functions inside it would effect the performance.

  if (!client.connected()) {
    reconnect();
  }
  client.loop();  
}
