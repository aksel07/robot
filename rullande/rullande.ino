int sensorPin1 = A8; // Sensor 1 is connected to analog pin 8
int sensorPin2 = A9;
int sensorValue1 = 0;
int sensorValue2 = 0;

int motorPinA = 12; // Pin to set direction of motor A
int brakePinA = 9;  // Pin to activate brake on motor A
int speedPinA = 3;  // Pin to control speed of motor A
int motorPinB = 13; // Pin to set direction of motor B
int brakePinB = 8;  // Pin to activate brake on motor B
int speedPinB = 11; // Pin to control speed of motor B

int motorSpeed = 100; // Decides how fast the robot will go

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  // Setup Channel A
  pinMode(motorPinA, OUTPUT); // Initiates Motor Channel A pin
  pinMode(brakePinA, OUTPUT); // Initiates Brake Channel A pin

  // Setup Channel B
  pinMode(motorPinB, OUTPUT); // Initiates Motor Channel B pin
  pinMode(brakePinB, OUTPUT); // Initiates Brake Channel B pin
}

void loop() {
  sensorValue1 = analogRead(sensorPin1); // Read sensor value from sensor 1
  sensorValue2 = analogRead(sensorPin2); // Read sensor value from sensor 2

  // If the robot is connected to the computer, this will print the sensor values to the serial monitor.
  Serial.print("Sensor1: ");
  Serial.print(sensorValue1);
  Serial.print(" Sensor2: ");
  Serial.print(sensorValue2);
  Serial.println("");

  // If sensor 1 sees a bright surface, activate motor A, brake otherwise.
  if (sensorValue1 > 500) {
    digitalWrite(motorPinA, LOW);       // Establishes forward direction of Channel A
    digitalWrite(brakePinA, LOW);       // Disengage the Brake for Channel A
    analogWrite(speedPinA, motorSpeed); // Spins the motor on Channel A
  }
  else {
    digitalWrite(brakePinA, HIGH); // Engage the Brake for Channel A  
  }

  // If sensor 2 sees a bright surface, activate motor B, brake otherwise.
  if (sensorValue2 > 500) {
    digitalWrite(motorPinB, HIGH);      // Establishes forward direction of Channel B
    digitalWrite(brakePinB, LOW);       // Disengage the Brake for Channel B
    analogWrite(speedPinB, motorSpeed); // Spins the motor on Channel B
  }
  else {
    digitalWrite(motorPinB, LOW);  // Turns off LED on motor shield
    digitalWrite(brakePinB, HIGH); // Engage the Brake for Channel B
  }
}

