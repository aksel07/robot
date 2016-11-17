int sensorPin1 = A8;
int sensorPin2 = A9;
int sensorValue1 = 0;
int sensorValue2 = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for 250 ms
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(250);                       // wait for 250 ms
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  Serial.print("Sensor1: ");
  Serial.print(sensorValue1);
  Serial.print(" Sensor2: ");
  Serial.print(sensorValue2);
  Serial.println("");
}
