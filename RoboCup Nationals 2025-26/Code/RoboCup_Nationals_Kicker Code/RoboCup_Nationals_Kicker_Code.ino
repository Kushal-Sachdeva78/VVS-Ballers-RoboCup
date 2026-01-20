/*
 * Ultrasonic Sensor (HC-SR04) + Relay Module
 * Trig: D2
 * Echo: D3
 * Relay: D7
 */

const int trigPin = 2;
const int echoPin = 3;
const int relayPin = 21;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relayPin, OUTPUT);

  // Initialize relay to OFF 
  // (Most relay modules are 'Active Low', so HIGH = OFF)
  digitalWrite(relayPin, HIGH); 
  
  Serial.begin(9600);
}

void loop() {
  long duration;
  int distance;

  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by sending a 10us pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin (returns the sound travel time in microseconds)
  duration = pulseIn(echoPin, HIGH);

  // Calculate distance in centimeters
  distance = duration * 0.034 / 2;


  // Debugging output to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
if(distance < 4 )
{
delay(100);
digitalWrite(relayPin,0);
delay(400);
digitalWrite(relayPin,1);
delay(400);
}
else
{
  digitalWrite(relayPin,1);
}
}