const int trigPin = 2;
const int echoPin = 3;
const int relayPin = 21;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); 
  
  Serial.begin(9600);
}

void loop() {
  long duration;
  int distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

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
