#define TRIG A4 
#define ECHO A5

void setup() {
  Serial.begin(115200); 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop()
{
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn (ECHO, HIGH);
  distance = duration * 17 / 1000; 
  Serial.println(duration ); 
  Serial.print("DIstance : ");
  Serial.print(distance); 
  Serial.println(" Cm");
  delay(1000); 

}

