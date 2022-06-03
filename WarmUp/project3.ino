#include <Arduino_MKRENV.h>

const int sensorPin = A0;
const float baselineTemp = 20.0;
const float criticalTemp = 30.0;

void setup() {
  // put your setup code here, to run once:
  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }
  Serial.begin(9600);

  for(int pinNumber = 2; pinNumber <5; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
    }
   pinMode(6, OUTPUT);

}

void loop() {
  // ENV.begin();
  float ref_temperature = ENV.readTemperature();
  // put your main code here, to run repeatedly:
  int sensorVal = analogRead(sensorPin);
  int ref_temp = digitalRead(6);
  float voltage = (sensorVal/1024.0) * 3.3;
  float temperature = (voltage - 0.5) / (0.010);

  Serial.println("Sensor Value: ");
  Serial.println(sensorVal);
  Serial.println("Volt: ");
  Serial.println(voltage);
  Serial.println("Temperature: ");
  Serial.println(temperature);
  Serial.println("Reference Temperature: ");
  Serial.println(ref_temperature);
  
  if(temperature < baselineTemp){
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  }
  else if(temperature > criticalTemp) {
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, HIGH);  
  }
  else{
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  }
  delay(500);
}
