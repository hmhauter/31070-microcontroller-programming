#include <Servo.h>
Servo myServo;

const int piezo = A0; 
const int switchPin = 2; 
const int yellowLed = 3; 
const int greenLed = 4; 
const int redLed = 5; 

int knockVal; 
int switchVal = LOW; 

const int quietKnock = 70;
const int loudKnock = 100;

boolean locked = false;
int numberOfKnocks = 0;

void setup() {
  myServo.attach(9);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(switchPin, INPUT);
  Serial.begin(9600);

  digitalWrite(greenLed, HIGH);
  myServo.write(0);
  Serial.println("The BOX is unlocked ;)!");
  

}

void loop() {
  Serial.println("Number of Knocks "); 
  Serial.println(switchVal); 
  
  if(locked == false) {
    Serial.println("Locked is False"); 
    switchVal = digitalRead(switchPin);
    // lock box again after pressing switch 
    if(switchVal == HIGH){ 
       locked = true; 
       numberOfKnocks = 0; 
       digitalWrite(greenLed,LOW); 
       digitalWrite(redLed,HIGH); 
       myServo.write(90); 
       Serial.println("The box is locked!"); 
       delay (1000);
    }
   }
   if(locked == true) {
    Serial.println("Locked is True"); 
    knockVal = analogRead(piezo); 
    if(numberOfKnocks < 3 && knockVal > 0){ 
     if(checkForKnock(knockVal) == true){
     numberOfKnocks++;
     } 
     Serial.print(3-numberOfKnocks); 
     Serial.println(" more knocks to go"); 
     }
    if(numberOfKnocks >= 3){ 
      // open box
     locked = false; 
     myServo.write(0); 
     digitalWrite(greenLed,HIGH); 
     digitalWrite(redLed,LOW); 
     Serial.println("The box is unlocked!"); 
     delay(1000); 
     }
    }
    

}

boolean checkForKnock(int value){
  // function that checks the intensity of the knock 
  if(value > quietKnock && value < loudKnock){ 
    digitalWrite(yellowLed, HIGH);
    delay(50);
    digitalWrite(yellowLed, LOW);
    Serial.print("Valid knock of value "); 
    Serial.println(value);
    return true;
   }
   else {
    Serial.print("Bad knock value "); 
    Serial.println(value); 
    return false;
    }
  
  }
