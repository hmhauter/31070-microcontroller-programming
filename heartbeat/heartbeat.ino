// let LED blink every x seconds withot delay command
// connect LED to PIN 1
const int ledPin =  1;
// state of the LED (=: LOW, 1: HIGH)
int ledState = 0;        
// last time LED was updated since resetting the MKR1000
unsigned long previousMillis = 0;      
// blink interval 
const long interval = 1000;          

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // update last time LED state was changed
    previousMillis = currentMillis;

    // invert LED state to let it blink
    ledState = !ledState;

    // update LED
    digitalWrite(ledPin, ledState);
  }
}
