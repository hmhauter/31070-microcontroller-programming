const int switchPin = 7;
unsigned long previousTime = 0;

int switchState = 0;
int prevSwitchState = 0;

int led = 1;

// time interval until next LED is HIGH
const long interval = 2000;


void setup() {
  for(int x = led; x<7;x++) {
    pinMode(x, OUTPUT);
    }
    pinMode(switchPin, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  // wait until interval is reached 
  if(currentTime - previousTime > interval) {
    previousTime = currentTime;
    digitalWrite(led, HIGH);
    led++;
    if(led == 8) {
      led = 1;
      // reset LEDs
      reset();
      }
    }
  switchState = digitalRead(switchPin);

  if(switchState != prevSwitchState){
   reset();
   led = 1;
   previousTime = currentTime;
   }
   prevSwitchState = switchState;
    
}

void reset() {
  // function that sets all LEDs to LOW 
   for(int x = led;x<7;x++){ 
   digitalWrite(x, LOW);
   } 
  
  }
