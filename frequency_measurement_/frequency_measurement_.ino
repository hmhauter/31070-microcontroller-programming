#include "Timer5.h"
#include <LiquidCrystal.h>
#include <math.h>
#include <string.h>
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <SAMD21turboPWM.h>
#include <Servo.h>

#include "arduino_secrets.h"
#include "thingProperties.h"

#define PI 3.1415926535897932384626433832795

LiquidCrystal lcd(8, 4, 9, 10, 3, 2); // 5 and 13 are changed // 13 wird 9 // 12 wird 8 // 11 wird 10
TurboPWM pwm;
Servo myServo;


// Global Variables

// Debugging 
const int ledPin = 1;
const int loadPin = 6;
const int generatorPin = 0;

const int buttonPin = 11;

// Frequency Calculation
volatile long int zeroCrossing = 0;

volatile float sample = 0.0;
float frequency = 0.0;
const int samplingRate = 10000;
const int offsetValueZeroCrossing = 257;

const int resolution = 1023; 

// Low Pass Filter
const float crossOverFreq = 150.0;

float alpha;
float oneMinusAlpha; //Calculation to speed up filter calculation in ISR

// Interpolation
volatile float oldY = 0.0;
volatile float newY = 0.0;
volatile float oldY_int = 0;
volatile float newY_int = 0;

const float frequencyPeriod = 1.0;
unsigned long previousMillis = 0;  

volatile int sampleCounter = 0;

const int measureFrequency = 50; 

// RMS Calculation 
float RMS_voltage = 0.0;
float RMS_current = 0.0;
float RMS_power = 0.0;

int outerSampleCounter = 0;
float voltSum = 0.0;
float currentSum = 0.0;
float powerSum = 0.0;

// Droop control 
const float proportionalGain = (1.0/86400.0);
const float carPower = 10000.0;
const float baseFrequency = 50.0;

// Manual Control of Current 
bool isManual = false;    // initially manual control should be disabled
float manualAmpere = 0.0;

// Set updater 
unsigned long int updater = 0;
bool canStart = false;
void setup() {
  // Cloud SetUp 
  initProperties();

  // Uncomment for debugging connection Arduino - Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::CONNECT, doThisOnConnect);
  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::SYNC, doThisOnSync);
  //  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::DISCONNECT, doThisOnDisconnect);
  // 
  setDebugMessageLevel(DBG_INFO);
  ArduinoCloud.printDebugInfo();


  Serial.begin(9600);
  alpha = calculateAlpha(crossOverFreq, (1.0/samplingRate));
  oneMinusAlpha = 1.0 - alpha;
  
  analogReadResolution(10);
  analogWriteResolution(10);
  
  pinMode(ledPin, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(loadPin, OUTPUT);
  pinMode(generatorPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  analogWrite(7, 1);
  digitalWrite(loadPin, 0);
  digitalWrite(ledPin, 0);
  digitalWrite(generatorPin, 0);
  
  
  MyTimer5.begin(samplingRate);
  MyTimer5.attachInterrupt(detectZeroCrossing);
  AdcBooster();
  // DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  MyTimer5.start();
  
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);

  // Setup PWM frequency f=1000 Hz
  pwm.setClockDivider(2, false);
  // pwm.timer([0, 1, 2], [1, 2, 4, 8, 16, 64, 256, 1024], [2-MaxSteps], [true, false]);
  pwm.timer(0, 1, 24000, true); // port 5
  

}

void loop() {
  //Serial.println("Update");
   //ArduinoCloud.update(); 
//while (ArduinoCloud.connected() == 0 || canStart == false) 
//  {
//    ArduinoCloud.update();
//    Serial.println("Waiting for connection to Arduino IoT Cloud");
//    delay(1000);
//}
//  


  if (zeroCrossing >= measureFrequency) {
   
    // this is important!
    // use scaling factors for frequency measurement (emperically determined)
    // don't use sample Rate - it is not accurate enough
    float period = (sampleCounter * (1.0 / 10927.0) *1.014 ) / zeroCrossing;

    // period -= interpolation();
    
    frequency = 1/period; //zeroCrossing / (sampleCounter * (1.0 / 10927.0)); // *1.014); //1.0186   1.00731
    // only frequency print out: scaling factor 1.007375
    
    // Load and Generator Control
    generatorLoadControl(); 
    
    // RMS calculations
    calculate_RMS();
    
    // LCD prints
    printLCD();

    // NOT MANUAL 
    float PWMPercent = 0.0;
    if(manualAmpere == false) {
      // Droop Control and PWM Calculation 
      float droopPower = droopControl();
      float PWMCurrent = calculateCurrent(droopPower);
      PWMPercent = dutyCycle(PWMCurrent);
      
      // Cloud Variables Updates
      Cpwmperc = PWMPercent;
      Campere = PWMCurrent;
      Cpower = droopPower;
      
    } else {
       PWMPercent = dutyCycle(manualAmpere); 
       Cpwmperc = PWMPercent;
       Cpower = manualAmpere*RMS_voltage;
       Campere = CmanualAmpere;
    }
    
    if(digitalRead(buttonPin) == 1) {
      isManual = !isManual;
      CisManual = isManual;
      manualLED(); 
    }
    
    pwm.analogWrite(5, PWMPercent*10);
    // for debugging PWM Frequency 
    float z = pwm.frequency(0);
    
    
    // reset values for next period
    voltSum = 0;
    currentSum = 0;
    outerSampleCounter = 0;
    zeroCrossing = 0;
    sampleCounter = 0;
    if(outerSampleCounter % 5 == 0) {
      Serial.println("Update Cloud");
      ArduinoCloud.update(); 
      }
    }

    float volt = map(sample, 0, (resolution/2), -240*sqrt(2), 240*sqrt(2));
    float current = map(sample, 0, (resolution/2), -42*sqrt(2), 42*sqrt(2));
    voltSum += powf(volt, 2);
    currentSum += powf(current, 2);
    powerSum += current * volt;
    
    outerSampleCounter++;
}

float interpolation() {
  float interpolX = ((float)newY_int-(float)offsetValueZeroCrossing) / ((float)newY_int-(float)oldY_int);
  return interpolX*(1.0 / 10927.0);
}

float droopControl() {
  return (carPower + ((baseFrequency - frequency) / proportionalGain));
}

float calculateCurrent(float droopPower) {
  return (droopPower / 240.0);
}

float dutyCycle(float current) {
  float PWM_percent;
  if (current <= 6.0) {
    PWM_percent = 10.0;   
    }
  else if(current >= 6.0 && current < 52.0 ) {
    PWM_percent = (75.0/46.0)*(current - 6.0) + 10.0;
    }
  else if(current >= 52.0 && current <= 52.5) {
    PWM_percent = 85.0;
    } 
  else if(current > 52.5 && current < 80.0) {
    PWM_percent = 0.4*(current - 52.5) + 85.0;
    }
  else if(current >= 80.0) {
    PWM_percent = 96.0;
    }
  return PWM_percent;
}

void printLCD() {
    // LCD prints
    lcd.clear();
    // Frequency
    lcd.print("Freq:");
    lcd.setCursor(6, 0);
    lcd.print(frequency, 4);
    lcd.setCursor(14, 0);
    lcd.print("Hz");
    // RMS Volt 
    lcd.setCursor(0, 1);
    lcd.print(RMS_voltage, 2);
    lcd.setCursor(6, 1);
    lcd.print("V");
    // RMS Current 
//    lcd.setCursor(10, 1);   
//    lcd.print(RMS_current, 2);
//    lcd.setCursor(15, 1);
//    lcd.print("A");
//  
  }

void calculate_RMS() {
    RMS_voltage = 0.997 * sqrt(voltSum/(float)outerSampleCounter);
    RMS_current = 1.0022 * sqrt(currentSum/(float)outerSampleCounter);
    RMS_power = powerSum/(float)outerSampleCounter;
}

void generatorLoadControl() {
    if(frequency >= 50.1) {
      digitalWrite(loadPin, 1);
      digitalWrite(generatorPin, 0);
    } 
    else if(frequency <= 49.9) {
      digitalWrite(loadPin, 0);
      digitalWrite(generatorPin, 1);
    } else {
      digitalWrite(loadPin, 0);
      digitalWrite(generatorPin, 0);
     
    }
}

void AdcBooster() {
  ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | // Divide Clock by 16.
  ADC_CTRLB_RESSEL_10BIT; // Result on 10 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | // 1 sample
  ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00; // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
} 

float lowPassFilter(float Input, float yOld){
  //Low-pass filter function
  return (float) alpha*Input+oneMinusAlpha*yOld;  
} 

void detectZeroCrossing(void) {
  // Interrupt Callback function 
  sample = analogRead(A1);
  // apply low pass filter to smoothen data
  newY = lowPassFilter(sample, oldY);

  sampleCounter++;
  // detect "zero crossing"
  if(newY >= offsetValueZeroCrossing && oldY < offsetValueZeroCrossing) {
    zeroCrossing++;
    newY_int = newY;
    oldY_int = oldY;
  }  
  oldY = newY;
  // only for debugging 
  // comment this for the final version 
  analogWrite(A0, newY);
}


float calculateAlpha(float crossOverFreq, float deltaT) {
  float RC = 1 / (2 * PI * crossOverFreq);
  return deltaT / (RC + deltaT);
}

void doThisOnConnect(){
  Serial.println("Board successfully connected to Arduino IoT Cloud");
}
void doThisOnSync(){
  Serial.println("Thing Properties synchronised");
  canStart = true;
  
}
void doThisOnDisconnect(){
  // Serial.println("Board disconnected from Arduino IoT Cloud");
}

void manualLED() {
  if (isManual == true) {
    digitalWrite(ledPin, 1);
  } else {
    digitalWrite(ledPin, 0);
  }
}

void onCisManualChange()  {
  isManual = CisManual;
  manualLED(); 
}

void onCmanualAmpereChange() {
  if (isManual == true) {
    manualAmpere = CmanualAmpere;
  }   
}
