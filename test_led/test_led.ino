#include "Timer5.h"
#include <LiquidCrystal.h>
#include <math.h>
#include <string.h>
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <SAMD21turboPWM.h>

#include "arduino_secrets.h"
#include "thingProperties.h"

#define PI 3.1415926535897932384626433832795
LiquidCrystal lcd(12, 4, 5, 11, 3, 2);
TurboPWM pwm;

// Global Variables

// Debugging 
const int ledPin = 1;

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

bool prevToggleState = 0;
bool toggleState = 0;

// Set updater 
unsigned long int updater = 0;
bool canStart = false;
void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  //alpha = calculateAlpha(crossOverFreq, (1.0/samplingRate));
  //oneMinusAlpha = 1.0 - alpha;
  
  analogReadResolution(10);
  analogWriteResolution(10);
  
  pinMode(ledPin, OUTPUT);
  pinMode(14, INPUT);
  pinMode(6, OUTPUT);
  digitalWrite(7, 1);
  digitalWrite(ledPin, 0);

  //MyTimer5.begin(samplingRate);
  //MyTimer5.attachInterrupt(detectZeroCrossing);
  // AdcBooster();
  // DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  //MyTimer5.start();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);

  // Setup PWM frequency f=50
  //pwm.setClockDivider(255, false);
  // pwm.timer([0, 1, 2], [1, 2, 4, 8, 16, 64, 256, 1024], [2-MaxSteps], [true, false]);
  // pwm.timer(1, 16, 255, false);
  
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

}

void loop() {
    while (ArduinoCloud.connected() == 0 || canStart == false) 
  {
    ArduinoCloud.update();
    Serial.println("Waiting for connection to Arduino IoT Cloud");
    delay(1000);
  }
  
 ArduinoCloud.update();
 toggleState = digitalRead(14); 
 Serial.println(toggleState);
 if(toggleState != prevToggleState && toggleState == 0) {
  Serial.println("TOGGLE");
    lcd.clear();
    // Frequency
    lcd.print("Hello :)");
  }

prevToggleState = toggleState;
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

void onCisManualChange()  {
  isManual = CisManual;
  if (isManual == true) {
    digitalWrite(ledPin, 1);
  } else {
    digitalWrite(ledPin, 0);
  }
  
}

void onCmanualAmpereChange() {
  if (isManual == true) {
    manualAmpere = CmanualAmpere;
  }   
}
