#include "Timer5.h"
#include <LiquidCrystal.h>
#include <math.h>
#include <string.h>
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <SAMD21turboPWM.h>

#include "arduino_secrets.h"
#include "thingProperties.h"
#include "connection.h"
#include "control.h"
#include "zeroCrossing.h"
#include "setUpPIN.h"

#define PI 3.1415926535897932384626433832795

LiquidCrystal lcd(8, 4, 9, 10, 3, 2); 
TurboPWM pwm;



// Global Variables

// Frequency Calculation
volatile long int zeroCrossing = 0;
volatile float sample = 0.0;
float frequency = 0.0;
volatile int sampleCounter = 0;

const int measureFrequency = 50; 
const int samplingRate = 10000;
const int offsetValueZeroCrossing = 257;
const int resolution = 1023; 

// Low Pass Filter
const float crossOverFreq = 150.0;
float alpha;
float oneMinusAlpha; 

// Interpolation
volatile float oldY = 0.0;
volatile float newY = 0.0;
volatile float oldY_int = 0;
volatile float newY_int = 0;

float prevInterpol = 0.0;

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
bool isManual = false;      // initially manual control should be disabled
float manualAmpere = 0.0;

// Debugging Value for Arduino Cloud 
bool canStart = false;

void setup() {
  // Cloud SetUp 
  initProperties();
  // Uncomment for debugging connection Arduino - Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::CONNECT, doThisOnConnect);
  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::SYNC, doThisOnSync);
  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::DISCONNECT, doThisOnDisconnect);
  setDebugMessageLevel(DBG_INFO);
  ArduinoCloud.printDebugInfo();


  // Serial.begin(9600);
  // Setup Values for Low Pass Filter 
  alpha = calculateAlpha(crossOverFreq, (1.0/samplingRate));
  oneMinusAlpha = 1.0 - alpha;
  
  // Set Resolution of analog PINS
  analogReadResolution(10);
  analogWriteResolution(10);
  
  // Define PINS
  pinMode(ledPIN, OUTPUT);
  pinMode(v0LCDPIN, OUTPUT);
  pinMode(loadPIN, OUTPUT);
  pinMode(generatorPIN, OUTPUT);

  analogWrite(v0LCDPIN, 1);
  digitalWrite(loadPIN, 0);
  digitalWrite(ledPIN, 0);
  digitalWrite(generatorPIN, 0);
  
  // Initialize Interrupt
  // Apply ADC Booster to accelerate the analog read
  AdcBooster();
  MyTimer5.begin(samplingRate);
  MyTimer5.attachInterrupt(detectZeroCrossing);
  MyTimer5.start();
  
  // Initialize LCD
  lcd.begin(16, 2); // initialize LCD size
  lcd.setCursor(0, 0);

  // Setup PWM frequency f=1000 Hz
  pwm.setClockDivider(2, false);
  pwm.timer(0, 1, 24000, true); // port 5

}

void loop() {

// Debug Connection to Cloud 
//while (ArduinoCloud.connected() == 0 || canStart == false) 
//{
//   ArduinoCloud.update();
//   Serial.println("Waiting for connection to Arduino IoT Cloud");
//   delay(1000);
//}  


  if (zeroCrossing >= measureFrequency) {
   
    // this is important!
    // use scaling factors for frequency measurement (emperically determined)
    // don't use sample Rate - it is not accurate enough
    float period = (sampleCounter * (1.0 / 10927.0) * 1.015425 ) / zeroCrossing; // *1.014
    float interpolX = interpolation();

    // period -= (interpolX - prevInterpol);
    // prevInterpol = interpolX;
    
    
    frequency = 1.0/period;

    // Load and Generator Control
    generatorLoadControl(); 
    
    // RMS calculations
    calculate_RMS();
    
    // LCD prints
    printLCD();

    float PWMPercent = 0.0;
    if(isManual == false) {
      // Droop Control and PWM Calculation 
      float droopPower = droopControl();
      float PWMCurrent = calculateCurrent(droopPower);
      PWMPercent = dutyCycle(PWMCurrent);
      
      // Write values to cloud variables 
      Cpwmperc = PWMPercent;
      Campere = PWMCurrent;
      Cpower = droopPower;
      
    } else {
      // Manual Control means that the current selected by the user is used 
      PWMPercent = dutyCycle(manualAmpere); 
      Cpwmperc = PWMPercent;
      Cpower = manualAmpere*RMS_voltage;
      Campere = CmanualAmpere;
    }

    // wite PWM to PIN 5 
    pwm.analogWrite(PWMPIN, PWMPercent*10);

    // for debugging PWM Frequency 
    float z = pwm.frequency(0);
    
    // reset values for next period
    voltSum = 0;
    currentSum = 0;
    outerSampleCounter = 0;
    zeroCrossing = 0;
    sampleCounter = 0;

    // update the values on the cloud
    ArduinoCloud.update(); 
  }

    // map read frequency (without low pass filter) to voltage
    float volt = map(sample, 0, (resolution/2), -240*sqrt(2), 240*sqrt(2));
    // sum the square of volt to calculate RMS later
    voltSum += powf(volt, 2);
    outerSampleCounter++;   // counter for RMS calculation
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

void manualLED() {
  // shows the user if he can manually change the current 
  if (isManual == true) {
    digitalWrite(ledPIN, 1);
  } else {
    digitalWrite(ledPIN, 0);
  }
}
