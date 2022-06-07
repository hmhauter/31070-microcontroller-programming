void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  AdcBooster();
  DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  

}

void loop() {
  // put your main code here, to run repeatedly:
  int val = analogRead(A1);
  float volt = (3.3/1023) * val;
  analogWrite(A0, volt);
  Serial.println(volt);
  delay(1);

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
