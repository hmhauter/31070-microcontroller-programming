# 31070-microcontroller-programming
This is the repository for our project from the course "Hands-on microcontroller programming", June 2022

###### Authors: 
Helena Hauter (s220274), Pauline Thüne (s220242)

###### Libraries: 
Following libraries are included and have to be installed on the Arduino IDE before using this code:
- Timer5: https://github.com/michael71/Timer5
- LiquidCrystal: https://github.com/arduino-libraries/LiquidCrystal
- ArduinoIoTCloud: https://www.arduino.cc/reference/en/libraries/arduinoiotcloud/
- SAMD21turboPWM: https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM

###### Objective: 
To avoid blackouts of the power system, it is essential to balance the supply and demand of power. Therefore, already slight imbalances need to be controlled. The project described in the following report aims to implement such frequency control for normal operation reserve, using an electric vehicel. Therefore, a microcontroller, a LCD and a dashboard, based on the Arduino IoT Cloud, are used to satisfy predefined requirements.