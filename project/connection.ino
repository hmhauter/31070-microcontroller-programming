// This file handels everything that has to do with the connection between the MKR1000 and the IOT Cloud
#include "setUpPIN.h"

// These are the onChange functions of the READ and WRITE variables of the IOT Cloud
void onCisManualChange()  {
  // is manual describes if the user wants to change the current manually
  isManual = CisManual;
  onCmanualAmpereChange();
  // activate or deactivate LED 
  manualLED(); 
}

void onCmanualAmpereChange() {
  // updates value if user changes current on the dashboard 
  if (isManual == true) {
    manualAmpere = CmanualAmpere;
  }   
}

void manualLED() {
  // shows the user if he can manually change the current 
  if (isManual == true) {
    digitalWrite(ledPIN, 1);
  } else {
    digitalWrite(ledPIN, 0);
  }
}


// The functions are especially usefull for debugging the connection of the MCR and the IOT Cloud 
void doThisOnConnect() {
  Serial.println("Board successfully connected to Arduino IoT Cloud");
}
void doThisOnSync() {
  Serial.println("Thing Properties synchronised");
  isSynchronised = true;
  
}
void doThisOnDisconnect() {
  Serial.println("Board disconnected from Arduino IoT Cloud");
  isSynchronised = false;
}
