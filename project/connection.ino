// This file handels everything that has to do with the connection between the MKR1000 and the IOT Cloud

// These are the onChange functions of the READ and WRITE variables of the IOT Cloud

void onCisManualChange()  {
  isManual = CisManual;
  manualLED(); 
}

void onCmanualAmpereChange() {
  if (isManual == true) {
    manualAmpere = CmanualAmpere;
  }   
}


// The functions are especially usefull for debugging the connection of the MCR and the IOT Cloud 
void doThisOnConnect() {
  Serial.println("Board successfully connected to Arduino IoT Cloud");
}
void doThisOnSync() {
   Serial.println("Thing Properties synchronised");
  canStart = true;
  
}
void doThisOnDisconnect() {
  Serial.println("Board disconnected from Arduino IoT Cloud");
}