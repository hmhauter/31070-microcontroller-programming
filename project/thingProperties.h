#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

#if defined(BOARD_HAS_WIFI)
#else
  #error "Arduino IoT Cloud currently only supports MKR1000, MKR WiFi 1010, MKR WAN 1300/1310, MKR NB 1500 and MKR GSM 1400"
#endif

#define THING_ID "0c2e4700-03de-4da1-b244-6426542eddc1"


// Read Only
float Cpower;
float Cpwmperc;
float Campere;

// Read and Write
float CmanualAmpere;
bool CisManual;

// Callback Functions for Writing 
void onCmanualAmpereChange();
void onCisManualChange();

void initProperties() {
  ArduinoCloud.addProperty(Cpower, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(Cpwmperc, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(Campere, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(CisManual, READWRITE, ON_CHANGE, onCisManualChange);
  ArduinoCloud.addProperty(CmanualAmpere, READWRITE, ON_CHANGE, onCmanualAmpereChange);
  ArduinoCloud.setThingId(THING_ID);
}

#if defined(BOARD_HAS_WIFI)
  WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_SSID, SECRET_PASS);
#endif
