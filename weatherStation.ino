#include "Arduino.h"
//The setup function is called once at startup of the sketch
//#include "Arduino.h"
//The setup function is called once at startup of the sketch




// Control Arduino board from BLE

// Enable lightweight
#define LIGHTWEIGHT 1

// Libraries
// Added #ifndef _SPARKFUNHTU21D_ #define _SPARKFUNHTU21D_ #endif
// in SparkFunHTU21D.h to solve the htu21d class already added error messsage
#define _SPARKFUNHTU21D_

#include <SPI.h>
#include <Adafruit_BLE_UART.h>
#include <aREST.h>
#include <SparkFunHTU21D.h>

// Pins
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

// DHT sensor
//#define DHTPIN 7
//#define DHTTYPE DHT11

// DHT instance
//DHT dht(DHTPIN, DHTTYPE);
HTU21D htu21d;
// Create aREST instance
aREST rest = aREST();

// BLE instance
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

// Variables to be exposed to the API
int temperature;
int humidity;
int light;

void setup(void)
{
  // Start Serial
  Serial.begin(9600);

  // Start BLE
  BTLEserial.begin();

  // Give name and ID to device
  rest.set_id("001");
  rest.set_name("weather_station");

  // Expose variables to API
  rest.variable("temperature",&temperature);
  rest.variable("humidity",&humidity);
  rest.variable("light",&light);

   // Init DHT
  htu21d.begin();

  // Welcome message
  Serial.println("Weather station started");
}

void loop() {

  // Measure from DHT
  float t = htu21d.readTemperature();
  float h = htu21d.readHumidity();
  temperature = (int)t;
  humidity = (int)h;

  // Measure light level
  float sensor_reading = analogRead(A0);
  light = (int)(sensor_reading/1024*100);

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();

  // Handle REST calls
  if (status == ACI_EVT_CONNECTED) {
    rest.handle(BTLEserial);
    Serial.println("bonjour");

  }
 }
