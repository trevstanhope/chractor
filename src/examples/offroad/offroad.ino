/*
Offroad kit for trail feedback
 
 This build supports the following sensor functions:
 - Roll
 - Pitch
 - Intake Temperature
 - Transfer Case Temperature
 - Steering Position
 - Front Suspension Position
 - Differential Ground Clearance Sensor
 
 */

/* --- Libraries --- */
#include <Canbus.h>
#include <ArduinoJson.h>
#include <RunningMedian.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "stdio.h"

/* --- Prototypes --- */
int checksum(char *buf);

/* --- Global --- */
// IO Pins
const unsigned int FAN_RELAY_PIN = 8;
const unsigned int TEMP_SENSOR_PIN = 5;
const unsigned int MOISTURE_DO_PIN = 6;
const unsigned int SENSOR_POWER_PIN = 7; 

// Et cetera
const unsigned int SAMPLES = 9;
const unsigned int OUTPUT_LENGTH = 256;
const unsigned int CANBUS_LENGTH = 8;
const unsigned int DATA_LENGTH = 128;
const unsigned int BAUD = 38400;
const unsigned int JSON_LENGTH = 256;
const unsigned int UPDATE_INTERVAL = 15000;
const unsigned int DIGITS = 2;
const unsigned int PRECISION = 2;

/* --- Variables --- */
int chksum;
bool canbus_status = 0;
bool pump_off = false;
bool fan_off = false;
bool irrigation_required = false;
bool cooling_required = false;

int temperature_pv = 0;
int moisture_pv = 0;

// Buffers
char output_buffer[OUTPUT_LENGTH];
char data_buffer[DATA_LENGTH];
unsigned char canbus_rx_buffer[CANBUS_LENGTH];  // Buffer to store the incoming data
unsigned char canbus_tx_buffer[CANBUS_LENGTH];  // Buffer to store the incoming data

// Objects
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature ds18b20(&oneWire);

/* --- Setup --- */
void setup() {

  // Initialize USB
  Serial.begin(BAUD);
  delay(10);

  // Initialise MCP2515 CAN controller at the specified speed
  int canbus_attempts = 0;
  while (!canbus_status) {
    canbus_status = Canbus.init(CANSPEED_500);
    delay(10);
  }

  // Sensors
  ds18b20.begin();
  pinMode(SENSOR_POWER_PIN, OUTPUT); // provide power to the moisture sensor
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  // Relays
  pinMode(FAN_RELAY_PIN, OUTPUT);
  digitalWrite(FAN_RELAY_PIN, fan_off);

}

/* --- Loop --- */
void loop() {

  // Check Temperature Sensor and Set Cooling Fan (Dual Threshold Method)
  for (int i = 0; i < SAMPLES; i++) {
    ds18b20.requestTemperatures();
    temperature_history.add(ds18b20.getTempCByIndex(0));
  }
  temperature_pv = int(temperature_history.getAverage());
  
  // Check CANBus
  if (canbus_status) {

    // Check CANBUS
    unsigned int UID = Canbus.message_rx(canbus_rx_buffer); // Check to see if we have a message on the Bus

    // Check ODB via CAN
    if (UID != 0) {
      // Read from network
      StaticJsonBuffer<JSON_LENGTH> json_buffer;
      JsonObject& root = json_buffer.createObject();
      if (msg == GET_RESPONSE) {
        if (nt == MOISTURE_CONTROL_V1) {
          root["moisture"] = canbus_rx_buffer[4];
          root["irrigating"] = canbus_rx_buffer[5];
          root["temperature"] = canbus_rx_buffer[6];
          root["cooling"] = canbus_rx_buffer[7];
        }
      }
    }
  }

  // Print Data to JSON Buffer
  StaticJsonBuffer<JSON_LENGTH> json_buffer;
  JsonObject& root = json_buffer.createObject();
  root["temp"] = temperature_pv;
  root.printTo(data_buffer, sizeof(data_buffer));
  int chksum = checksum(data_buffer);
  sprintf(output_buffer, "{\"data\":%s,\"chksum\":%d}", data_buffer, chksum);
  Serial.println(output_buffer);

  // Wait
  delay(UPDATE_INTERVAL);

}

/* --- Functions --- */
// Checksum
int checksum(char* buf) {
  int sum = 0;
  for (int i = 0; i < DATA_LENGTH; i++) {
    sum = sum + buf[i];
  }
  return sum % 256;
}

int getFractionalPart(float value) {
  int int_part, frac_part;
  char buf[16];
  dtostrf(value, DIGITS, PRECISION, buf); 
  sscanf(buf, "%d.%d", &int_part, &frac_part);
  return frac_part;
}

int getIntegerPart(float value) {
  int int_part, frac_part;
  char buf[16];
  dtostrf(value, DIGITS, PRECISION, buf); 
  sscanf(buf, "%d.%d", &int_part, &frac_part);
  return int_part;
}

float getMoistureContent(int pin) {
  int v = analogRead(pin);
  float mc = (float)(v - volts_a) * (moisture_b - moisture_a) / (float)(volts_b - volts_a) + moisture_a;
  return mc;
}
