// Code generated by Arduino IoT Cloud, DO NOT EDIT.
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
const char DEVICE_LOGIN_NAME[] = "47149b95-33b2-4a67-b6d5-17728eec0e59";
const char SSID[] = SECRET_SSID; // Network SSID (name)
const char PASS[] = SECRET_OPTIONAL_PASS; // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[] = SECRET_DEVICE_KEY; // Secret device password
float loadvoltage;
void initProperties(){
ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
ArduinoCloud.addProperty(loadvoltage, READ, 10 * SECONDS, NULL);
}
WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
/*
Sketch generated by the Arduino IoT Cloud Thing "Untitled"
https://create.arduino.cc/cloud/things/4d8ce79a-0e39-4864-be2a-d8e2204071a0
Arduino IoT Cloud Variables description
The following variables are automatically generated and updated when changes are made to the Thing
float loadvoltage;
Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
which are called when their values are changed from the Dashboard.
These functions are generated with the Thing and added at the end of this sketch.
*/
#include "thingProperties.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
//Adafruit_INA219 ina219_B(0x041);
float shuntvoltage = 0.00;
float busvoltage = 0.00;
float current = 0.00;
float power = 0.00;
void setup() {
Serial.begin(9600);
delay(1500);

// Defined in thingProperties.h
initProperties();
// Connect to Arduino IoT Cloud
ArduinoCloud.begin(ArduinoIoTPreferredConnection);
/*
The following function allows you to obtain more information
related to the state of network and IoT Cloud connection and errors
the higher number the more granular information you’ll get.
The default is 0 (only errors).
Maximum is 4
*/
setDebugMessageLevel(2);
ArduinoCloud.printDebugInfo();
if (! ina219.begin()) {
Serial.println("Failed to find INA219 chip");
while (1) {
delay(10);
}
}
ina219.setCalibration_16V_400mA();
Serial.println("IoT Energy Meter with INA219 ...");
}
void loop() {
ArduinoCloud.update();
ina219values();
}
void ina219values() {
shuntvoltage = ina219.getShuntVoltage_mV();
busvoltage = ina219.getBusVoltage_V();
current = ina219.getCurrent_mA();
power = ina219.getPower_mW();
loadvoltage = busvoltage + (shuntvoltage / 1000);
Serial.print("Bus Voltage: "); Serial.print(busvoltage); Serial.println(" V");
Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
Serial.print("Load Voltage: "); Serial.print(loadvoltage); Serial.println(" V");
Serial.print("Current: "); Serial.print(current); Serial.println(" mA");
Serial.print("Power: "); Serial.print(power); Serial.println(" mW");
Serial.println("-------------------------");
}
