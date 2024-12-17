#//include <Blynk.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <ThingSpeak.h>

int intervalSensor = 2000;
long prevMillisThingSpeak = 0;
int intervalThingSpeak = 15000; // Intervalo minímo para escrever no ThingSpeak write é de 15 segundos



const char ssid[] = "Juan";
const char password[] = "17207511";
WiFiClient client;


const long CHANNEL = 2396474;
const char *WRITE_API = "WWZOJZLTJ2P0COPY";



Adafruit_INA219 ina219;
Adafruit_INA219 ina219_B(0x041);

float shuntvoltage = 0.00;
float busvoltage = 0.00;
float current = 0.00;
float loadvoltage = 0.00;
float power = 0.00;
float energy = 0.00;

float shuntvoltageB = 0.00;
float busvoltageB = 0.00;
float currentB = 0.00;
float loadvoltageB = 0.00;
float powerB = 0.00;
float energyB = 0.00;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Envia os dados do sensor para o ThingSpeak usando o ESP32");
  Serial.println();

  WiFi.mode(WIFI_STA); //Modo Station
  ThingSpeak.begin(client);  // Inicializa o ThingSpeak

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) {
      delay(10);
    }
  }

  if (! ina219_B.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) {
      delay(10);
    }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();
  ina219_B.setCalibration_16V_400mA();

  Serial.println("IoT Energy Meter with INA219 ...");
}

void loop(){

ina219values();
   
 
  // Conecta ou reconecta o WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Atenção para conectar o SSID: ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      Serial.print(".");
      delay(5000);
    }
    Serial.println("\nConectado");
  }

    delay(1000);     

  if (millis() - prevMillisThingSpeak > intervalThingSpeak) {

    // Configura os campos com os valores
    ThingSpeak.setField(1,loadvoltage);
    ThingSpeak.setField(2,loadvoltageB);
    ThingSpeak.setField(3,current);
    ThingSpeak.setField(4,currentB);
    ThingSpeak.setField(5,power);
    ThingSpeak.setField(6,powerB);
    ThingSpeak.setField(7,energy);
    ThingSpeak.setField(8,energyB);


    // Escreve no canal do ThingSpeak 
    int x = ThingSpeak.writeFields(CHANNEL, WRITE_API);
    if (x == 200) {
      Serial.println("Update realizado com sucesso");
    }
    else {
      Serial.println("Problema no canal - erro HTTP " + String(x));
    }

    prevMillisThingSpeak = millis();
  }
}



void ina219values()  {
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  power = loadvoltage * current;
  energy = energy + power / 3600;

  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power); Serial.println(" mW");
  Serial.println("-------------------------");

  shuntvoltageB = ina219_B.getShuntVoltage_mV();
  busvoltageB = ina219_B.getBusVoltage_V();
  currentB = ina219_B.getCurrent_mA();
  loadvoltageB = busvoltageB + (shuntvoltageB / 1000);
  powerB = loadvoltageB * currentB;
  energyB = energyB + powerB / 3600;

  Serial.print("Bus VoltageB:   "); Serial.print(busvoltageB); Serial.println(" V");
  Serial.print("Shunt VoltageB: "); Serial.print(shuntvoltageB); Serial.println(" mV");
  Serial.print("Load VoltageB:  "); Serial.print(loadvoltageB); Serial.println(" V");
  Serial.print("CurrentB:       "); Serial.print(currentB); Serial.println(" mA");
  Serial.print("PowerB:         "); Serial.print(powerB); Serial.println(" mW");
  Serial.println("-------------------------");

}
