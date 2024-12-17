// Definição das informações da aplicação na plataforma Blynk
#define BLYNK_TEMPLATE_ID "TMPLRuyBRdi-"
#define BLYNK_TEMPLATE_NAME "Iot Energy"
#define BLYNK_AUTH_TOKEN "NwC3COvxOHx_vBNaNpsslH2SC7sJ5R5w"
#define BLYNK_FIRMWARE_VERSION "0.1.0"
#define BLYNK_PRINT Serial
#define APP_DEBUG
#define USE_NODE_MCU_BOARD
// Inclusão das bibliotecas utilizadas
#include <Wire.h> // Biblioteca com funções para comunicação I2C com o controlador
#include <Adafruit_INA219.h> // Biblioteca com funções para leitura das variáveis elétricas da placa INA219
// Bibliotecas com funções para comunicação com uma rede WiFi
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h> // Biblioteca com funções de comunicação entre Blynk e ESP32
Adafruit_INA219 ina219; // Declara um objeto referente a Adafruit_INA219. Por default, o endereço da placa é 0x40
// Definição das variáveis temporais utilizadas para mostrar os valores no sistema supervisório
unsigned long previousMillis = 0;
unsigned long interval = 2000;
float shuntvoltage = 0.00; // Definição da variável para medição da tensão no resistor shunt
float busvoltage = 0.00; // Definição da variável para medição da tensão de barramento
float current = 0.00; // Definição da variável para medição da corrente na carga
float loadvoltage = 0.00; // Definição da variável para medição da tensão na carga
// SSID (Service Set Identifier) ou identificador da rede WiFi utilizada
char ssid[] = "Breno_2G";
// Senha da rede WiFi utilizada
char pass[] = "breno52";
// Função chamada sempre que há conexão com o Blynk Server
BLYNK_CONNECTED()
{
Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
Blynk.setProperty(V3, "onImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}
// Função de configuração
void setup()
{
Serial.begin(115200); // Inicia a comunicação serial com um baud rate de 115200
while (!Serial) {
delay(1);
}
Serial.begin(115200);
Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass); // Inicia a comunicação com a aplicação desenvolvida no Blynk através da rede WiFi

// Inicializa o INA219
if (! ina219.begin()) {
Serial.println("Falha em encontrar o INA219"); // Caso haja falha na inicialização do INA219, mostra no monitor serial a mensagem de que houve falha em encontrar o INA219
while (1) {
delay(10);
}
}
// Por default, a inicialização usa o intervalo de medição de 32 V e 2 A
// No entanto, podemos utilizar um intervalo de 32 V e 1 A com o seguinte comando:
// ina219.setCalibration_32V_1A();
// Também podemos utilizar um intervalo de 16 V e 400 mA com o seguinte comando:
ina219.setCalibration_16V_400mA();
Serial.println("Medição das variáveis elétricas com INA219 ..."); // Caso o sensor seja identificado com sucesso, o monitor serial mostra que está havendo medição das variáveis elétricas com INA219
}
// Função de repetição
void loop()
{
Blynk.run(); // Inicia a aplicação desenvolvida na plataforma Blynk
// A função millis() retorna o número de milissegundos passados desde que o ESP32 começou a executar o programa atual. Portanto, o código abaixo faz os valores das medidas das variáveis elétricas serem atualizadas a intervalores regulares dados pela variável interval
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= interval)
{
previousMillis = currentMillis;
// Função para obter a medição das variáveis elétricas com o INA219
ina219values();
// Função para mostrar as variáveis elétricas medidas com o INA219 na aplicação desenvolvida na plataforma Blynk
displaydata();
}
}
void ina219values() {
shuntvoltage = ina219.getShuntVoltage_mV(); // Função para obter a medição da tensão no resistor shunt em mV
busvoltage = ina219.getBusVoltage_V(); // Função para obter a medição da tensão de barramento em V
current = ina219.getCurrent_mA(); // Função para obter a medição da corrente na carga em mA
loadvoltage = busvoltage + (shuntvoltage / 1000); // Cálculo da tensão na carga
Serial.print("Load Voltage: "); Serial.print(loadvoltage); Serial.println(" V"); // Mostra a tensão na carga em V no monitor serial
Serial.print("Current: "); Serial.print(current); Serial.println(" mA"); Mostra a corrente na carga em mA no monitor serial
Serial.println("-------------------------");
}
void displaydata() {
// Mostra a tensão na carga no Widget Voltage mostrado na Fig. 18
Blynk.virtualWrite(V0, String(loadvoltage, 2) + String(" V") );
// Mostra a corrente na carga no Widget Current mostrado na Fig. 18
if (current > 1000) {
Blynk.virtualWrite(V1, String((current / 1000), 2) + String(" A") );
}

else
{
Blynk.virtualWrite(V1, String(current, 2) + String(" mA"));
}
}
