// Inclusão das bibliotecas utilizadas
#include <Wire.h> // Biblioteca com funções para comunicação I2C com o controlador
#include <Adafruit_INA219.h> // Biblioteca com funções para leitura das variáveis elétricas da placa INA219
Adafruit_INA219 ina219; // Declara um objeto referente a Adafruit_INA219. Por default, o endereço da placa é 0x40
// Função de configuração
void setup(void)
{
Serial.begin(115200); // Inicia a comunicação serial com um baud rate de 115200
while (!Serial) {
delay(1);
}
Serial.println("Bem-vindo!"); // Mostra a mensagem bem-vindo no monitor serial do IDE do Arduino
// Inicializa o INA219
if (! ina219.begin()) {
Serial.println("Falha em encontrar o INA219"); // Caso haja falha na inicialização do INA219, mostra no monitor serial a mensagem de que houve falha em encontrar o INA219
while (1) { delay(10); }
}
// Por default, a inicialização usa o intervalo de medição de 32 V e 2 A
// No entanto, podemos utilizar um intervalo de 32 V e 1 A com o seguinte comando:
// ina219.setCalibration_32V_1A();
// Também podemos utilizar um intervalo de 16 V e 400 mA com o seguinte comando:
ina219.setCalibration_16V_400mA();
Serial.println("Medição das variáveis elétricas com INA219 ..."); // Caso o sensor seja identificado com sucesso, o monitor serial mostra que está havendo medição das variáveis elétricas com INA219
}
// Função de repetição
void loop(void)
{
// Definição das variáveis de medição
float shuntvoltage = 0; // Definição da variável para medição da tensão no resistor shunt
float busvoltage = 0; // Definição da variável para medição da tensão de barramento
float current_mA = 0; // Definição da variável para medição da corrente na carga
float loadvoltage = 0; // Definição da variável para medição da tensão na carga
float power_mW = 0; // Definição da variável para medição da potência na carga
shuntvoltage = ina219.getShuntVoltage_mV(); // Função para obter a medição da tensão no resistor shunt em mV
busvoltage = ina219.getBusVoltage_V(); // Função para obter a medição da tensão de barramento em V
current_mA = ina219.getCurrent_mA(); // Função para obter a medição da corrente na carga em mA
power_mW = ina219.getPower_mW(); // Função para obter a medição da potência na carga em mW
loadvoltage = busvoltage + (shuntvoltage / 1000); // Cálculo da tensão na carga

// Funções para mostrar as variáveis de medição na tela
Serial.print("Tensão de Barramento: "); Serial.print(busvoltage); Serial.println(" V"); // Mostra a tensão de barramento em V no monitor serial
Serial.print("Tensão de Shunt: "); Serial.print(shuntvoltage); Serial.println(" mV"); // Mostra a tensão no resistor shunt em mV no monitor serial
Serial.print("Tensão na Carga: "); Serial.print(loadvoltage); Serial.println(" V"); // Mostra a tensão na carga em V no monitor serial
Serial.print("Corrente na Carga: "); Serial.print(current_mA); Serial.println(" mA"); // Mostra a corrente na carga em mA no monitor serial
Serial.print("Potência na Carga: "); Serial.print(power_mW); Serial.println(" mW"); // Mostra a potência na carga em mW no monitor serial
Serial.println("");
delay(2000);
}
