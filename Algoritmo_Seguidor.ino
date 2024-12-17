#include <util/atomic.h> 

// Tentar aplicar filtros no sinal do encoder

// Eixo 1 - Base
#define ENCA_1 2 // Leitura do encoder A - Pino de interrupção externa
#define ENCB_1 3 // Leitura do encoder B - Pino de interrupção externa
// Pinos de controle do sentido de rotação
#define IN2_1 4 
#define IN1_1 5
// Pino de controle do PWM do motor
#define PWM_1 6

// Eixo 2 - Suporte
#define ENCA_2 18 // Leitura do encoder A - Pino de interrupção externa 
#define ENCB_2 19 // Leitura do encoder B - Pino de interrupção externa 
// Pinos de controle do sentido de rotação
#define IN2_2 10
#define IN1_2 11
// Pino de controle do PWM do motor
#define PWM_2 12

// Botão de segurança
#define BOTAO 21 // Pino de interrupção externa
volatile int estadoBotao; // Estado do botão de segurança

// LDRs para leitura da luminosidade solar
#define LDRSD A0 // Entrada analógica do LDR superior direito
#define LDRSE A1 // Entrada analógica do LDR superior esquerdo
#define LDRIE A2 // Entrada analógica do LDR inferior esquerdo
#define LDRID A3 // Entrada analógica do LDR inferior direito
 
// Variáveis referentes ao encoder do eixo 1
volatile int posi_1 = 0; // Leitura do encoder 
volatile byte ant_1  = 0; // Estado anterior do encoder
volatile byte act_1  = 0; // Estado atual do encoder 
volatile int posi_1_global = 0; // Leitura do encoder

// Variáveis referentes ao encoder do eixo 2
volatile int posi_2 = 0; // Leitura do encoder  
volatile byte ant_2  = 0; // Estado anterior do encoder
volatile byte act_2  = 0; // Estado atual do encoder 
volatile int posi_2_global = 0;

// depois preciso definir período de amostragem
// unsigned long sampleTime = 100;
// void loop() {
//   if (millis() - lastTime >= sampleTime || lastTime==0)
//   {  // Se actualiza cada sampleTime (milisegundos)
//       lastTime = millis();
//       Serial.print("Numero de cuentas: ");Serial.println(n);
//    }
// }

// Inicializações, configurações e determinação de entradas e saídas 
void setup() 
{
// Inicialização da comunicação serial com um baud rate específico
Serial.begin(9600);
// Definição das entradas e saídas
pinMode(BOTAO,INPUT_PULLUP); // Utiliza um resistor de pull-up interno do Arduino para o botão de segurança
pinMode(ENCA_1,INPUT);
pinMode(ENCB_1,INPUT);
pinMode(ENCA_2,INPUT);
pinMode(ENCB_2,INPUT);
pinMode(IN1_1,OUTPUT);
pinMode(IN2_1,OUTPUT);
pinMode(PWM_1,OUTPUT);
pinMode(IN1_2,OUTPUT);
pinMode(IN2_2,OUTPUT);
pinMode(PWM_2,OUTPUT);
// Interrupção do botão de segurança
// Botão pressionado (LOW) / Botão não pressionado (HIGH)
attachInterrupt(digitalPinToInterrupt(BOTAO),parar,LOW); 
// Interrupção do encoder do motor do eixo 1
attachInterrupt(digitalPinToInterrupt(ENCA_1),encoder_1,CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCB_1),encoder_1,CHANGE);
// Interrupção do encoder do motor do eixo 2
attachInterrupt(digitalPinToInterrupt(ENCA_2),encoder_2,CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCB_2),encoder_2,CHANGE);
Serial.println("Ref Pos"); // Plota Valor de referência e posição do eixo
}

// Função principal 
void loop() 
{
// Leitura da luminosidade solar pelos LDRs 
int SupDir = analogRead(LDRSD); // Valor da leitura do LDR superior direito
// Serial.print("SupDir = "); 
// Serial.println(SupDir);
int SupEsq = analogRead(LDRSE); // Valor da leitura do LDR superior esquerdo
// Serial.print("SupEsq = "); 
// Serial.println(SupEsq);
int InfEsq = analogRead(LDRIE); // Valor da leitura do LDR inferior esquerdo
// Serial.print("InfEsq = "); 
// Serial.println(InfEsq);
int InfDir = analogRead(LDRID); // Valor da leitura do LDR inferior direito
// Serial.print("InfDir = "); 
// Serial.println(InfDir);

// Lógica para seguir o caminho do sol de acordo com a luminosidade medida - Eixo 1
int ValorDir = (SupDir + InfDir) / 2; // Média da leitura dos LDRs da direita
int ValorEsq = (SupEsq + InfEsq) / 2; // Média da leitura dos LDRs da esquerda
int DifDirEsq = ValorDir - ValorEsq; // Diferença entre os LDRs da direita e da esquerda
int TolDirEsq = 50; // Tolerância entre as medidas dos LDRs da direita e da esquerda. Determina o início do movimento do eixo 1
// talvez usar histerese

// Variáveis de referência e limites de segurança do eixo 1
const int cpr_1 = 367/90; // Resolução do encoder - Aproximadamente 367 contagens em 90 graus
int ref_1 = (0)*(cpr_1); // Referência de posição 
int max_1 = (70)*(cpr_1); // Posição de segurança máxima  
int min_1 = (-70)*(cpr_1); // Posição de segurança mínima 
// Variáveis referentes ao controle PID do eixo 1
long prevT_1 = 0; // Tempo anterior
float eprev_1 = 0; // Erro anterior
float eintegral_1 = 0; // Erro integral
float eintegralant_1 = 0; // Erro integral
int e_1; // Erro
int pos_1 = 0; // Posição do eixo
int dead_zone_11 = 0; // Compensação da zona morta para o eixo 1
int dead_zone_12 = 0; // Compensação da zona morta para o eixo 1
int dir_1; // Sentido de rotação para o eixo 1
long currT0_1; // Tempo atual antes do laço de controle de movimentação do eixo 1
long currT_1; // Tempo atual dentro do laço de controle de movimentação do eixo 1
long T_cont_1 = 1000000; // Tempo de execução do laço de controle em microsegundos
long timeInterval_1 = 500;
// Parâmetros do controle PID do eixo 1  
float kp_1 = 600; // Ganho proporcional
float kd_1 = 10; // Ganho derivativo
float ki_1 = 100; // Ganho integral

// Movimentação do eixo 1
if (-1 * TolDirEsq > DifDirEsq || DifDirEsq > TolDirEsq) 
{  
if (ValorDir > ValorEsq) 
{
ref_1 = (10)*(cpr_1); // Passo ou referência de 40 graus a cada mudança do sol
}
else if (ValorDir < ValorEsq)   
{
ref_1 = -(10)*(cpr_1); // Passo ou referência de -40 graus a cada mudança do sol
}

//delay(2000); // Aguarda 2s
posi_1 = 0; // Zera a leitura do encoder do eixo 1
pos_1 = 0; // Zera a leitura do encoder do eixo 1
currT0_1 = micros(); // Tempo atual antes do laço de controle do eixo 1

// Laço de controle de movimentação do eixo 1
do
{
currT_1 = micros(); // Tempo atual dentro do laço de controle do eixo 1
if(currT_1-prevT_1>=timeInterval_1)
{
float deltaT_1 = ((float) (currT_1 - prevT_1))/( 1.0e6 ); // Passo temporal
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
{
pos_1 = posi_1;
}
e_1 = (pos_1 - ref_1)/(cpr_1); // Erro 
// não deveria ser o contrário? ref_1 - pos_1?
float dedt_1 = (e_1-eprev_1)/(deltaT_1); // Derivada do erro
// eintegral_1 = eintegral_1 + e_1*deltaT_1; // Integral do erro
eintegral_1 = eintegralant_1 + (e_1+eprev_1)*deltaT_1; // Integral do erro 
// ACHO QUE PRECISA DIVIDIR ESTA INTEGRAL DO ERRO POR 2 - TESTAR
float u_1 = kp_1*e_1 + kd_1*dedt_1 + ki_1*eintegral_1; // Sinal de controle PID
float pwr_1 = fabs(u_1); // Valor do PWM

// Lógica para determinação do sentido de rotação e valor do PWM do motor
if(u_1>=0)
{
dir_1 = -1;
pwr_1 = pwr_1 + dead_zone_11; // Compensação da zona morta para o eixo 1
}
else
{
dir_1 = 1;
pwr_1 = pwr_1 + dead_zone_12; // Compensação da zona morta para o eixo 1
}
// Saturação do motor 
if( pwr_1 > 255 )
{
pwr_1 = 255;
eintegral_1 = eintegralant_1;
}

// Limites de segurança do eixo 1
if(pos_1>=max_1 || pos_1<=min_1 )
{
setMotor(0,0,PWM_1,IN1_1,IN2_1); 
}
else
{
setMotor(dir_1,pwr_1,PWM_1,IN1_1,IN2_1);
}

eprev_1 = e_1;
eintegralant_1 = eintegral_1;

Serial.print(ref_1/cpr_1); // Plota Valor de referência do eixo 1
Serial.print(" ");
Serial.print(pos_1/cpr_1); // Plota posição do eixo 1
Serial.println(); 
prevT_1 = currT_1; 
}
} // Fim do laço de controle de movimentação do eixo 1
while((currT_1-currT0_1)<T_cont_1); 

eprev_1 = 0;
setMotor(0,0,PWM_1,IN1_1,IN2_1); // Para o motor do eixo 1
//delay(2000); // Aguarda 2s
posi_1_global = posi_1_global + pos_1/cpr_1;
Serial.print(posi_1_global); // Plota posição global do eixo 2
posi_1 = 0; // Zera a leitura do encoder do eixo 1


} // Fim do laço de movimentação do eixo 1

else // Se não há leitura de variação de luminosidade, não movimenta o eixo 1
{
setMotor(0,0,PWM_1,IN1_1,IN2_1); 
}

delay(2000); // Aguarda 2s para iniciar a movimentação do eixo 2

// Lógica para seguir o caminho do sol de acordo com a luminosidade medida - Eixo 2
int ValorSup = (SupDir + SupEsq) / 2; // Média da leitura dos LDRs superiores
int ValorInf = (InfDir + InfEsq) / 2; // Média da leitura dos LDRs inferiores
int DifSupInf = ValorSup - ValorInf; // Diferença entre os LDRs superiores e inferiores
int TolSupInf = 50; // Tolerância entre as medidas dos LDRs superiores e inferiores. Determina o início do movimento do eixo 2 
// talvez usar histerese

// Variáveis de referência e limites de segurança do eixo 2
const int cpr_2 = 750/90; // Resolução do encoder - Aproximadamente 750 contagens em 90 graus
int ref_2 = (0)*(cpr_2); // Referência de posição 
int max_2 = (50)*(cpr_2); // Posição de segurança máxima  
int min_2 = (-50)*(cpr_2); // Posição de segurança mínima 
// Variáveis referentes ao controle PID do eixo 2
long prevT_2 = 0; // Tempo anterior
float eprev_2 = 0; // Erro anterior
float eintegral_2 = 0; // Erro integral
float eintegralant_2 = 0; // Erro integral
int e_2; // Erro
int pos_2 = 0; // Posição do eixo
int dead_zone_21 = 0; // Compensação da zona morta para o eixo 2
int dead_zone_22 = 0; // Compensação da zona morta para o eixo 2
int dir_2; // Sentido de rotação para o eixo 2
long currT0_2; // Tempo atual antes do laço de controle de movimentação do eixo 2
long currT_2; // Tempo atual dentro do laço de controle de movimentação do eixo 2
long T_cont_2 = 1000000; // Tempo de execução do laço de controle em microsegundos
long timeInterval_2 = 500;
// Parâmetros do controle PID do eixo 2  
float kp_2 =600; // Ganho proporcional
float kd_2 = 10; // Ganho derivativo
float ki_2 = 100; // Ganho integral

// Movimentação do eixo 2 
if (-1 * TolSupInf > DifSupInf || DifSupInf > TolSupInf)
{  
if (ValorSup > ValorInf)  
{
ref_2 = (20)*(cpr_2); // Passo ou referência de 15 graus a cada mudança do sol
}
else if (ValorSup < ValorInf)   
{
ref_2 = -(20)*(cpr_2); // Passo ou referência de -15 graus a cada mudança do sol
}

//delay(2000); // Aguarda 2s
posi_2 = 0; // Zera a leitura do encoder do eixo 2
pos_2 = 0;
currT0_2 = micros(); // Tempo atual antes do laço de controle do eixo 2

// Laço de controle de movimentação do eixo 2
do
{
currT_2 = micros(); // Tempo atual dentro do laço de controle do eixo 2
if(currT_2 - prevT_2>=timeInterval_2)
{
float deltaT_2 = ((float) (currT_2 - prevT_2))/( 1.0e6 ); // Passo temporal
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
{
pos_2 = posi_2;
}
e_2 = (pos_2 - ref_2)/(cpr_2); // Erro
// não deveria ser o contrário? ref_2 - pos_2?
float dedt_2 = (e_2-eprev_2)/(deltaT_2); // Derivada do erro
// eintegral_2 = eintegral_2 + e_2*deltaT_2; // Integral do erro
eintegral_2 = eintegralant_2 + (e_2+eprev_2)*deltaT_2; // Integral do erro
// ACHO QUE PRECISA DIVIDIR ESTA INTEGRAL DO ERRO POR 2 - TESTAR
float u_2 = kp_2*e_2 + kd_2*dedt_2 + ki_2*eintegral_2; // Sinal de controle PID
float pwr_2 = fabs(u_2); // Valor do PWM

// Lógica para determinação do sentido de rotação e valor do PWM do motor
if(u_2>=0)
{
dir_2 = 1;
// if(fabs(posi_2_global)<=60)
// {
// pwr_2 = pwr_2 + dead_zone_21; // Compensação da zona morta para o eixo 2
// }
pwr_2 = pwr_2 + dead_zone_21;
}

else
{
dir_2 = -1;
// if(fabs(posi_2_global)<=60)
// {
// pwr_2 = pwr_2 + dead_zone_22; // Compensação da zona morta para o eixo 2
// }
pwr_2 = pwr_2 + dead_zone_22;
}

// Saturação do motor 
if( pwr_2 > 255 )
{
pwr_2 = 255;
eintegral_2 = eintegralant_2;
}

// Limites de segurança do eixo 2
if(pos_2>=max_2 || pos_2<=min_2 )
{
setMotor(0,0,PWM_2,IN1_2,IN2_2); 
}
else
{
setMotor(dir_2,pwr_2,PWM_2,IN1_2,IN2_2);
}

eprev_2 = e_2;
eintegralant_2 = eintegral_2;

Serial.print(ref_2/cpr_2); // Plota Valor de referência do eixo 2
Serial.print(" ");
Serial.print(pos_2/cpr_2); // Plota posição do eixo 2
Serial.println();
prevT_2 = currT_2;
}
} // Fim do laço de controle de movimentação do eixo 2
while((currT_2-currT0_2)<T_cont_2); 

eprev_2 = 0;
setMotor(0,0,PWM_2,IN1_2,IN2_2); // Para o motor do eixo 2
//delay(2000); // Aguarda 2s
posi_2_global = posi_2_global + pos_2/cpr_2;
Serial.print(posi_2_global); // Plota posição global do eixo 2
posi_2 = 0; // Zera a leitura do encoder do eixo 1

} // Fim do laço de movimentação do eixo 2

else // Se não há leitura de variação de luminosidade, não movimenta o eixo 2
{
setMotor(0,0,PWM_2,IN1_2,IN2_2); 
}

delay(2000); // Aguarda 2s para iniciar a movimentação do eixo 1

// delay(2000); // Aguarda xs para uma nova movimentação completa

} // Fim do laço loop

// Função para acionamento dos motores
// Os parâmetros são: sentido de rotação, valor do PWM do motor e pinos
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm,pwmVal);
  if(dir == 1)
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else if(dir == -1)
  {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

// Interrupção do botão de segurança
void parar()
{
estadoBotao = digitalRead(BOTAO);

// Botão pressionado (LOW) / Botão não pressionado (HIGH)
if (estadoBotao == LOW)
{
digitalWrite(IN1_1,LOW);
digitalWrite(IN2_1,LOW);
digitalWrite(IN1_2,LOW);
digitalWrite(IN2_2,LOW);
} 
}

// Interrupção do encoder do motor do eixo 1
void encoder_1(void)
{
ant_1 = act_1;
  
if(digitalRead(ENCA_1)) bitSet(act_1,1); else bitClear(act_1,1);            
if(digitalRead(ENCB_1)) bitSet(act_1,0); else bitClear(act_1,0);

if(ant_1 == 2 && act_1 ==0) posi_1++;
if(ant_1 == 0 && act_1 ==1) posi_1++;
if(ant_1 == 3 && act_1 ==2) posi_1++;
if(ant_1 == 1 && act_1 ==3) posi_1++;
  
if(ant_1 == 1 && act_1 ==0) posi_1--;
if(ant_1 == 3 && act_1 ==1) posi_1--;
if(ant_1 == 0 && act_1 ==2) posi_1--;
if(ant_1 == 2 && act_1 ==3) posi_1--;    
}

// Interrupção do encoder do motor do eixo 2
void encoder_2(void)
{
ant_2 = act_2;
  
if(digitalRead(ENCA_2)) bitSet(act_2,1); else bitClear(act_2,1);            
if(digitalRead(ENCB_2)) bitSet(act_2,0); else bitClear(act_2,0);

if(ant_2 == 2 && act_2 ==0) posi_2++;
if(ant_2 == 0 && act_2 ==1) posi_2++;
if(ant_2 == 3 && act_2 ==2) posi_2++;
if(ant_2 == 1 && act_2 ==3) posi_2++;
  
if(ant_2 == 1 && act_2 ==0) posi_2--;
if(ant_2 == 3 && act_2 ==1) posi_2--;
if(ant_2 == 0 && act_2 ==2) posi_2--;
if(ant_2 == 2 && act_2 ==3) posi_2--;       
}
