
/*
Sensor: DS18B20
Library: OneWire.h
<DallasTemperature.h>
*/

#include <OneWire.h>
#include <DallasTemperature.h>
// #include <pincapsense.h>

//PINS
const int START_PIN = 4; 
const int pin_mv = 9; // PWM
const int pin_pv = A4; //Pino de entrada para o variável medida
const int MIX_PIN = 2;
const int DS18B20_PIN = 3; // Pino do sensor
const int BUZZER_PIN = 10; // Pino do buzzer
float initial_temp; // temperatura inicial

#define Nota_E7 2637

//PID
float interrupt_s = 1/1000; // tempo do interrupt em s
float kp = 0.01;  //constante ganho proporcional
float Ti = 0.5;   //Tempo Integral
float Td = .1;    //Tempo Integral
float erro = 0;   //Erro
float erro_ant = 0; //erro anterior para ação derivativa
float P = 0;    //Ação Proporcional
float S = 0;    //Integrador
float I = 0;    //Ação Integral
float D = 0;    //Ação Integral
float acao; 

const int lim_int_sup = 29000;
const int lim_int_inf = -29000;
const int lim_pwm_sup = 254;// adotou-se 254 porque a saída do pwm em 255 é muito diferente do valor em 254, podendo gerar instabilidade na malha fechada
const int valor_cap;

//GENERAL CONTROL
bool start = 0; 
bool first_loop = 1;
bool second_loop = 0;
bool third_loop = 0;

float temp = 0;
float time_95;
float time_45;
bool loop_once_first = 1;
bool loop_once_second = 0;
bool loop_once_third = 0;
bool begin = 0;
bool mix_button = 0;

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

float PID(float temp_aim)
  {
    /*
    Função que mantem a temperatura em no valor recebido, 
    aquece até lá se necessário 
    */
    // Serial.println(temp_aim);
      float spValue = temp_aim; //leitura do valor de potenciometro
      float pvValue = analogRead(pin_pv); //leitura do valor de setpoint (Sensor temp)

      erro = pvValue - spValue;
        
      P = erro * kp;
      
      S += erro;    //fórmula do integrador
      if (S > lim_int_sup){S = lim_int_sup;}//limitador Integral superior
      if (S < lim_int_inf) {S = lim_int_inf;}//limitador Integral inferior
      
      I = S / Ti; //fórmula da ação integral
      
      D = (erro_ant - erro) * Td;
      
      erro_ant = erro;
      
      acao = P + I + D;   //cálculo da ação final do controlador PI
      if (acao > lim_pwm_sup) //limitador saída superior pwm
      {
             acao = lim_pwm_sup;
      }
      if (acao < 0) //limitador saída inferior pwm
      {
             acao = 0;
      }
      // Serial.print("I: ");
      // Serial.println(I);
      analogWrite(pin_mv, acao); //linha principal para ação na variável manipulada
	}

// Função para ler a temperatura do sensor:
float readTemperature() {
  sensors.requestTemperatures(); // Comando para ler as temperaturas
  return sensors.getTempCByIndex(0); // Le a temperatura em Celsius 
}

void buzzer(int val){
  int tempo = 4000;
  if(val == 0){
    tone(BUZZER_PIN,294,tempo); //RE
  }
  if(val == 1){
    tone(BUZZER_PIN,440,tempo); //LA
  }
  if(val == 2){
    tone(BUZZER_PIN,349,tempo/2); // FA
  }
}

void setup() {
  Serial.begin(9600);
  // inicialization pins
  pinMode(START_PIN, INPUT); //START switch pin
  pinMode(MIX_PIN,INPUT);
  // temperature pins
  pinMode(START_PIN, INPUT); //ON switch pin
  // Inicializando o sensor
  sensors.begin();
}

void loop() {
  start = digitalRead(START_PIN);
  // Serial.println(start);
  if (start){begin = 1;}
  
  // put your main code here, to run repeatedly:
  if (begin){
    
    if(first_loop){
      temp = readTemperature();
      PID(95);
      Serial.print(" temp: ");
      Serial.println(temp);
      // se a temperatura estiver entre 95
      if (temp > 29 && loop_once_first){
        time_95 = millis();  // calcula tempo
        loop_once_first = 0;      
      }

      if ((millis() - time_95 > 1*60*1000) && !loop_once_first){ //depois de 5 minutos
        first_loop = 0;
        second_loop = 1;
        loop_once_second = 1;
      }     
    }
    
    
    if(second_loop){
        temp = readTemperature();
        PID(45);
        
        if(loop_once_second && temp < 46){
          buzzer(0);
          loop_once_second = 0;
        }

        mix_button = digitalRead(MIX_PIN);
        if(mix_button){
          second_loop = 0;
          third_loop = 1;
        }
    }

    if(third_loop){
      temp = readTemperature();
      PID(45);
      if (loop_once_third){
        time_45 = millis();  // calcula tempo
        loop_once_third = 0;      
      }

      if (millis() - time_45 > 6*60*60*1000 && !loop_once_third){ //depois de 6 horas
        third_loop = 0;
        buzzer(1);
        begin = 0;
      }     
    }
  }
}
