
/*
  Controle pid de um secador de pratos
  disciplina de controle por computador
  2022/1ª
  ruan
  icaro
  luciano

*/
// Bibliotecas para o display LCD
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <PID_v1.h>

// Biblioteca para o encoder
#include <RotaryEncoder.h>

//bibliotecas dos sensores
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//biblioteca de timer. Usa o millis() mas não polui o codigo
#include <TimeLib.h>


// Defines Gerais
#define addr_lcd 0x3F
#define size_ldc_c 20
#define size_ldc_l 4

//defines dos sensores
#define DHTPIN A1
#define DHTTYPE DHT22
#define DS18B20PIN 52


// Definições Entradas Digitais
#define S1    2  // Chave táctil 1    // troca entre configuracoes e controle
//#define S2    3  // Chave táctil 2  // problema de hardware
#define S3    4  // Chave táctil 3
#define S4    5  // Chave táctil 4
#define S5    6  // Chave táctil do Encoder
#define ENA   7  // Encoder Sentido Horário
#define ENB   8 // Encoder Sentido Anti-Horário

// Definições Saídas Saídas Digitais
//#define Bz    9 // Buzzer    // o buzzer apita o tempo todo por interferencia de hardware
#define RI    10 // Retroiluminação
#define LED1  11 // LED 1
#define LED2  12 // LED 2
#define led   13 // LED do arduino nano

// Definições dos relés de acionamento
#define R1    25
#define R2    29
#define SSR   33

// Inicializalções
LiquidCrystal_PCF8574 lcd(addr_lcd);
RotaryEncoder encoder(ENA, ENB); 
OneWire oneWire(DS18B20PIN);
DallasTemperature DS18B20(&oneWire);
DHT dht(DHTPIN, DHTTYPE);

// Variáveis para o encoder
int atual       = 1;
static int pos  = 1;
int newPos      = 0;

//limpa lcd perido
int clclcd = 0;

// O ° no LCD para a temperatura
int grau[8]      = {0b01100, 0b10010, 0b10010, 0b01100,
                    0b00000, 0b00000, 0b00000, 0b00000
                   };

// variaveis de controle
int operacao = 0; // 0 esta no preset, 1 esta em operacao
int opcao = 1; // variavel de controle de opcoes no set()
float dht_h;
float dht_t;
float ds18_t;
double Setpoint = 49, Input, Output; //Setpoint 50°C
float erro = 0.0; // diferenca entre setopoint e pv
float pv = 0.0; //valor atual...
bool aquecendo = true; // Variavel para definir o estagio de secagem
int minUmidade = 24;

//Kp = 118, Ki = 0.602, Kd = 5.53e+03
float Kp = 555;
float Ki = 15.5;
float Kd = 0.85;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

//variaveis do temporizador
int t_secagem = 10;// segundos
float t_controle = millis();

int k = 0; //iteracao
float saida = 1;

bool ledDeTeste = true;

void setup() {
  Serial.begin(9600);

  windowStartTime = millis();

  myPID.SetOutputLimits(0, WindowSize);
  
  myPID.SetMode(AUTOMATIC);
  Input = ds18_t;
  myPID.Compute();

  // Inicialização dos pinos de entrada digitais
  pinMode(S1, INPUT);
  //pinMode(S2, INPUT); //error
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(ENA, INPUT);
  pinMode(ENB, INPUT);
  // Inicialização dos pinos de saída digitais
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(SSR, OUTPUT);
  pinMode(RI, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  // pinMode(Bz, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(LED2, HIGH);
  delay(300);
  digitalWrite(LED2, LOW);  // led2 vermelho
  digitalWrite(LED1, HIGH); // led1 amarelo
  analogWrite(RI, 50);
  //Inicia os sersores
  dht.begin();
  DS18B20.begin();


  // Inicialização I2C (Display)   //** tomar cuidado o ds18b20 tbm usa o wire
  Wire.begin();
  Wire.beginTransmission(addr_lcd);
  lcd.begin(size_ldc_c, size_ldc_l); // initialize the lcd
  //Informacoes iniciais
  lcd.createChar(0, grau); // crio o °
  lcd.setBacklight (HIGH);
  lcd.setCursor(0, 0);
  lcd.print("                    ");
  lcd.setCursor(0, 0);
  lcd.print(" SECADORA DE PRATOS ");
  lcd.setCursor(0, 2);
  lcd.print("     CARREGANDO     ");
  delay(200);

}//-------------------------------------------------------------------fim setup

void loop() {
  /*
    curr_time = now() - init_time;
    // Leitura do DHT22
    dht_h = dht.readHumidity();
    dht_t = dht.readTemperature();
    // Leitura do DS18B20
    DS18B20.requestTemperatures();
    tempdallas = DS18B20.getTempCByIndex(0);
  */
  //lcd.setCursor(8, 0);  // sequencia para escrever no lcd: ajusta o cursor
  //lcd.print("LOOP"); // escreve o nome


  while (!operacao) {
    set();
  }

  //condicionar a volta no set se apertar um botao e
  //se o controle não estiver em operacao

  while (operacao) { // aqui o bicho pega
    controle();
  }

  if (digitalRead(S1) == HIGH) {
    while (digitalRead(S1) == HIGH) {}
    delay(10); // evita contar muito rapido
    operacao = false; //entra no set() e sai do controle
  }

}//-------------------------------------------------------------------fim loop

void limpalcd() {
  // evita lixo no lcd
  lcd.setCursor(0, 0);  // sequencia para escrever no lcd: ajusta o cursor
  lcd.print("                    "); // escreve o nome
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  //Serial.println("limpalcd"); //debug
}

void leencoder() {

  //Le as informacoes do encoder
  encoder.tick();
  newPos = encoder.getPosition();
  //Se a posicao foi alterada, mostra o valor
  //no Serial Monitor
  if (pos != newPos) {
    //Serial.print(newPos);
    //Serial.println();
    atual = pos;
    pos = newPos;

  }
  //Serial.println("lendo o encoder"); //debug
}// -------------------------------------------------------------void encoder


void set() { // definicoes iniciais

  // definir Setpoint
  // talves o tempo de secagem com a umidade baixa
  //controle de opcoes entre Setpoint tempo de secagem e se tiver outras
  // sempre que entrar no set() reinicia a opcao
  k = 0;
  digitalWrite(SSR, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);

  limpalcd();
  lcd.setCursor(8, 0);  // sequencia para escrever no lcd: ajusta o cursor
  lcd.print("SET"); // escreve o nome

  while (!operacao) {

    if (digitalRead(S1) == HIGH) {
      while (digitalRead(S1) == HIGH) {}
      delay(10); // evitar contar muito rapido
      //Serial.println("S1 true"); //debug
      limpalcd();
      operacao = true; // sai do set() e vai para o controle
      break; //
    }

    if (digitalRead(S5) == HIGH) {
      while (digitalRead(S5) == HIGH) {}
      delay(10); // evitar contar muito rapido
      //Serial.println("S5 true"); //debug
      opcao ++;// troca de opcao
    }

    if (opcao >= 4) {
      opcao = 1;// se a opcao for maior que a quantidade de opcoes, volta pra primeira
    }

    while (opcao == 1) {
      //Le as informacoes do encoder
      leencoder();

      if (atual > pos) Setpoint ++;      // aumenta ou diminui o set point
      else if (atual < pos) Setpoint --;
      atual = pos;                       // sem essa condicao ele fica dentro dos if's sempre incrementando
      lcd.setCursor(0, 2);
      lcd.print("Setpoint :");

      clclcd ++;  // contador do limpador periodico do lcd limpeza por ciclo
      if (clclcd >= 20) {     // condicao para evitar caracter lixo no lcd
        lcd.setCursor(0, 2);
        lcd.print("                  ");
        lcd.setCursor(0, 3);
        lcd.print("                  ");
        clclcd = 0;
      }

      lcd.setCursor(11, 2);
      lcd.print(Setpoint);
      String uhuu = String(Setpoint);  // coloca o °C depois do valor
      int uhu = uhuu.length();         // da quantidade de casas decimais
      lcd.setCursor(11 + uhu, 2);
      lcd.write((uint8_t)0);
      lcd.print("C");

      if (digitalRead(S5) == HIGH) {
        while (digitalRead(S5) == HIGH) {}
        delay(10); // evitar contar muito rapido
        //Serial.println("S5 true"); //debug
        opcao ++;// troca de opcao
        atual = 0;

      }
      if (digitalRead(S1) == HIGH) {
        while (digitalRead(S1) == HIGH) {}
        delay(10); 
        Serial.println("S1 true");
        operacao = true; // sai do set() e vai para o controle
        limpalcd();
        break; // forca sair do while de set
      }
    }//-----------------------------------------------------------------fim opcao1

    while (opcao == 2) {
      //Le as informacoes do encoder
      leencoder();


      if (atual > pos) t_secagem ++;
      else if (atual < pos) t_secagem --;
      atual = pos;                       // sem essa condicao ele fica dentro dos if's sempre incrementando
      lcd.setCursor(0, 2);
      lcd.print(" TEMPO DE SECAGEM: ");

      clclcd ++;  // contador do limpador periodico do lcd
      if (clclcd >= 20) {     // condicao para evitar caracter lixo no lcd
        lcd.setCursor(0, 2);
        lcd.print("                  ");
        lcd.setCursor(0, 3);
        lcd.print("                  ");
        clclcd = 0;
      }
      lcd.setCursor(8, 3);
      lcd.print(t_secagem);
      String uhuu = String(t_secagem);
      int uhu = uhuu.length();
      lcd.setCursor(8 + uhu, 3);
      lcd.print("s");

      if (digitalRead(S5) == HIGH) {
        while (digitalRead(S5) == HIGH) {}
        delay(10);                        // evitar contar muito rapido
        Serial.println("S5 true");
        opcao ++;                         // troca de opcao
        atual = 0;                        // volta o valor de atual entre as opcoes
      }
      if (digitalRead(S1) == HIGH) {
        while (digitalRead(S1) == HIGH) {}
        delay(10); // evitar contar muito rapido
        Serial.println("S1 true");
        operacao = true; // sai do set() e vai para o controle
        limpalcd();
        break;

      }
    }//------------------------------------------------------------fim opcao2
  
  while (opcao == 3) {
      //Le as informacoes do encoder
      leencoder();


      if (atual > pos) minUmidade ++;
      else if (atual < pos) minUmidade --;
      atual = pos;                       // sem essa condicao ele fica dentro dos if's sempre incrementando
      lcd.setCursor(0, 2);
      lcd.print("   UMIDADE MINIMA ");

      clclcd ++;  // contador do limpador periodico do lcd
      if (clclcd >= 20) {     // condicao para evitar caracter lixo no lcd
        lcd.setCursor(0, 2);
        lcd.print("                  ");
        lcd.setCursor(0, 3);
        lcd.print("                  ");
        clclcd = 0;
      }
      lcd.setCursor(8, 3);
      lcd.print(minUmidade);
      String uhuu = String(minUmidade);
      int uhu = uhuu.length();
      lcd.setCursor(8 + uhu, 3);
      lcd.print("%");

      if (digitalRead(S5) == HIGH) {
        while (digitalRead(S5) == HIGH) {}
        delay(10);      
        Serial.println("S5 true");
        opcao ++;                         // troca de opcao
        atual = 0;                        // volta o valor de atual entre as opcoes
      }
      if (digitalRead(S1) == HIGH) {
        while (digitalRead(S1) == HIGH) {}
        delay(10); // evitar contar muito rapido
        Serial.println("S1 true");
        operacao = true; // sai do set() e vai para o controle
        limpalcd();
        break;

      }
    }//------------------------------------------------------------fim opcao3
  
  }

}//-------------------------------------------------------------------fim set


void controle() {

  time_t init_time = now();
  time_t umid_time = 0;
  time_t curr_time = now();

  float t_print = millis();
  float t_controle = millis();
  int aux = 0;
  aquecendo = true;

  limpalcd();
  lcd.setCursor(0, 0);
  lcd.print("CONTROLE: ");
  lcd.setCursor(0, 1);
  lcd.print("TEMPERATURA: ");
  lcd.setCursor(0, 2);
  lcd.print("UMIDADE: ");
  lcd.setCursor(0, 3);
  lcd.print("TEMPO: ");

  while (operacao) {
    curr_time = now();
    lcd.setCursor(7, 3);
    lcd.print(curr_time - init_time);
    String uhuu3 = String(curr_time - init_time);
    int uhu3 = uhuu3.length();
    lcd.setCursor(7 + uhu3, 3);
    lcd.print("s");

    if (digitalRead(S5) == HIGH) {  // sai do while
      while (digitalRead(S5) == HIGH) {}
      delay(10); // evitar contar muito rapido
      //Serial.println("S1 true"); //debug

      digitalWrite(SSR, LOW);
      digitalWrite(R1, LOW);
      digitalWrite(R2, LOW);
      digitalWrite(RI, HIGH);
      delay(500);
      digitalWrite(RI, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      limpalcd();
      operacao = false;
      break; //
    }
    // Leitura do DHT22
    float dht_hb = dht_h;
    float dht_h = dht.readHumidity();
    // float dht_t = dht.readTemperature();

    // Leitura do DS18B20
    DS18B20.requestTemperatures();
    float ds18_tb = ds18_t; // salva o valor anterior do ds18b20
    // se o sensor der erro na leitura uma condicao mantem o valor anteior
    float ds18_t = DS18B20.getTempCByIndex(0);
    if(millis()-t_print > 1000){
      Serial.print(millis()/1000);
      Serial.print(",");
      Serial.println(ds18_t);
      t_print=millis();
    }
    if (ds18_t < 0 ) ds18_t = ds18_tb; // condicao para evitar erros unicos de leitura dos sensores
    if (dht_h < 0 ) dht_h = dht_hb;

    clclcd ++;
    if (clclcd >= 20) {                // condicao para evitar caracter lixo no lcd
      lcd.setCursor(9, 2);
      lcd.print("    ");
      lcd.setCursor(16, 1);
      lcd.print("   ");
      lcd.setCursor(13, 1);
      lcd.print("       ");
      clclcd = 0;
    }
    String uhuu1 = String(ds18_t);
    int uhu1 = uhuu1.length();
    lcd.setCursor(13 + uhu1, 1);
    lcd.write((uint8_t)0);
    lcd.setCursor(14 + uhu1, 1);
    lcd.print("C");
    lcd.setCursor(13, 1);  // printa T1
    lcd.print(ds18_t);

    String uhuu2 = String(dht_h);
    int uhu2 = uhuu2.length();
    lcd.setCursor(9 + uhu2, 2);
    lcd.print("%");
    lcd.setCursor(9, 2);  // printa UMIDADE
    lcd.print(dht_h);


    // tentativa de fazer com que o forno fique aquecendo por um tempo apos a umidade fique baixa, para gaarantir secagem
    if (int(dht_h) <= minUmidade && aux == 0) {  // guarda o tempo da primeira vez que a uidade baixa de 20%  && aux == 0
      umid_time = now();
      analogWrite(RI, 150);
      aux = 1;
    }

    if ((int(curr_time) - int(umid_time)) > int(t_secagem) && int(dht_h) <= minUmidade) { // quando passa x segundos de umidade baixa
      aquecendo = false;
    }

    if (aquecendo) {
      //aquecimento
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      lcd.setCursor(10, 0);
      lcd.print("AQUECENDO");

      pwm();
    }
    else {
      //resfriamento
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      analogWrite(RI, 100);
      lcd.setCursor(8, 0);
      lcd.print("             ");
      lcd.setCursor(10, 0);
      lcd.print("RESFRIANDO");
      digitalWrite(SSR, LOW);

      if (ds18_t < 35) { // temperatura segura para abrir a porta do forno
        operacao = false;
      }
    }

  }
}

void pwm() { // aqui vai o controle
  // digitalWrite(SSR, HIGH);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, HIGH);/*
  Serial.print("saida: ");
  Serial.println(Output);
  Serial.print("millis: ");
  Serial.println(millis());
  Serial.print("t_controle: ");
  Serial.println(t_controle);
*/

  Input = DS18B20.getTempCByIndex(0);
  myPID.Compute();
/*  Serial.print("Output: ");
  Serial.println(Output);
  Serial.print("Input: ");
  Serial.println(Input);
  Serial.print("Setpoint");
  Serial.println(Setpoint);
*/
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > now - windowStartTime) digitalWrite(SSR, HIGH);
  else digitalWrite(SSR, LOW);
  
  lcd.setCursor(13, 3);
  lcd.print("D");
  lcd.setCursor(15, 3);
  
}
