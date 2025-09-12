#include <SparkFun_TB6612.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>
#include <ezBuzzer.h>  // ezBuzzer library

//Declarando variveis dos motores
#define motorLeftSpeed 5
#define motorLeftA 8
#define motorLeftB 7
#define stby 9
#define motorRightA 10
#define motorRightB 11
#define motorRightSpeed 6

const int offsetA = 1;
const int offsetB = -1;

//Estanciando biblioteca dos motores
Motor motorLeft = Motor(motorLeftA, motorLeftB, motorLeftSpeed, offsetA, stby);
Motor motorRight = Motor(motorRightA, motorRightB, motorRightSpeed, offsetB, stby);

//Variaveis Auxiliares Velocidade
#define speedMax 100
int speedMin = -20;
int speedBase = 30;

//Declarando variaveis do Array
QTRSensors arraySensors;
const uint8_t sensorCount = 6;
uint16_t valuesSensors[sensorCount];

#define emitterPin 12

//VARIAVEIS PID
float Kp = 1.45,
      Ki = 0.0,
      Kd = 15.0;

uint16_t position = 0;

int error = 0;
int setPoint = 50;

int P = 0;
int I = 0;
int D = 0;

int PIDValue = 0;

int lastError = 0;
int countI = 0;

/*SENSORES LATERAIS*/
#define sensorSideLeft A7
#define sensorSideRight A6

bool vSSLeft = 0,
     vSSRight = 0,
     last_vSSLeft = 0,
     last_vSSRight = 0;

// interseccao
bool inter = false;

// contadores
int inter_count = 0;
int vSSLeft_count = 0;
int vSSRight_count = 0;

int media = 600;
int final = 0;
int BoostOn = 0;

/*BLUETOOTH*/
const int pinoRX = 0;
const int pinoTX = 1;
int dadoBluetooth = 0;

SoftwareSerial bluetooth(pinoRX, pinoTX);


/*LEDs*/
#define led 4

/*BOTÔES*/
const int btnLeft = 2;
const int btnRight = 13;

// buzzer
#define BUZZER_PIN 3
ezBuzzer buzzer(BUZZER_PIN);

void teste() {
  buzzer.beep(100);
}

/*CONFIG INICIAL DOS SENSORES*/
void initRobot() {
  //Inicializando Array
  arraySensors.setTypeAnalog();
  arraySensors.setSensorPins((const uint8_t[]){ A5, A4, A3, A2, A1, A0 }, sensorCount);
  arraySensors.setEmitterPin(emitterPin);

  //Led
  onLed(500);
  onLed(500);
}

/*FUNÇÃO PARA PISCAR LED E APITAR BUZZER NO COMEÇO DA CORRIDA*/
void startRun() {
  delay(100);
  onLed();
  delay(100);
  offLed();
  delay(100);
  onLed();
  delay(100);
  offLed();
}

/*FUNÇÃO PARA PISCAR LED E APITAR BUZZER NO FINAL DA CORRIDA*/
void finishRun() {
  motorLeft.brake();
  motorRight.brake();
  delay(10);
  onLed();
  delay(500);
  offLed();
}

/*FUNÇÕES DO LED*/
void onLed(int time) {
  digitalWrite(led, HIGH);
  delay(time);
  digitalWrite(led, LOW);
}
void onLed() {
  digitalWrite(led, HIGH);
}
void offLed() {
  digitalWrite(led, LOW);
}


/*FUNÇÕES PARA CHECAR SE O BOTÃO FOI PRECIONADO*/
bool btnPressed(int btn) {
  if (digitalRead(btn) == HIGH) {
    delay(200);
    teste();
    return true;
  } else
    return false;
}

void calibrateArray() {
  Serial.println("Calibrando Sensores Array...");

  delay(1000);

  onLed();

  for (uint16_t i = 0; i < 200; i++) {
    arraySensors.calibrate();
  }

  offLed();

  Serial.println("Sensores Calibrados!");

  delay(500);
}


void followLine() {
  //Parada pelo Sensor Lateral
  detectMarker();

  // Cálculo do PID.
  position = arraySensors.readLineWhite(valuesSensors);

  position = map(position, 0, 5000, 0, 100);

  error = position - setPoint;

  P = error;
  D = (error - lastError);

  if (D >= 50) D = 50;
  if (D <= -50) D = -50;

  PIDValue = (P * Kp) + (D * Kd);

  //if (PIDValue >=  50) PIDValue =  50;
  //if (PIDValue <= -50) PIDValue = -50;

  // Atualiza os valores do erro Anterior.
  lastError = error;

  // Atribuição da velocidade dos motores.
  int speedRight = (speedBase - PIDValue);
  int speedLeft = -(speedBase + PIDValue);

  // Checa se as velocidades ultrapassam 255 ou são mensores que 0, corrige caso necessario
  if (speedRight > speedMax) speedRight = speedMax;
  if (speedLeft > speedMax) speedLeft = speedMax;

  if (speedRight < speedMin) speedRight = speedMin;
  if (speedLeft < speedMin) speedLeft = speedMin;

  int percent = 255 * speedMin / 100;

  speedRight = map(speedRight, speedMin, 100, percent, 255);
  speedLeft = map(speedLeft, speedMin, 100, percent, 255);


  //Atribui as velocidades aos motores
  motorLeft.drive(speedLeft);
  motorRight.drive(speedRight);
}

void readSideSensors() {
  last_vSSLeft = vSSLeft;
  last_vSSRight = vSSRight;

  int x, y;

  x = analogRead(sensorSideLeft);
  y = analogRead(sensorSideRight);


  if (x > media) {
    //Serial.println("E dtc");
    vSSLeft = 1;
  } else vSSLeft = 0;

  if (y > media) {
    //Serial.println("D dtc");
    vSSRight = 1;
  } else vSSRight = 0;
}

void detectMarker() {
  readSideSensors();

  // checa interseccao
  if (!inter && vSSLeft && vSSRight) {
    inter = true;
  }

  if (last_vSSLeft == false && vSSLeft == true) {
    Serial.println("esq ent");
  } else if (last_vSSLeft == true && vSSLeft == false) {
    if (!vSSRight) {
      if (inter) {
        intersection();
      } else {
        vSSLeft_count++;
        Serial.println("vSSLeft: " + String(vSSLeft_count));
      }
    }
  }

  if (last_vSSRight == false && vSSRight == true) {
    Serial.println("dir ent");
  } else if (last_vSSRight == true && vSSRight == false) {
    if (!vSSLeft) {
      if (inter) {
        intersection();
      } else {
        markerRight();
        Serial.println("final:" + String(final));
      }
    }
  }
}


void intersection() {
  inter_count++;
  inter = false;
  Serial.println("inter: " + String(inter_count));
}

void markerLeft() {
  onLed();
}

void markerRight() {
  onLed();

  final++;

  if (final >= 2) {
    offLed();
    motorLeft.brake();
    motorRight.brake();
    delay(5000);
    teste();
    final = 0;
  }
}

String recebeDado() {
  switch (dadoBluetooth) {
    case '0':
      return "0";
      break;
    case '1':
      return "1";
      break;
    case '2':
      return "2";
      break;
    case '3':
      return "3";
      break;
    case '4':
      return "4";
      break;
    case '5':
      return "5";
      break;
    case '6':
      return "6";
      break;
    case '7':
      return "7";
      break;
    case '8':
      return "8";
      break;
    case '9':
      return "9";
      break;
    default:
      break;
  }
}

String fazLeitura() {
  do {

  } while (bluetooth.available() == 0);

  dadoBluetooth = bluetooth.read();

  return recebeDado();
}

void set_Kp() {
  String num1;
  float num2;

  //Obter as entradas
  Serial.println("Insira o primeiro digito");
  String w = String(fazLeitura());
  Serial.println(w);
  Serial.println("Insira o segundo digito");
  String x = String(fazLeitura());
  Serial.println(x);
  Serial.println("Insira o terceiro digito");
  String y = String(fazLeitura());
  Serial.println(y);
  Serial.println("Insira o quarto digito");
  String z = String(fazLeitura());
  Serial.println(z);

  num1 = w + x + "." + y + z;

  num2 = num1.toFloat();

  Kp = num2;

  Serial.println(Kp);
}

void set_Kd() {
  String num1;
  float num2;

  //Obter as entradas
  Serial.println("Insira o primeiro digito");
  String w = String(fazLeitura());
  Serial.println(w);
  Serial.println("Insira o segundo digito");
  String x = String(fazLeitura());
  Serial.println(x);
  Serial.println("Insira o terceiro digito");
  String y = String(fazLeitura());
  Serial.println(y);
  Serial.println("Insira o quarto digito");
  String z = String(fazLeitura());
  Serial.println(z);

  num1 = w + x + "." + y + z;

  num2 = num1.toFloat();

  Kd = num2;

  Serial.println(Kd);
}

void debugModoSensor() {
  for (int i = 0; i < 50; i++) {
    position = arraySensors.readLineWhite(valuesSensors);
    position = map(position, 0, 5000, 0, 100);
    Serial.print("Posicao: ");
    Serial.println(position);
    delay(100);
  }
}

void debugModoLateral() {
  Serial.println("Testando esquerdo...");
  delay(1000);

  for (int i = 0; i < 10; i++) {
    Serial.print("Esquerdo: ");
    Serial.println(analogRead(sensorSideLeft));
    delay(500);
  }

  Serial.println("Testando direito...");
  onLed(500);
  delay(1000);

  for (int i = 0; i < 10; i++) {
    Serial.print("Direito: ");
    Serial.println(analogRead(sensorSideRight));
    delay(500);
  }
}

void debugModoMotor() {
  int velocidade = 0;

  while (true) {
    if (bluetooth.available() > 0) {
      dadoBluetooth = bluetooth.read();

      if (dadoBluetooth == 'V') {
        if (velocidade >= 192) {
          velocidade = 255;
          Serial.println("Velocidade maxima atingida");
        } else {
          velocidade += 63;
        }
        Serial.print("Velocidade: ");
        Serial.println(velocidade);
        delay(1000);
        motorLeft.drive(velocidade);
        motorRight.drive(velocidade);
      } else if (dadoBluetooth == 'v') {
        if (velocidade <= -192) {
          velocidade = -255;
          Serial.println("Velocidade minima atingida");
        } else {
          velocidade -= 63;
        }
        Serial.print("Velocidade: ");
        Serial.println(velocidade);
        delay(1000);
        motorLeft.drive(velocidade);
        motorRight.drive(velocidade);
      } else if (dadoBluetooth == 'f') {
        Serial.println("freiando");
        motorLeft.drive(0);
        motorRight.drive(0);
      } else if (dadoBluetooth == 'x') {
        motorLeft.drive(0);
        motorRight.drive(0);
        Serial.println("voltando menu");
        break;
      }
    }
  }
}

void debug() {
  Serial.println("Modo debug ativado");
  Serial.println("s: teste de sensores");
  Serial.println("l: teste de sensores laterais");
  Serial.println("m: teste de motores");
  Serial.println("X: sair");

  char modo = 'n';
  int velocidade = 0;

  while (true) {
    if (bluetooth.available() > 0) {
      dadoBluetooth = bluetooth.read();

      if (dadoBluetooth == 's') {
        modo = 's';
      } else if (dadoBluetooth == 'l') {
        modo = 'l';
      } else if (dadoBluetooth == 'm') {
        modo = 'm';
      } else if (dadoBluetooth == 'X') {
        Serial.println("Modo debug desativado");
        break;
      }

      if (modo == 's') {
        debugModoSensor();
        modo = 'n';
        Serial.println("Modo debug ativado");
        Serial.println("s: teste de sensores");
        Serial.println("l: teste de sensores laterais");
        Serial.println("m: teste de motores");
        Serial.println("X: sair");
      } else if (modo == 'l') {
        debugModoLateral();
        modo = 'n';
        Serial.println("Modo debug ativado");
        Serial.println("s: teste de sensores");
        Serial.println("l: teste de sensores laterais");
        Serial.println("m: teste de motores");
        Serial.println("X: sair");
      } else if (modo == 'm') {
        Serial.println("- Menu modo motor -");
        Serial.println("V: +25%");
        Serial.println("v: -25%");
        Serial.println("f: freiar motores");
        Serial.println("x: voltar ao menu");
        debugModoMotor();
        modo = 'n';
        Serial.println("Modo debug ativado");
        Serial.println("s: teste de sensores");
        Serial.println("l: teste de sensores laterais");
        Serial.println("m: teste de motores");
        Serial.println("X: sair");
      } else if (modo == 'x') {
        Serial.println("Modo debug desativado");
        break;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  //Setas os pinos como entrada ou saida
  pinMode(motorLeftSpeed, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftSpeed, OUTPUT);

  pinMode(led, OUTPUT);
  pinMode(btnLeft, INPUT);
  pinMode(btnRight, INPUT);

  initRobot();

  bluetooth.begin(9600);
  bluetooth.print("$");
  bluetooth.print("$");
  bluetooth.print("$");
}

void loop() {
  buzzer.loop();
  if (btnPressed(btnLeft)) {
    startRun();
    while (!btnPressed(btnLeft) && bluetooth.available() == 0) {
      followLine();
    }
    finishRun();
  }
  if (btnPressed(btnRight)) {
    calibrateArray();
  }

  if (bluetooth.available() > 0) {
    dadoBluetooth = bluetooth.read();

    if (dadoBluetooth == '5') {
      speedBase = 25;
      Kp = 1.45;
      Kd = 15.00;

      delay(20);
      //onBuzzer(50);
      delay(20);
    } else if (dadoBluetooth == 'o') {
      set_Kp();
    } else if (dadoBluetooth == 'u') {
      set_Kd();
    } else if (dadoBluetooth == 'S') {
      speedBase += 5;
    } else if (dadoBluetooth == 's') {
      speedBase -= 5;
    } else if (dadoBluetooth == 'M') {
      speedMin += 5;
    } else if (dadoBluetooth == 'm') {
      speedMin -= 5;
    } else if (dadoBluetooth == 'P') {
      Kp += 0.25;
    } else if (dadoBluetooth == 'p') {
      Kp -= 0.25;
    } else if (dadoBluetooth == 'D') {
      Kd += 0.25;
    } else if (dadoBluetooth == 'd') {
      Kd -= 0.25;
    } else if (dadoBluetooth == '1') {
      debug();
    }



    if (dadoBluetooth != 'T' && dadoBluetooth != 'w') {
      Serial.println(Kp);
      Serial.println(Kd);
      Serial.println(speedBase);
    }
  }
}

