#include <Arduino.h>

#define PULSOS_POR_VOLTA 480.0
#define TCONTROLE 20

float velocidadeAlvo = 10.0;

enum Movimento {
  PARADO,
  FRENTE,
  CURVA_DIREITA,
  CURVA_ESQUERDA
};

Movimento movimentoAtual = FRENTE;

struct MotorPI {
  int rpwm;
  int lpwm;

  int encA;
  int encB;

  volatile long* pulsos;

  long posAnterior;

  float integral;
  float kp;
  float ki;

  int invertido;
};

// Encoders
volatile long pulsosRFE = 0;
volatile long pulsosRFD = 0;
volatile long pulsosRTE = 0;
volatile long pulsosRTD = 0;

// Roda frontal esquerda
MotorPI RTE = {13, 25, 14, 16, &pulsosRFE, 0, 0, 1.5, 3.0, 1};

// Roda frontal direita
MotorPI RFD = {26, 27, 17, 18, &pulsosRFD, 0, 0, 1.5, 3.0, 1};

// Roda traseira esquerda
MotorPI RFE = {32, 33, 34, 35, &pulsosRTE, 0, 0, 1.5, 3.0, 1};

// Roda traseira direita
MotorPI RTD = {12, 4, 22, 23, &pulsosRTD, 0, 0, 1.5, 3.0, 1};


void setMotor(MotorPI& m, int pwm) {
  pwm = pwm * m.invertido;
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(m.rpwm, pwm);
    analogWrite(m.lpwm, 0);
  } else if (pwm < 0) {
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, -pwm);
  } else {
    analogWrite(m.rpwm, 0);
    analogWrite(m.lpwm, 0);
  }
}

float atualizarPI(MotorPI& m, float alvoRPM, float deltaT) {
  long posAtual;

  noInterrupts();
  posAtual = *(m.pulsos);
  interrupts();

  long deltaPulsos = posAtual - m.posAnterior;
  m.posAnterior = posAtual;

  float rpm = (deltaPulsos / PULSOS_POR_VOLTA) * (60.0 / deltaT);

  float erro = alvoRPM - rpm;

  m.integral += erro * deltaT;
  m.integral = constrain(m.integral, -100.0, 100.0);

  float controle = m.kp * erro + m.ki * m.integral;

  int pwm = constrain((int)controle, -255, 255);

  setMotor(m, pwm);

  return rpm;
}

void mover(float rpmEsquerda, float rpmDireita, float deltaT) {
  atualizarPI(RFE, rpmEsquerda, deltaT);
  atualizarPI(RTE, rpmEsquerda, deltaT);

  atualizarPI(RFD, rpmDireita, deltaT);
  atualizarPI(RTD, rpmDireita, deltaT);
}

void Frente(float rpm, float deltaT) {
  mover(rpm, rpm, deltaT);
}

void CurvaDireita(float rpm, float deltaT) {
  mover(rpm, rpm * 0.5, deltaT);
}

void CurvaEsquerda(float rpm, float deltaT) {
  mover(rpm * 0.5, rpm, deltaT);
}

void Parar() {
  setMotor(RFE, 0);
  setMotor(RFD, 0);
  setMotor(RTE, 0);
  setMotor(RTD, 0);
}

void zerarIntegrais() {
  RFE.integral = 0;
  RFD.integral = 0;
  RTE.integral = 0;
  RTD.integral = 0;
}

void executarMovimento(float deltaT) {
  if (movimentoAtual == FRENTE) {
    Frente(velocidadeAlvo, deltaT);
  } 
  else if (movimentoAtual == CURVA_DIREITA) {
    CurvaDireita(velocidadeAlvo, deltaT);
  } 
  else if (movimentoAtual == CURVA_ESQUERDA) {
    CurvaEsquerda(velocidadeAlvo, deltaT);
  } 
  else {
    Parar();
  }
}

void IRAM_ATTR isrRFE() {
  if (digitalRead(RFE.encB) == HIGH) pulsosRFE++;
  else pulsosRFE--;
}

void IRAM_ATTR isrRFD() {
  if (digitalRead(RFD.encB) == HIGH) pulsosRFD++;
  else pulsosRFD--;
}

void IRAM_ATTR isrRTE() {
  if (digitalRead(RTE.encB) == HIGH) pulsosRTE++;
  else pulsosRTE--;
}

void IRAM_ATTR isrRTD() {
  if (digitalRead(RTD.encB) == HIGH) pulsosRTD++;
  else pulsosRTD--;
}

void configurarMotor(MotorPI& m) {
  pinMode(m.rpwm, OUTPUT);
  pinMode(m.lpwm, OUTPUT);

  pinMode(m.encA, INPUT_PULLUP);
  pinMode(m.encB, INPUT_PULLUP);

  setMotor(m, 0);
}

unsigned long ultimoControle = 0;
unsigned long tempoInicio = 0;

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  pinMode(26, OUTPUT);

  configurarMotor(RFE);
  configurarMotor(RFD);
  configurarMotor(RTE);
  configurarMotor(RTD);

  attachInterrupt(digitalPinToInterrupt(RFE.encA), isrRFE, RISING);
  attachInterrupt(digitalPinToInterrupt(RFD.encA), isrRFD, RISING);
  attachInterrupt(digitalPinToInterrupt(RTE.encA), isrRTE, RISING);
  attachInterrupt(digitalPinToInterrupt(RTD.encA), isrRTD, RISING); 

  ultimoControle = millis();
  tempoInicio = millis();
}
void loop() {
  unsigned long agora = millis();

  if (agora - ultimoControle >= TCONTROLE) {
    float deltaT = (agora - ultimoControle) / 1000.0;

    Frente(velocidadeAlvo, deltaT);

    ultimoControle = agora;
    }

  /* Para testar o funcionamento das rodas:
  setMotor(RFE, 50);
  setMotor(RFD, 50);
  setMotor(RTE, 50);
  setMotor(RTD, 50); */



  delay(1000); 

  
}