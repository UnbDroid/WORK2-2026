#include <Arduino.h>

#define PULSOS_POR_VOLTA 480.0
#define TCONTROLE 20
#define R 20
#define H 19.5
#define RAIO 0.0375


double velocidadeAlvo = -40;

unsigned long agora = 0;

double rpmRFE;
double rpmRFD;
double rpmRTE;
double rpmRTD;

struct MotorPI {
  int rpwm;
  int lpwm;

  int encA;
  int encB;

  volatile long* pulsos;

  long posAnterior;

  double integral;
  double kp;
  double ki;

  int invertidoMotor;
  int invertidoEncoder;
};

volatile long pulsosRFE = 0;
volatile long pulsosRFD = 0;
volatile long pulsosRTE = 0;
volatile long pulsosRTD = 0;

MotorPI RTE = {13, 25, 14, 16, &pulsosRTE, 0, 0, 1.5, 3.0, -1, 1};
MotorPI RFD = {26, 27, 36, 39, &pulsosRFD, 0, 0, 1.5, 3.0, 1, 1};
MotorPI RFE = {32, 33, 22, 23, &pulsosRFE, 0, 0, 1.5, 3.0, -1, 1};
MotorPI RTD = {15, 4, 34, 35, &pulsosRTD, 0, 0, 1.5, 3.0, 1, 1};

void setMotor(MotorPI& m, int pwm) {
  pwm = pwm * m.invertidoMotor;
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

double atualizarPI(MotorPI& m, double alvoRPM, double deltaT) {
  long posAtual;

  noInterrupts();
  posAtual = *(m.pulsos);
  interrupts();

  long deltaPulsos = posAtual - m.posAnterior;
  m.posAnterior = posAtual;

  double rpm = (deltaPulsos / PULSOS_POR_VOLTA) * (60.0 / deltaT);
  

  double erro = alvoRPM - rpm;

  m.integral += erro * deltaT;
  m.integral = constrain(m.integral, -100.0, 100.0);

  double controle = m.kp * erro + m.ki * m.integral;

  int pwm = constrain((int)controle, -255, 255);

  setMotor(m, pwm);

  return rpm;
}

void mover(double rpmEsquerda, double rpmDireita, double deltaT) {
  rpmRFE = atualizarPI(RFE, rpmEsquerda, deltaT);
  rpmRTE = atualizarPI(RTE, rpmEsquerda, deltaT);

  rpmRFD = atualizarPI(RFD, rpmDireita, deltaT);
  rpmRTD = atualizarPI(RTD, rpmDireita, deltaT);
}

void moverLateral(double dig1, double dig2, double deltaT) {
  rpmRFE = atualizarPI(RFE, dig1, deltaT);
  rpmRTE = atualizarPI(RTE, dig2, deltaT);

  rpmRFD = atualizarPI(RFD, dig2, deltaT);
  rpmRTD = atualizarPI(RTD, dig1, deltaT);
}

void movimentar (double vx, double vy, double w, double deltaT, double inverter) {
  double velRFE = (vx + vy + R * w - H * w);
  double velRTD = (vx + vy - R * w + H * w);
  double velRFD = (vx - vy - R * w + H * w);
  double velRTE = (vx - vy + R * w - H * w);

  rpmRFE = atualizarPI(RFE, velRFE, deltaT);  
  rpmRTE = atualizarPI(RTE, velRTE, deltaT);
  rpmRFD = atualizarPI(RFD, velRFD, deltaT);
  rpmRTD = atualizarPI(RTD, velRTD, deltaT);
}

void Frente(double rpm, double deltaT) {
  mover(rpm, rpm, deltaT);
}

void Tras(double rpm, double deltaT) {
  mover(-rpm, -rpm, deltaT);
}

void CurvaDireita(double rpm, double deltaT) {
  mover(rpm, rpm * 0.5, deltaT);
}

void CurvaEsquerda(double rpm, double deltaT) {
  mover(rpm * 0.5, rpm, deltaT);
}

void Esquerda (double rpm, double deltaT) {
  moverLateral(rpm, -rpm, deltaT);
}
void Direita (double rpm, double deltaT) {
  moverLateral(-rpm, rpm, deltaT);
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
  agora = millis(); 
}

void loop() {
  agora = millis();

  if (agora - ultimoControle >= TCONTROLE) {
    double deltaT = (agora - ultimoControle) / 1000.0;

    movimentar(-velocidadeAlvo, 0, 0, deltaT, 1);

    ultimoControle = agora;
  } 

  /*Serial.print("RPM RFE: ");
  Serial.print(rpmRFE);
  Serial.print(" / ");
  Serial.print("RPM RFD: ");
  Serial.print(rpmRFD);
  Serial.print(" / ");
  Serial.print("RPM RTE: ");
  Serial.print(rpmRTE);
  Serial.print(" / ");
  Serial.print("RPM RTD: ");
  Serial.println(rpmRTD);*/

  delay(10); 
}