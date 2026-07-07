#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <string.h>

#define PULSOS_POR_VOLTA 480.0
#define TCONTROLE 20
#define R 20
#define H 19.5
#define RAIO 0.0375

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t sub_cmd_vel;
rcl_publisher_t pub_debug;

geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__String msg_debug;

double vx_alvo = 0;
double vy_alvo = 0;
double w_alvo = 0;

unsigned long ultimoCmdVel = 0;
unsigned long ultimoControle = 0;
unsigned long ultimoDebug = 0;
unsigned long agora = 0;

double rpmRFE = 0;
double rpmRFD = 0;
double rpmRTE = 0;
double rpmRTD = 0;

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

void publicarDebug(const char *texto) {
  strcpy(msg_debug.data.data, texto);
  msg_debug.data.size = strlen(texto);
  rcl_publish(&pub_debug, &msg_debug, NULL);
}

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

void movimentar(double vx, double vy, double w, double deltaT) {
  double velRFE = (vx + vy + R * w - H * w);
  double velRTD = (vx + vy - R * w + H * w);
  double velRFD = (vx - vy - R * w + H * w);
  double velRTE = (vx - vy + R * w - H * w);

  rpmRFE = atualizarPI(RFE, velRFE, deltaT);
  rpmRTE = atualizarPI(RTE, velRTE, deltaT);
  rpmRFD = atualizarPI(RFD, velRFD, deltaT);
  rpmRTD = atualizarPI(RTD, velRTD, deltaT);
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

void publicarDebugVelocidades() {
  char texto[250];

  sprintf(
    texto,
    " vx=%.2f vy=%.2f w=%.2f |  RFE=%.2f RFD=%.2f RTE=%.2f RTD=%.2f",
    vx_alvo,
    vy_alvo,
    w_alvo,
    rpmRFE,
    rpmRFD,
    rpmRTE,
    rpmRTD
  );

  publicarDebug(texto);
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

void callback_cmd_vel(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  double escalaLinear = 80.0;
  double escalaAngular = 40.0;

  vx_alvo = msg->linear.x * escalaLinear;
  vy_alvo = msg->linear.y * escalaLinear;
  w_alvo = msg->angular.z * escalaAngular;

  ultimoCmdVel = millis();
}

void configurarMicroROS() {
  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(
    &node,
    "esp32_motor_node",
    "",
    &support
  );

  rclc_subscription_init_default(
    &sub_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  rclc_publisher_init_default(
    &pub_debug,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug_velocidades"
  );

  msg_debug.data.data = (char *)malloc(250);
  msg_debug.data.capacity = 250;
  msg_debug.data.size = 0;

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
    &executor,
    &sub_cmd_vel,
    &msg_cmd_vel,
    &callback_cmd_vel,
    ON_NEW_DATA
  );
}

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

  configurarMicroROS();

  ultimoControle = millis();
  ultimoCmdVel = millis();
  ultimoDebug = millis();
}

void loop() {
  // setMotor(RFE, 120);
  // setMotor(RFD, 120);
  // setMotor(RTE, 120);
  // setMotor(RTD, 120);

  // delay(2000);

  // Parar();

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  agora = millis();

  if (agora - ultimoControle >= TCONTROLE) {
    double deltaT = (agora - ultimoControle) / 1000.0;

if (agora - ultimoCmdVel > 500) {
  vx_alvo = 0;
  vy_alvo = 0;
  w_alvo = 0;

  rpmRFE = 0;
  rpmRFD = 0;
  rpmRTE = 0;
  rpmRTD = 0;

  Parar();
  zerarIntegrais();
} else {
      movimentar(vx_alvo, vy_alvo, w_alvo, deltaT);
    }

    ultimoControle = agora;
  }

  if (agora - ultimoDebug >= 500) {
    publicarDebugVelocidades();
    ultimoDebug = agora;
  }

  delay(1);
}