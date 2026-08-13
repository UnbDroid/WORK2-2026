/*
 * This is a simple template to use microros with ESP32-Arduino
 * This sketch has sample of publisher to publish from timer_callback
 * and subscrption to listen of new data from other ROS node.
 * 
 * Some of the codes below are gathered from github and forums
 * 
 * Made by Rasheed Kittinanthapanya
 * 
*/

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "esp_system.h"
#include <ESP32Servo.h>

/*
 * TODO : Include your necessary header here
*/

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
/*
 * TODO : include your desired msg header file
*/
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>

/*
 * Optional,
 * LED pin to check connection
 * between micro-ros-agent and ESP32
*/
// #define LED_PIN 23
// #define LED_PIN_TEST 18 // subscription LED
// bool led_test_state = false;

/*
 * Helper functions to help reconnect
*/
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/*
 * Declare rcl object
*/
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your 
 * publisher & subscription objects below
*/
rcl_subscription_t garra_sub;
// rcl_subscription_t led_sub;
/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/
std_msgs__msg__String garra_msg;
// std_msgs__msg__Bool led_msg;

char command_buffer[32]; //diminuir
int giro180 = 1600;
int giro90 = 800;
int giro270 = 2400;

void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Motivo do reset: ");
  switch (reason) {
    case ESP_RST_POWERON:   Serial.println("POWERON (ligou da tomada/USB)"); break;
    case ESP_RST_EXT:       Serial.println("EXT (pino de reset externo)"); break;
    case ESP_RST_SW:        Serial.println("SW (reset por software)"); break;
    case ESP_RST_PANIC:     Serial.println("PANIC (crash de código / exceção)"); break;
    case ESP_RST_INT_WDT:   Serial.println("INT_WDT (watchdog de interrupção travada)"); break;
    case ESP_RST_TASK_WDT:  Serial.println("TASK_WDT (watchdog de task travada)"); break;
    case ESP_RST_WDT:       Serial.println("WDT (outro watchdog)"); break;
    case ESP_RST_BROWNOUT:  Serial.println(">>> BROWNOUT (queda de tensão) <<<"); break;
    case ESP_RST_SDIO:      Serial.println("SDIO"); break;
    default:                Serial.println("Outro / desconhecido"); break;
  }
}

//Stepper Motor

#define stepPin 33
#define dirPin 32
#define MS1 27
#define MS2 26
#define MS3 25

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/
// void led_callback(const void *msgin) {

//   std_msgs__msg__Bool *led_msg = (std_msgs__msg__Bool *)msgin;
//   /*
//    * Do something with your receive message
//    */
//   led_test_state = led_msg->data;
//   digitalWrite(LED_PIN_TEST, led_test_state);

// }

void girar_90() {
  if (!stepper) return;

  Serial.println("Moving to position 800");
  stepper->moveTo(giro90, true);
}

void girar_180() {
  if (!stepper) return;

  Serial.println("Moving to position 1600");
  stepper->moveTo(giro180, true);

}

void girar_270() {
  if (!stepper) return;
    
  Serial.println("Moving to position 2400");
  stepper->moveTo(giro270, true);
}

void voltar() {
  if (!stepper) return;

  Serial.println("Moving to position 0");
  stepper->moveTo(0, true);     // blocking (tirar true para não bloquear)
}

// Motor DC

#define IN1 21
#define IN2 19
#define ENCODER_A 23 //branco
#define ENCODER_B 22

volatile long pulso_global = 0;      // posição absoluta REAL, só a ISR escreve aqui
volatile long pulsos_restantes = 0;  // contagem regressiva do movimento atual

const long alvo1 = 10000;
const long alvo2 = 20000;

void pararMotor() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
}

void IRAM_ATTR encoderISR() {

    // Determina o sentido usando o canal B
    if (digitalRead(ENCODER_B)) {
        pulso_global++;
    } else {
        pulso_global--;
    }

    // Para o motor dentro do interrupt para não depender do loop
    if (pulsos_restantes > 0) {
      pulsos_restantes--;
      if (pulsos_restantes == 0) {
        pararMotor();
        //detachInterrupt(digitalPinToInterrupt(ENCODER_A)); (causa erro)
      }
    }
}

// Alvo desejado deve ter referência global
void iniciarMotorDC(long alvoDesejado) {
  long delta = alvoDesejado - pulso_global;

  if (delta == 0) {
    pararMotor();          // já está no lugar certo, nem liga o motor
    return;
  }

  noInterrupts(); // protege a seção crítica enquanto reseta o estado
  pulsos_restantes = labs(delta);
  interrupts();

  // Reanexa a interrupção, caso tenha sido desligada num movimento anterior
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  // Girar em sentido horário (subir) caso alvo desejado seja maior e anti-horário (descer) caso seja menor
  if (delta > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  while (true) {
    long restante_temp;
    
    // Leitura segura da variável modificada pela ISR
    noInterrupts();
    restante_temp = pulsos_restantes;
    interrupts();

    // Se chegou ao destino, sai do loop
    if (restante_temp <= 0) {
      break;
    }

    // Processa chamadas do micro-ROS enquanto espera, evitando timeout da comunicação
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    
    // Alimenta o Watchdog do ESP32 para evitar resets por PANIC/WDT
    yield(); 
  }
}

void garra_callback(const void *msgin)
{
    const std_msgs__msg__String *garra_msg =
        (const std_msgs__msg__String *)msgin;

    if (strcmp(garra_msg->data.data, "90") == 0)
    {
        Serial.println("Comando 90 recebido!");

        iniciarMotorDC(10000);
        girar_90();
        iniciarMotorDC(0);
    }

    else if (strcmp(garra_msg->data.data, "180") == 0)
    {
        Serial.println("Comando 180 recebido!");

        girar_180();
        iniciarMotorDC(10000);
        girar_90();
        girar_270();
        iniciarMotorDC(0);
        voltar();
        girar_180();
    }

    else if (strcmp(garra_msg->data.data, "270") == 0)
    {
        Serial.println("Comando 270 recebido!");

        girar_270();
        iniciarMotorDC(20000);
    }

    else if (strcmp(garra_msg->data.data, "0") == 0)
    {
        Serial.println("Comando de retornar a origem recebido!");

        voltar();
    }

    else
    {
        Serial.print("Comando desconhecido: ");
        Serial.println(garra_msg->data.data);
    }
}

/*
   Create object (Initialization)
*/
bool create_entities()
{
  /*
     TODO : Define your
     - ROS node name
     - namespace
     - ROS_DOMAIN_ID
  */
  const char * node_name = "esp32_node";
  const char * ns = "";
  const int domain_id = 0;
  
  /*
   * Initialize node ?
   */
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  
  /*
   * TODO : Init your publisher and subscriber 
   */

  rclc_subscription_init_default(
    &garra_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/topico_garra"
  );

  // rclc_subscription_init(
  //   &led_sub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  //   "/esp/led", &rmw_qos_profile_default
  // );

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 1;

  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &garra_sub, &garra_msg, &garra_callback, ON_NEW_DATA);
  // rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);

  return true;
}
/*
 * Clean up all the created objects
 */
void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  // rcl_subscription_fini(&led_sub, &node);
  rcl_subscription_fini(&garra_sub, &node);
  rcl_node_fini(&node);
  rcl_init_options_fini(&init_options);
  rclc_support_fini(&support);
  /*
   * TODO : Make sure the name of publisher and subscriber are correct
   */
  
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  printResetReason(); 
  delay(2000);
  /*
   * TODO : select either of USB or WiFi 
   * comment the one that not use
   */
  set_microros_serial_transports(Serial);
  delay(2000);
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

  /*
   * Optional, setup output pin for LEDs
   */
  // pinMode(LED_PIN, OUTPUT);
  // pinMode(LED_PIN_TEST, OUTPUT);


  // Stepper driver
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  // Initialize engine
    engine.init();
    
    // Connect and configure stepper
    stepper = engine.stepperConnectToPin(stepPin);
    if (stepper) {
        stepper->setDirectionPin(dirPin);
        stepper->setAutoEnable(true);
        
        // Motion parameters
        stepper->setSpeedInHz(4000);       
        stepper->setAcceleration(8000);    
        
        Serial.println("Stepper initialized");
    } else {
        Serial.println("Stepper connection failed!");
    }

  //Motor DC
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_A),
      encoderISR,
      RISING
  );

  /*
   * TODO : Initialze the message data variable
   */
  // Message buffer
  garra_msg.data.data = command_buffer;
  garra_msg.data.capacity = sizeof(command_buffer);
  garra_msg.data.size = 0;

  // led_msg.data = false;

  /*
   * Setup first state
   */
  state = WAITING_AGENT;

}

void loop() {
  /*
   * Try ping the micro-ros-agent (HOST PC), then switch the state 
   * from the example
   * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
   * 
   */
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;); //mudado de 100 para 10 para evitar engasgos
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); //talvez 100ms seja tempo demais
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // Print periódico da contagem de pulsos, sem bloquear o loop
  EXECUTE_EVERY_N_MS(100, Serial.println(pulso_global););

  /*
   * Output LED when in AGENT_CONNECTED state
   */
  // if (state == AGENT_CONNECTED) {
  //   digitalWrite(LED_PIN, 1);
  // } else {
  //   digitalWrite(LED_PIN, 0);
  // }

  /*
   * TODO : 
   * Do anything else you want to do here,
   * like read sensor data,  
   * calculate something, etc.
   */

}