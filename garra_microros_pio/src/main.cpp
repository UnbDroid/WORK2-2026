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

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>

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

rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_subscription_t garra_sub;
rcl_publisher_t garra_status_pub; //publica status de acao para behaviour tree

std_msgs__msg__String garra_msg;
std_msgs__msg__String status_msg;

char status_buffer[16];

#define stepPin1 14
#define dirPin1 27
#define MS1_1 26

char command_buffer[32]; //diminuir
int slot1 = 3600;
int slot2 = 4800;
int slot3 = 6000;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;

void rotacionar(int npasso) {
  if (!stepper1) return;

  // Serial.println("Moving to position 800");
  stepper1->moveTo(npasso, true);
  publish_status("1");
}

//Stepper Motor2 (movimento vertical)

#define stepPin2 33 
#define dirPin2 32
#define enablePin2 25

int altura_inicial = 0;
int cinco_cm = 0;
int dez_cm = -60000;
int quinze_cm = 12800;
int shelf_cm = 60000;

FastAccelStepper *stepper2 = NULL;

void vertical(int npasso) {
  if (!stepper2) return;

  // Serial.println("Moving to position 800");
  stepper2->moveTo(npasso, true);
  publish_status("1");
}

//Servomotor

Servo myservo;
#define SERVO_PIN 18

void fechar_garra(){
  myservo.write(90);
  delay(500);
  publish_status("1");
}

void abrir_garra(){
  myservo.write(0);
  delay(500);
  publish_status("1");
}

void publish_status(const char * cmd_done) {
  strncpy(status_buffer, cmd_done, sizeof(status_buffer));
  status_msg.data.data = status_buffer;
  status_msg.data.size = strlen(status_buffer);
  status_msg.data.capacity = sizeof(status_buffer);
  rcl_publish(&garra_status_pub, &status_msg, NULL);
}

void garra_callback(const void *msgin)
{
    const std_msgs__msg__String *garra_msg =
        (const std_msgs__msg__String *)msgin;

    if (strcmp(garra_msg->data.data, "1") == 0)
    {
        // Serial.println("Comando 90 recebido!");
      rotacionar(slot1);
        
    }

    else if (strcmp(garra_msg->data.data, "2") == 0)
    {
        // Serial.println("Comando 180 recebido!");
      rotacionar(slot2);
        
    }
    else if (strcmp(garra_msg->data.data, "3") == 0)
    {
        // Serial.println("Comando 270 recebido!");
      rotacionar(slot3);
        
    }

    else if (strcmp(garra_msg->data.data, "0") == 0)
    {
        // Serial.println("Comando de retornar a origem recebido!");
      rotacionar(0);
        
    }

    else if (strcmp(garra_msg->data.data, "5cm") == 0)
    {
        // Serial.println("Comando 180 recebido!");
      vertical(cinco_cm);
        
    }
    else if (strcmp(garra_msg->data.data, "10cm") == 0)
    {
        // Serial.println("Comando 270 recebido!");
      vertical(dez_cm);
        
    }

    else if (strcmp(garra_msg->data.data, "15cm") == 0)
    {
        // Serial.println("Comando de retornar a origem recebido!");
      vertical(quinze_cm);
        
    }

    else if (strcmp(garra_msg->data.data, "shelf") == 0)
    {
        // Serial.println("Comando de retornar a origem recebido!");
      vertical(shelf_cm);
        
    }

    else if (strcmp(garra_msg->data.data, "abre") == 0)
    {
        // Serial.println("Comando de retornar a origem recebido!");
      abrir_garra();
        
    }

    else if (strcmp(garra_msg->data.data, "fecha") == 0)
    {
        // Serial.println("Comando de retornar a origem recebido!");
      fechar_garra();
        
    }

    else
    {
        // Serial.print("Comando desconhecido: ");
        // Serial.println(garra_msg->data.data);
    }
}

/*
   Create object (Initialization)
*/
bool create_entities()
{
  const char * node_name = "esp32_node";
  const char * ns = "";
  const int domain_id = 0;

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();

  rcl_ret_t rc;

  rc = rcl_init_options_init(&init_options, allocator);
  if (rc != RCL_RET_OK) return false;

  rc = rcl_init_options_set_domain_id(&init_options, domain_id);
  if (rc != RCL_RET_OK) return false;

  rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (rc != RCL_RET_OK) return false;

  rc = rclc_node_init_default(&node, node_name, ns, &support);
  if (rc != RCL_RET_OK) return false;

  rc = rclc_subscription_init_default(
    &garra_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/topico_garra"
  );
  if (rc != RCL_RET_OK) return false;

  unsigned int num_handles = 1;
  rc = rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  if (rc != RCL_RET_OK) return false;

  rc = rclc_executor_add_subscription(&executor, &garra_sub, &garra_msg, &garra_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) return false;

  rc = rclc_publisher_init_default(&garra_status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/topico_garra_status");
  if (rc != RCL_RET_OK) return false;

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_subscription_fini(&garra_sub, &node);
  rcl_node_fini(&node);
  rcl_init_options_fini(&init_options);
  rclc_support_fini(&support);
  rcl_publisher_fini(&garra_status_pub, &node);
  
}

void setup() {
  Serial.begin(115200);
  // delay(2000);
  // printResetReason(); 
  // delay(2000);
  /*
   * TODO : select either of USB or WiFi 
   * comment the one that not use
   */
  set_microros_serial_transports(Serial);
  delay(2000);
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

  // Stepper driver 1
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(MS1_1, OUTPUT);

  digitalWrite(MS1_1, HIGH);

  // Stepper driver 2
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // Initialize engine1
    engine.init();
    
    // Connect and configure stepper
    stepper1 = engine.stepperConnectToPin(stepPin1);
    if (stepper1) {
        stepper1->setDirectionPin(dirPin1);
        stepper1->setAutoEnable(true);
        
        // Motion parameters
        stepper1->setSpeedInHz(1000);       
        stepper1->setAcceleration(1500);    
        
        // Serial.println("Stepper initialized");
    } else {
        // Serial.println("Stepper connection failed!");
    }
    
    // Connect and configure stepper
    stepper2 = engine.stepperConnectToPin(stepPin2);
    if (stepper2) {
        stepper2->setDirectionPin(dirPin2);
        stepper2->setEnablePin(enablePin2, true);  // true = active-low
        stepper2->setAutoEnable(true);
        
        // Motion parameters
        stepper2->setSpeedInHz(8000);       
        stepper2->setAcceleration(6000);    
        
        // Serial.println("Stepper initialized");
    } else {
        // Serial.println("Stepper connection failed!");
    }

  //Servomotor
  myservo.attach(SERVO_PIN);

  // Message buffer
  garra_msg.data.data = command_buffer;
  garra_msg.data.capacity = sizeof(command_buffer);
  garra_msg.data.size = 0;

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
  // EXECUTE_EVERY_N_MS(100, Serial.println(pulso_global););

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