// ESP code para leer la celda de carga HX711 y publicar el peso en un tópico de micro-ROS
#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include "HX711.h"

// Pines de la celda de carga
#define DOUT 15
#define CLK  2

HX711 scale;

// Factor de calibración predefinido (ya no lo cambiamos por Serial)
float calibration_factor = 54.0;  

// Variables de micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Macros para manejo de errores de micro-ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

// Esta función se ejecuta cada vez que el timer se dispara
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    // Solo leemos si el sensor está listo para no bloquear el microcontrolador
    if (scale.is_ready()) {
      scale.set_scale(calibration_factor);
      
      // Hacemos solo 1 lectura a la vez. 
      // Si hacemos 10 lecturas como en tu código original, micro-ROS podría desincronizarse por tardar mucho.
      float units = scale.get_units(1); 

      if (units < 0) {
        units = 0.0;
      }

      // Asignamos el valor en gramos al mensaje y publicamos
      msg.data = units*9.81;
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
  }
}

void setup() {
  // Inicializamos el transporte de micro-ROS (Toma el control del Serial)
  set_microros_transports();
  
  // Inicializamos la celda de carga
  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare(); // Resetea a 0 (Asegúrate de no tener peso al encenderlo)

  delay(1000); // Pequeña pausa para estabilizar la conexión

  allocator = rcl_get_default_allocator();

  // Inicializamos el soporte de micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Creamos el nodo llamado "hx711_node"
  RCCHECK(rclc_node_init_default(&node, "hx711_node", "", &support));

  // Creamos el publicador
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "load_cell_weight")); // Nombre del tópico

  // Creamos un timer para publicar a 10 Hz (cada 100 ms)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Creamos el ejecutor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(10);
  // Mantenemos vivo el nodo de micro-ROS
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}