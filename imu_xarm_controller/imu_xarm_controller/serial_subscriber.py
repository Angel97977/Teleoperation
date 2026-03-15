#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Importamos Float32 porque es el tipo de dato que manda nuestro publicador

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber_node')
        
        # Declaramos la suscripción: Tipo de mensaje, Tópico exacto, Callback, QoS (tamaño de cola)
        self.subscription = self.create_subscription(
            Float32,
            '/load_cell_weight',
            self.listener_callback,
            10
        )
        self.get_logger().info('🎧 ¡Suscriptor iniciado! Escuchando los datos del sensor de la ESP32...')

    def listener_callback(self, msg):
        # Extraemos el valor numérico del mensaje
        peso_actual = msg.data
        
        # Aquí va la lógica de lo que quieres hacer con el dato
        # Por ahora, solo lo vamos a imprimir en la terminal de forma bonita
        self.get_logger().info(f'📦 Peso recibido: {peso_actual:.2f} newtons')

        # Ejemplo de lógica rápida:
        if peso_actual > 500.0:
            self.get_logger().warning('⚠️ ¡Advertencia! El peso superó los 500 gramos.')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Apagando suscriptor de manera segura...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()