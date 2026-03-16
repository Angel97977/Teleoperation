import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import csv
import time

class ForceLogger(Node):
    def __init__(self):
        super().__init__('DataSaver')
        
        # Nos suscribimos al tópico de la ESP32
        self.subscription = self.create_subscription(
            Float32,
            '/load_cell_weight',
            self.listener_callback,
            10)
            
        # Creamos un archivo CSV con la fecha/hora actual en el nombre
        self.file_name = f"force_data_{int(time.time())}.csv"
        
        # Escribimos los encabezados de las columnas
        with open(self.file_name, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time_Seconds', 'Force_Newtons'])
            
        self.get_logger().info(f'💾 Guardando datos en: {self.file_name} ... Presiona Ctrl+C para detener.')
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def listener_callback(self, msg):
        # Calculamos el tiempo transcurrido desde que inició el script
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        force_value = msg.data
        
        # Agregamos la nueva fila al archivo CSV
        with open(self.file_name, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([current_time, force_value])

def main(args=None):
    rclpy.init(args=args)
    force_logger = ForceLogger()
    
    try:
        rclpy.spin(force_logger)
    except KeyboardInterrupt:
        force_logger.get_logger().info('Finalizando guardado de datos.')
    finally:
        force_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()