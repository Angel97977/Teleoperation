#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # 1. Configuración del puerto serial (Ajusta la ruta si en Linux te sale /dev/ttyACM0)
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 9600
        
        try:
            # Abrimos el puerto. Timeout de 0.1s para que no se congele si no hay datos.
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'✅ Conectado al serial {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Error al abrir {self.serial_port}: {e}')
            self.ser = None

        # 2. Declaramos el Publicador (Tópico: /load_cell_weight, Tipo: Float32)
        self.publisher_ = self.create_publisher(Float32, '/load_cell_weight', 10)
        
        # 3. Creamos un temporizador que cheque el serial rápido (20 Hz)
        self.timer = self.create_timer(0.05, self.read_serial_and_publish)
        self.get_logger().info('¡Nodo puente iniciado y publicando datos!')

    def read_serial_and_publish(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                # Leemos la línea que manda la ESP, la decodificamos y quitamos espacios/saltos
                line = self.ser.readline().decode('utf-8').strip()
                
                if line:
                    # Convertimos el texto (ej. "120.5") a número decimal
                    peso = float(line)
                    
                    # Creamos el mensaje de ROS y le asignamos el valor
                    msg = Float32()
                    msg.data = peso
                    
                    # ¡Publicamos al entorno de ROS 2!
                    self.publisher_.publish(msg)
                    
                    # (Opcional) Quita el "#" de abajo si quieres ver los datos en la terminal
                    # self.get_logger().info(f'Publicando: {msg.data} g')
                    
            except ValueError:
                # Si la ESP manda texto de calibración ("Zero factor: ...") lo ignoramos para no crashear
                pass
            except Exception as e:
                self.get_logger().warning(f'Error leyendo el serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Apagando puente serial...")
    finally:
        # Cerramos el puerto limpio al salir
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()