import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import time

class DataSaver(Node):
    def __init__(self):
        super().__init__('data_saver_node')
        
        # Variables para almacenar la última lectura
        self.master_q = [0.0] * 6
        self.slave_q = [0.0] * 6
        self.data_received = False
        
        # Suscriptores (ufactory2 = Master, ufactory = Slave)
        self.sub_master = self.create_subscription(
            JointState, '/ufactory2/joint_states', self.master_callback, 10)
            
        self.sub_slave = self.create_subscription(
            JointState, '/ufactory/joint_states', self.slave_callback, 10)
            
        # Archivo CSV
        self.file_name = f"DataSaver_{int(time.time())}.csv"
        self.headers = [
            'time_s', 
            'q1_master_rad', 'q1_slave_rad', 'e1_rad',
            'q2_master_rad', 'q2_slave_rad', 'e2_rad',
            'q3_master_rad', 'q3_slave_rad', 'e3_rad',
            'q4_master_rad', 'q4_slave_rad', 'e4_rad',
            'q5_master_rad', 'q5_slave_rad', 'e5_rad',
            'q6_master_rad', 'q6_slave_rad', 'e6_rad'
        ]
        
        # Crear archivo y escribir encabezados
        with open(self.file_name, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.headers)
            
        self.get_logger().info(f'💾 Guardando datos en: {self.file_name} ... Presiona Ctrl+C para detener.')
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Timer para guardar datos a 50Hz (0.02 segundos)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def master_callback(self, msg):
        # Tomamos las primeras 6 articulaciones
        if len(msg.position) >= 6:
            self.master_q = list(msg.position[:6])
            self.data_received = True

    def slave_callback(self, msg):
        if len(msg.position) >= 6:
            self.slave_q = list(msg.position[:6])

    def timer_callback(self):
        if not self.data_received:
            return # Esperamos a recibir datos antes de empezar a grabar
            
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        row = [current_time]
        for i in range(6):
            qm = self.master_q[i]
            qs = self.slave_q[i]
            e = qm - qs # Calculamos el error
            
            row.extend([qm, qs, e])
            
        # Guardar la fila en el CSV
        with open(self.file_name, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(row)

def main(args=None):
    rclpy.init(args=args)
    data_saver = DataSaver()
    
    try:
        rclpy.spin(data_saver)
    except KeyboardInterrupt:
        data_saver.get_logger().info('Finalizando guardado de datos.')
    finally:
        data_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()