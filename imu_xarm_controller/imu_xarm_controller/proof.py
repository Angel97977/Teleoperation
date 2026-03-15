#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from xarm_msgs.srv import SetInt16, SetInt16ById, MoveJoint, Call

# Los límites los dejamos en grados por comodidad visual
JOINT_LIMITS = [
    (-175.0, 175.0),
    (-220.0,  40.0),
    (  -6.0, 220.0),
    (-175.0, 175.0),
    ( -92.0, 175.0),
    (-355.0, 355.0),
]

class MasterSlaveController(Node):
    def __init__(self):
        super().__init__('master_slave_node')

        self.declare_parameter('master_namespace',   '/ufactory')
        self.declare_parameter('slave_namespace',    '/ufactory2')
        self.declare_parameter('mirror_rate_hz',     10.0)
        self.declare_parameter('move_speed',         0.1)  # 0.1 rad/s (aprox 5.7 grados/s)
        self.declare_parameter('move_acc',           0.05)
        self.declare_parameter('velocity_threshold', 2.0)  # Threshold en grados/s
        self.declare_parameter('delay_sec',          1.5)

        mn = self.get_parameter('master_namespace').value
        sn = self.get_parameter('slave_namespace').value
        self.rate_hz    = self.get_parameter('mirror_rate_hz').value
        self.speed      = self.get_parameter('move_speed').value
        self.acc        = self.get_parameter('move_acc').value
        self.vel_thresh = self.get_parameter('velocity_threshold').value
        self.delay_sec  = self.get_parameter('delay_sec').value

        def cli(ns, srv_type, name):
            return self.create_client(srv_type, f'{ns}/{name}')

        self.m_clear  = cli(mn, Call,         'clean_error')
        self.m_enable = cli(mn, SetInt16ById, 'motion_enable')
        self.m_mode   = cli(mn, SetInt16,     'set_mode')
        self.m_state  = cli(mn, SetInt16,     'set_state')
        
        self.s_clear  = cli(sn, Call,         'clean_error')
        self.s_enable = cli(sn, SetInt16ById, 'motion_enable')
        self.s_mode   = cli(sn, SetInt16,     'set_mode')
        self.s_state  = cli(sn, SetInt16,     'set_state')
        self.s_servo  = cli(sn, MoveJoint,    'set_servo_angle')

        self._latest_master = None
        self._master_prev_pos = None # Ahora guardará RADIANES
        self._prev_time = None

        self._is_moving = False
        self._stopped_time = None

        self.get_logger().info('Esperando servicios...')
        self.s_servo.wait_for_service()
        self.m_mode.wait_for_service()
        self.get_logger().info('¡Brazos encontrados!')

        self._arm_init('Master', self.m_clear, self.m_enable, self.m_mode, self.m_state)
        self._arm_init('Slave',  self.s_clear, self.s_enable, self.s_mode, self.s_state)

        self.create_subscription(JointState, f'{mn}/joint_states', self._master_js_cb, 10)

        self.get_logger().info('Esperando lecturas de posición del maestro...')
        while rclpy.ok() and self._latest_master is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Configurando Maestro(Drag/2) y Esclavo(Position/0)...')
        self._set_mode_state(self.m_mode, self.m_state, 2)
        self._set_mode_state(self.s_mode, self.s_state, 0)
        time.sleep(1.0) 

        # IMPORTANTE: Guardamos la posición inicial en RADIANES
        self._master_prev_pos = list(self._latest_master.position[:6])
        grados_iniciales = [math.degrees(p) for p in self._master_prev_pos]
        self.get_logger().info(f'Posición inicial: {[f"{a:.1f}°" for a in grados_iniciales]}')

        self.create_timer(1.0 / self.rate_hz, self._control_tick)
        self.get_logger().info('✅ ¡Sistema listo! Esperando a que muevas el maestro...')

    def _arm_init(self, name, cli_clear, cli_enable, cli_mode, cli_state):
        cli_clear.call_async(Call.Request()); time.sleep(0.4)
        req = SetInt16ById.Request(); req.id = 8; req.data = 1
        cli_enable.call_async(req); time.sleep(0.4)
        self._set_mode_state(cli_mode, cli_state, 0)
        self.get_logger().info(f'{name} inicializado.')

    def _set_mode_state(self, cli_mode, cli_state, mode):
        r = SetInt16.Request(); r.data = mode
        cli_mode.call_async(r); time.sleep(0.4)
        r2 = SetInt16.Request(); r2.data = 0
        cli_state.call_async(r2); time.sleep(0.4)

    def _master_js_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self._latest_master = msg

    def _control_tick(self):
        if self._latest_master is None:
            return

        now = time.time()
        dt  = (now - self._prev_time) if self._prev_time else 0.1
        dt  = max(0.001, dt)
        self._prev_time = now

        # 1. Obtenemos posición actual en RADIANES
        master_now_rad = list(self._latest_master.position[:6])
        
        # 2. Calculamos diferencia
        delta_rad = [master_now_rad[i] - self._master_prev_pos[i] for i in range(6)]
        
        # 3. Calculamos la velocidad en GRADOS para comparar con el vel_thresh fácilmente
        max_vel_rad = max(abs(d) / dt for d in delta_rad)
        max_vel_deg = math.degrees(max_vel_rad)

        self._master_prev_pos = master_now_rad[:]

        if max_vel_deg > self.vel_thresh:
            if not self._is_moving:
                self.get_logger().info('🔹 Maestro moviéndose...', throttle_duration_sec=1.0)
            self._is_moving = True
            self._stopped_time = None
        else:
            if self._is_moving:
                if self._stopped_time is None:
                    self._stopped_time = now
                
                if (now - self._stopped_time) >= self.delay_sec:
                    self.get_logger().info('✅ Maestro detenido. Replicando...')
                    self._is_moving = False
                    self._stopped_time = None
                    # Le enviamos los radianes al esclavo
                    self._execute_slave_move(master_now_rad)

    def _execute_slave_move(self, target_angles_rad):
        clamped_rad = []
        for i, (a_rad, (lo_deg, hi_deg)) in enumerate(zip(target_angles_rad, JOINT_LIMITS)):
            # Convertimos los límites de grados a radianes para comparar
            lo_rad = math.radians(lo_deg)
            hi_rad = math.radians(hi_deg)
            
            c_rad = max(lo_rad, min(hi_rad, a_rad))
            
            if c_rad != a_rad:
                self.get_logger().warn(f'Límite J{i+1}: {math.degrees(a_rad):.1f}° truncado a {math.degrees(c_rad):.1f}°')
            clamped_rad.append(c_rad)

        req = MoveJoint.Request()
        req.angles = clamped_rad  # AHORA SÍ ESTÁ EN RADIANES
        req.speed  = self.speed
        req.acc    = self.acc
        req.mvtime = 0.0
        req.wait   = False
        self.s_servo.call_async(req)
        
        # Imprimimos en grados para que nosotros lo entendamos en consola
        self.get_logger().info(f'Esclavo en camino a: {[f"{math.degrees(a):.1f}°" for a in clamped_rad]}')

def main(args=None):
    rclpy.init(args=args)
    node = MasterSlaveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Apagando nodo manualmente...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()