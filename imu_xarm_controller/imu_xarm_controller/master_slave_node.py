#!/usr/bin/env python3
import rclpy, math, time
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from xarm_msgs.srv import SetInt16, SetInt16ById, MoveJoint, Call

class MasterSlaveController(Node):
    def __init__(self):
        super().__init__('master_slave_node')

        self.declare_parameter('master_namespace',   '/ufactory')
        self.declare_parameter('slave_namespace',    '/ufactory2')
        self.declare_parameter('mirror_rate_hz',     30.0)
        self.declare_parameter('move_acc',           8.0)   
        self.declare_parameter('deadband_rad',       0.005)
        self.declare_parameter('smooth_window',      4)     
        
        # --- LÍMITE AJUSTADO A NEWTONS ---
        # 1000 Kg * 9.81 = 9810.0 Newtons
        self.declare_parameter('collision_limit',    9810.0) 

        mn = self.get_parameter('master_namespace').value
        sn = self.get_parameter('slave_namespace').value
        self.rate_hz     = self.get_parameter('mirror_rate_hz').value
        self.acc         = self.get_parameter('move_acc').value
        self.deadband    = self.get_parameter('deadband_rad').value
        smooth_win       = self.get_parameter('smooth_window').value
        self.limit       = self.get_parameter('collision_limit').value

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

        self._latest_master   = None
        self._master_prev_pos = None
        self._prev_time       = None
        self._smooth_vel      = 0.0
        self._emergency_stop  = False 

        self._pos_buffer = deque(maxlen=smooth_win)

        self.get_logger().info('Esperando servicios...')
        self.s_servo.wait_for_service()
        self.m_mode.wait_for_service()
        self.get_logger().info('¡Brazos encontrados!')

        self._arm_init('Master', self.m_clear, self.m_enable, self.m_mode, self.m_state)
        self._arm_init('Slave',  self.s_clear, self.s_enable, self.s_mode, self.s_state)

        self.create_subscription(JointState, f'{mn}/joint_states', self._master_js_cb, 10)
        
        # Escuchamos directamente el tópico de micro-ROS
        self.create_subscription(Float32, '/load_cell_weight', self._sensor_cb, 10)

        self.get_logger().info('Esperando lecturas de posición del maestro...')
        while rclpy.ok() and self._latest_master is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Configurando Maestro(Drag/2) y Esclavo(Online Replan/6)...')
        self._set_mode_state(self.m_mode, self.m_state, 2)
        self._set_mode_state(self.s_mode, self.s_state, 6)
        time.sleep(1.0)

        self._master_prev_pos = list(self._latest_master.position[:6])
        for _ in range(self._pos_buffer.maxlen):
            self._pos_buffer.append(self._master_prev_pos[:])

        self.create_timer(1.0 / self.rate_hz, self._control_tick)
        self.get_logger().info(f'✅ ¡Sistema listo! E-Stop activado si la fuerza supera los {self.limit} N')

    def _arm_init(self, name, cli_clear, cli_enable, cli_mode, cli_state):
        cli_clear.call_async(Call.Request())
        time.sleep(0.4)
        req = SetInt16ById.Request(); req.id = 8; req.data = 1
        cli_enable.call_async(req)
        time.sleep(0.4)
        self._set_mode_state(cli_mode, cli_state, 0)
        self.get_logger().info(f'{name} inicializado.')

    def _set_mode_state(self, cli_mode, cli_state, mode):
        r = SetInt16.Request(); r.data = mode
        cli_mode.call_async(r)
        time.sleep(0.4)
        r2 = SetInt16.Request(); r2.data = 0
        cli_state.call_async(r2)
        time.sleep(0.4)

    def _master_js_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self._latest_master = msg

    def _sensor_cb(self, msg: Float32):
        # Evaluamos el mensaje en Newtons
        if not self._emergency_stop and msg.data >= self.limit:
            self._emergency_stop = True
            self.get_logger().fatal(f'🚨 ¡COLISIÓN DETECTADA! Fuerza límite excedida: {msg.data:.2f} N >= {self.limit} N')
            
            r_stop = SetInt16.Request()
            r_stop.data = 4
            self.s_state.call_async(r_stop)
            self.m_state.call_async(r_stop)
            self.get_logger().fatal('🚨 BRAZOS BLOQUEADOS EN STATE 4 POR SEGURIDAD.')

    def _control_tick(self):
        if self._emergency_stop:
            return 
            
        if self._latest_master is None:
            return

        now = time.time()
        dt  = (now - self._prev_time) if self._prev_time else (1.0 / self.rate_hz)
        dt  = max(0.001, dt)
        self._prev_time = now

        master_now_rad = list(self._latest_master.position[:6])
        delta_rad      = [master_now_rad[i] - self._master_prev_pos[i] for i in range(6)]
        raw_vel_rad    = max(abs(d) / dt for d in delta_rad)

        self._smooth_vel = (0.3 * raw_vel_rad) + (0.7 * self._smooth_vel)

        self._master_prev_pos = master_now_rad[:]

        if raw_vel_rad > self.deadband:
            self._pos_buffer.append(master_now_rad[:])
            n = len(self._pos_buffer)
            smoothed_pos = [
                sum(self._pos_buffer[s][j] for s in range(n)) / n
                for j in range(6)
            ]

            speed_to_send = max(0.05, self._smooth_vel)
            mvtime = 1.5 / self.rate_hz 

            self._execute_slave_move(smoothed_pos, speed_to_send, mvtime)
        else:
            self._smooth_vel = 0.0
            if self._master_prev_pos:
                self._pos_buffer.clear()
                for _ in range(self._pos_buffer.maxlen):
                    self._pos_buffer.append(self._master_prev_pos[:])

    def _execute_slave_move(self, target_angles_rad, speed_rad, mvtime=0.0):
        req = MoveJoint.Request()
        req.angles = target_angles_rad
        req.speed  = float(speed_rad)
        req.acc    = self.acc
        req.mvtime = float(mvtime)
        req.wait   = False
        self.s_servo.call_async(req)

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