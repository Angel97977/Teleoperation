"""
ROBOT ESCLAVO TE3001B - Peg-in-Hole Teleoperado
Control de Impedancia + DetecciÃ³n de Contacto
Prof. Alberto MuÃ±oz Â· amunoz@tec.mx
TE3001B â€” FundamentaciÃ³n de RobÃ³tica Computational Robotics Lab Â· Tec de Monterrey

Ejecutar en la PC ESCLAVO:
	python3 Slave.py

Nodos de red:
	escucha UDP: 9001 (recibe xd del maestro)
	envÃ­a UDP:   9002 al maestro (Fe, estado)  

# Controles manuales
#   w/a/d   : desplazar referencia cartesiana (modo manual cuando el usuario presiona cualquiera de esas teclas)
#   s       : desplazar hacia abajo (movimiento, no guardar)
#   r       : reinicia por completo la tarea y el timer
#   t       : cuando este en COMPLETADO, pasar al estado TERMINADO
#   c       : guarda captura instantanea sin detener el programa
#   p       : cierra la ventana y guarda los datos

"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
# eliminar atajos por defecto que interfieren con los controles manuales (q para salir, s para guardar)
matplotlib.rcParams['keymap.quit'] = []
matplotlib.rcParams['keymap.save'] = []
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket
import threading
import time
import json
import argparse
from datetime import datetime

# parÃ¡metros del robot (idÃ©nticos al maestro)
L1, L2, L3 = 0.35, 0.30, 0.20
M1, M2, M3 = 1.5, 1.0, 0.5
G_GRAV = 9.81
DT = 0.01

# ganancias de impedancia - ajustadas para precisiÃ³n y suavidad
# se reducen para evitar picos grandes al hacer contacto
KD_IMP = 120.0  # rigidez cartesiana
BD_IMP = 35.0   # amortiguamiento
KP_ART = np.diag([80.0, 60.0, 50.0])   # rigidez articular
KV_ART = np.diag([15.0, 12.0, 10.0])   # amortiguamiento

# geometrÃ­a del Peg-in-Hole
PEG_LENGTH = 0.08
PEG_RADIUS = 0.008
HOLE_CENTER = np.array([0.55, 0.10])
HOLE_RADIUS = 0.009
F_CONTACT_K = 800.0   # rigidez de contacto reducida para evitar grandes picos de fuerza
F_THRESHOLD = 2.0
INSERT_LATERAL_TOL = HOLE_RADIUS + 0.003  # tolerancia lateral robusta para completar [m]
COMPLETE_DEPTH_TARGET = 0.020          # profundidad minima para completar [m]
COMPLETE_HOLD_TIME = 0.20              # tiempo continuo para confirmar completado [s]

def build_capture_tag(counter):
    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    return f'{ts}_{counter:03d}'

def fk_3r(q):
    q1, q2, q3 = q
    x1 = L1 * np.cos(q1); y1 = L1 * np.sin(q1)
    x2 = x1 + L2 * np.cos(q1 + q2); y2 = y1 + L2 * np.sin(q1 + q2)
    x3 = x2 + L3 * np.cos(q1 + q2 + q3); y3 = y2 + L3 * np.sin(q1 + q2 + q3)
    return np.array([x3, y3])


def fk_3r_full(q):
    q1, q2, q3 = q
    p0 = np.array([0.0, 0.0])
    p1 = np.array([L1 * np.cos(q1), L1 * np.sin(q1)])
    p2 = p1 + np.array([L2 * np.cos(q1 + q2), L2 * np.sin(q1 + q2)])
    p3 = p2 + np.array([L3 * np.cos(q1 + q2 + q3), L3 * np.sin(q1 + q2 + q3)])
    return np.array([p0, p1, p2, p3])


def jacobian_3r(q):
    q1, q2, q3 = q
    s1 = np.sin(q1); s12 = np.sin(q1 + q2); s123 = np.sin(q1 + q2 + q3)
    c1 = np.cos(q1); c12 = np.cos(q1 + q2); c123 = np.cos(q1 + q2 + q3)
    return np.array([
        [-L1 * s1 - L2 * s12 - L3 * s123, -L2 * s12 - L3 * s123, -L3 * s123],
        [ L1 * c1 + L2 * c12 + L3 * c123,  L2 * c12 + L3 * c123,  L3 * c123]
    ])


def inertia_matrix(q):
    q1, q2, q3 = q
    c2 = np.cos(q2); c3 = np.cos(q3); c23 = np.cos(q2 + q3)
    m11 = (M1 * L1 ** 2 + M2 * (L1 ** 2 + L2 ** 2 + 2 * L1 * L2 * c2) +
           M3 * (L1 ** 2 + L2 ** 2 + L3 ** 2 + 2 * L1 * L2 * c2 + 2 * L1 * L3 * c23 + 2 * L2 * L3 * c3))
    m12 = (M2 * (L2 ** 2 + L1 * L2 * c2) +
           M3 * (L2 ** 2 + L3 ** 2 + L1 * L2 * c2 + L1 * L3 * c23 + 2 * L2 * L3 * c3))
    m13 = M3 * (L3 ** 2 + L1 * L3 * c23 + L2 * L3 * c3)
    m22 = M2 * L2 ** 2 + M3 * (L2 ** 2 + L3 ** 2 + 2 * L2 * L3 * c3)
    m23 = M3 * (L3 ** 2 + L2 * L3 * c3)
    m33 = M3 * L3 ** 2
    return np.array([[m11, m12, m13], [m12, m22, m23], [m13, m23, m33]])


def coriolis_matrix(q, dq):
    eps = 1e-5
    C = np.zeros((3, 3))
    for k in range(3):
        qp = q.copy(); qp[k] += eps
        qm = q.copy(); qm[k] -= eps
        dM = (inertia_matrix(qp) - inertia_matrix(qm)) / (2 * eps)
        C += 0.5 * dM * dq[k]
    return C


def gravity_vector(q):
    q1, q2, q3 = q
    c1 = np.cos(q1); c12 = np.cos(q1 + q2); c123 = np.cos(q1 + q2 + q3)
    g1 = G_GRAV * ((M1 + M2 + M3) * L1 * c1 + (M2 + M3) * L2 * c12 + M3 * L3 * c123)
    g2 = G_GRAV * ((M2 + M3) * L2 * c12 + M3 * L3 * c123)
    g3 = G_GRAV * M3 * L3 * c123
    return np.array([g1, g2, g3])


def integrate_dynamics(q, dq, tau, dt=DT):
    M_mat = inertia_matrix(q)
    C_mat = coriolis_matrix(q, dq)
    g_vec = gravity_vector(q)
    ddq = np.linalg.solve(M_mat, tau - C_mat @ dq - g_vec)
    dq_new = np.clip(dq + ddq * dt, -2.0, 2.0)
    q_lim = np.array([np.pi / 2, 2 * np.pi / 3, np.pi / 2])
    q_new = np.clip(q + dq_new * dt, -q_lim, q_lim)
    return q_new, dq_new


class PegHoleContact:
    """Modelo de contacto elastico para insercion Peg-in-Hole."""
    APPROACH, CONTACT, INSERTION, COMPLETE = 0, 1, 2, 3
    STATE_NAMES = {0: 'APROXIMACION', 1: 'CONTACTO', 2: 'INSERCION', 3: 'COMPLETADO'}

    def __init__(self):
        self.phase = self.APPROACH
        self.depth = 0.0

    def compute_contact_force(self, x_ef):
        delta = x_ef - HOLE_CENTER
        dx = float(delta[0])
        dy = float(delta[1])
        lateral_err = abs(dx)
        depth = max(0.0, HOLE_CENTER[1] - x_ef[1])

        F_contact = np.zeros(2)
        in_contact = False

        near_hole_plane = abs(dy) < PEG_LENGTH * 1.5
        if not near_hole_plane:
            self.phase = self.APPROACH
            self.depth = 0.0
            return F_contact, self.STATE_NAMES[self.phase], in_contact

        if lateral_err <= INSERT_LATERAL_TOL:
            self.phase = self.INSERTION
            self.depth = depth
            # COMPLETADO se confirma en SlaveRobot.step con ventana temporal
        elif lateral_err <= HOLE_RADIUS + 0.02:
            self.phase = self.CONTACT
            penetration = lateral_err - HOLE_RADIUS
            F_mag = F_CONTACT_K * max(0.0, penetration)
            if lateral_err > 1e-9:
                F_contact = np.array([-F_mag * np.sign(dx), 0.0])
            in_contact = True
            self.depth = depth
        else:
            self.phase = self.APPROACH
            self.depth = depth

        return F_contact, self.STATE_NAMES[self.phase], in_contact


def impedance_control(q, dq, x_des, dx_des, F_contact=None, kd=KD_IMP, bd=BD_IMP):
    x_cur = fk_3r(q)
    J = jacobian_3r(q)
    dx_cur = J @ dq
    e_x = x_des - x_cur
    de_x = dx_des - dx_cur
    F_imp = kd * e_x + bd * de_x
    F_total = F_imp.copy()
    if F_contact is not None:
        F_total += F_contact
    tau_imp = J.T @ F_total
    g_vec = gravity_vector(q)
    C_mat = coriolis_matrix(q, dq)
    tau = tau_imp + g_vec + C_mat @ dq
    tau = np.clip(tau, -15.0, 15.0)
    return tau, F_total, e_x


class SlaveNetServer:
    """Servidor UDP del esclavo: recibe xd y responde Fe, contacto y posiciÃ³n.

    Ahora tambiÃ©n retorna la posiciÃ³n actual del efector (`x_ef`), lo que permite al
    maestro comparar y detectar retrasos/desincronizaciones.
    """
    def __init__(self, master_ip='127.0.0.1', port_rx=9001, port_tx=9002):
        self.master_ip = master_ip
        self.port_rx = port_rx
        self.port_tx = port_tx
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.port_rx))
        self.sock.settimeout(0.005)
        self.x_des = np.array([0.55, 0.40])
        self.gripper = True
        self.master_addr = None
        # modo de origen de la referencia recibida: 'MASTER', 'AUTO', 'MANUAL', 'NONE'
        self.source_mode = 'NONE'
        self.last_recv_time = 0.0
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
                parsed = json.loads(data.decode())
                # if master sends an explicit mode, honor it; otherwise treat presence of xd as MASTER
                mode = parsed.get('mode')
                if 'xd' in parsed:
                    xd_arr = np.array(parsed['xd'])
                    # Si el maestro pide SYNC, sÃ³lo ajustar la altura (y)
                    if mode == 'SYNC':
                        try:
                            self.x_des[1] = float(xd_arr[1])
                        except Exception:
                            pass
                        self.source_mode = 'MASTER'
                    else:
                        self.x_des = xd_arr
                        if mode is not None:
                            self.source_mode = parsed['mode']
                        else:
                            self.source_mode = 'MASTER'
                    self.last_recv_time = time.time()
                if 'mode' in parsed:
                    self.source_mode = parsed['mode']
                    self.last_recv_time = time.time()
                if 'gripper' in parsed:
                    self.gripper = bool(parsed['gripper'])
                self.master_addr = addr
            except (socket.timeout, json.JSONDecodeError, AttributeError, ConnectionResetError, OSError):
                pass

    def send_force(self, Fe, contact, x_ef=None, master_port=9002):
        # Enviar fuerza, contacto y opcionalmente la posiciÃ³n del efector
        msg_dict = {'Fe': Fe.tolist(), 'contact': int(bool(contact))}
        if x_ef is not None:
            msg_dict['x_ef'] = x_ef.tolist()
        msg = json.dumps(msg_dict)
        try:
            self.sock.sendto(msg.encode(), (self.master_ip, master_port))
        except Exception:
            pass


class SlaveRobot:
    def __init__(self, master_ip='127.0.0.1'):
        # sincronizar postura inicial con el maestro para evitar diferencias de referencia
        self.q = np.array([0.4, -0.3, 0.2])
        self.dq = np.zeros(3)
        self.contact_model = PegHoleContact()
        self.net = SlaveNetServer(master_ip)
        N = 500
        self.hist_t = np.zeros(N)
        self.hist_tau = np.zeros((N, 3))
        self.hist_Fc = np.zeros((N, 2))
        self.hist_x = np.zeros((N, 2))
        self.hist_ex = np.zeros((N, 2))
        self.hist_phase = np.zeros(N, dtype=int)  # para marcar transiciones de fase
        self.idx = 0
        self.t = 0.0
        self.contact_state = 'APROXIMACION'
        self.state = "APROXIMACION"
        # Keyboard control flags - usar diccionario de booleanos (mÃ¡s robusto)
        self.key_state = {'w': False, 'a': False, 's': False, 'd': False}
        self.user_override = False  # si usuario presiona w/a/s/d entra control manual

        self.F_threshold = 2.0      # umbral de contacto
        self.align_tol = 0.005      # 5 mm tolerancia alineacion

        self.insert_force = np.array([0.0, -5.0])  # fuerza constante hacia abajo
        self.search_amp = 0.01      # 1 cm oscilacion lateral
        self.search_freq = 2.0      # frecuencia de busqueda
        self.Fe = np.zeros(2)
        # Manual reference from keyboard (inicialmente en la posiciÃ³n actual del efector)
        self.x_des_manual = fk_3r(self.q)
        self.x_des_prev = self.x_des_manual.copy()  # para calcular velocidad deseada
        # velocidad mÃ¡xima de movimiento (aplica tanto a control manual como al seguir al maestro)
        self.max_speed = 0.12  # [m/s] â€“ doble de velocidad para mayor respuesta
        self.manual_speed = self.max_speed
        # timeout para decidir que la referencia MASTER dejÃ³ de llegar
        self.net_recv_timeout = 0.25  # s
        # flag para guardar datos sÃ³lo cuando el usuario lo pida
        self.save_requested = False
        
        # MÃ¡quina de estados AUTOMÃTICA (se usa solo para referencia de velocidad pero no se ejecuta)
        self.auto_timer = 0.0
        self.auto_phase = 'APPROACH'  # APPROACH, SEARCH, INSERT, DONE
        self.approach_target = np.array([0.55, 0.35])  # 5cm arriba del agujero
        self.auto_speed = 0.05  # [m/s] velocidad de referencia en automÃ¡tico
        # distancia mÃ¡xima alcanzable (radio) para evitar mandar referencias inalcanzables
        self.max_radius = L1 + L2 + L3 - 0.01
        # Estado de terminaciÃ³n
        self.completion_state = 'ESPERANDO'  # ESPERANDO, COMPLETADO, TERMINADO
        self.task_start_time = None  # inicia al primer contacto real
        self.demo_limit_s = 60.0
        self.completion_elapsed = None
        self.capture_count = 0
        self.prev_contact_state = self.contact_state
        self.last_dist_to_hole = np.inf
        self.last_lateral_error = np.inf
        self.last_depth = 0.0
        self.complete_lateral_tol = INSERT_LATERAL_TOL
        self.complete_depth_target = COMPLETE_DEPTH_TARGET
        self.complete_hold_steps = max(1, int(COMPLETE_HOLD_TIME / DT))
        self.complete_counter = 0

    def ik_dls(self, x_des, damp=0.01):
        q = self.q.copy()
        for _ in range(8):
            e = x_des - fk_3r(q)
            if np.linalg.norm(e) < 1e-4:
                break
            J = jacobian_3r(q)
            Jp = J.T @ np.linalg.inv(J @ J.T + (damp ** 2) * np.eye(2))
            q = q + Jp @ e
        return q

    def update_from_keyboard(self):
        """Process keyboard input to move reference (a/s/w/d) - robusto con booleanos."""
        any_pressed = any(self.key_state.values())
        if any_pressed:
            self.user_override = True
        
        # Desplazamiento manual limitado por max_speed
        v = self.max_speed * DT
        if self.key_state['w']:
            self.x_des_manual[1] += v  # move up
        if self.key_state['s']:
            self.x_des_manual[1] -= v  # move down
        if self.key_state['a']:
            self.x_des_manual[0] -= v  # move left
        if self.key_state['d']:
            self.x_des_manual[0] += v  # move right
        
        # Clamp to workspace
        self.x_des_manual[0] = np.clip(self.x_des_manual[0], -0.8, 0.8)
        self.x_des_manual[1] = np.clip(self.x_des_manual[1], -0.4, 0.95)
        # Evitar referencias fuera del radio alcanzable, similar a la secuencia automÃ¡tica
        r = np.linalg.norm(self.x_des_manual)
        if r > self.max_radius:
            self.x_des_manual = self.x_des_manual * (self.max_radius / r)
    
    def update_auto_sequence(self):
        """MÃ¡quina de estados automÃ¡tica para inserciÃ³n Peg-in-Hole."""
        self.auto_timer += DT
        x_ef = fk_3r(self.q)
        
        if self.auto_phase == 'APPROACH':
            # Fase 1: Acercarse al agujero desde arriba
            # mover referencia suavemente hacia target
            delta = self.approach_target - self.x_des_manual
            step = np.clip(delta, -self.auto_speed*DT, self.auto_speed*DT)
            self.x_des_manual += step
            dist = np.linalg.norm(x_ef - self.approach_target)
            if dist < 0.02:  # 2cm de tolerancia
                self.auto_phase = 'SEARCH'
                self.auto_timer = 0.0
                print('Auto: fase SEARCH iniciada')
        
        elif self.auto_phase == 'SEARCH':
            # Fase 2: Buscar alineaciÃ³n en X,Y (oscilar ligeramente)
            offset = self.search_amp * np.sin(2 * np.pi * self.search_freq * self.auto_timer)
            target = np.array([HOLE_CENTER[0] + offset, HOLE_CENTER[1] + 0.08])
            delta = target - self.x_des_manual
            step = np.clip(delta, -self.auto_speed*DT, self.auto_speed*DT)
            self.x_des_manual += step
            if self.auto_timer > 2.0 or np.linalg.norm(self.Fe) > self.F_threshold:
                # O bien pasÃ³ el tiempo de bÃºsqueda, o hizo contacto
                self.auto_phase = 'INSERT'
                self.auto_timer = 0.0
        
        elif self.auto_phase == 'INSERT':
            # Fase 3: Insertar profundamente
            insert_depth = min(self.auto_timer * 0.1, PEG_LENGTH * 0.9)  # insertar gradualmente
            target = np.array([HOLE_CENTER[0], HOLE_CENTER[1] - insert_depth])
            delta = target - self.x_des_manual
            step = np.clip(delta, -self.auto_speed*DT, self.auto_speed*DT)
            self.x_des_manual += step
            if self.contact_state == 'COMPLETADO' or self.auto_timer > 5.0:
                self.auto_phase = 'DONE'
                self.auto_timer = 0.0
                print('Auto: fase DONE (completado)')
        
        elif self.auto_phase == 'DONE':
            # Fase 4: Mantener posiciÃ³n
            self.x_des_manual = np.array([HOLE_CENTER[0], HOLE_CENTER[1] - PEG_LENGTH * 0.85])
        # al final de cada actualizaciÃ³n, asegurar referencia alcanzable
        r = np.linalg.norm(self.x_des_manual)
        if r > self.max_radius:
            self.x_des_manual = self.x_des_manual * (self.max_radius / r)


    def step(self):
        """
        Un paso de simulacion/integracion (llamado periÃ³dicamente).
        """
        # Update keyboard input
        self.update_from_keyboard()        
        # DESACTIVAR LÃ“GICA AUTOMÃTICA: el esclavo funciona sÃ³lo en MANUAL o siguiendo al MASTER.
        # No ejecutar la secuencia automÃ¡tica bajo ninguna circunstancia.
        
        # SelecciÃ³n autorizada de la referencia cartesiana (prioridad: MANUAL > MASTER > AUTO)
        if self.user_override:
            # Usuario controla con teclado
            x_des = self.x_des_manual.copy()
        else:
            # Seguir al MASTER sÃ³lo si hay mensajes recientes; en caso contrario, mantener la Ãºltima referencia (HOLD)
            master_mode = getattr(self.net, 'source_mode', 'NONE')
            last_recv = getattr(self.net, 'last_recv_time', 0.0)
            if master_mode == 'MASTER' and (time.time() - last_recv < getattr(self, 'net_recv_timeout', 0.25)):
                x_des = self.net.x_des.copy()
                # limitar referencia recibida al espacio alcanzable
                r = np.linalg.norm(x_des)
                max_r = L1 + L2 + L3 - 0.01
                if r > max_r:
                    x_des = x_des * (max_r / r)
            else:
                # mantener la referencia previa (no entrar en automÃ¡tico)
                x_des = self.x_des_prev.copy() if hasattr(self, 'x_des_prev') else fk_3r(self.q)
        # calcular velocidad deseada a partir de variaciÃ³n de referencia
        try:
            raw_dx = (x_des - self.x_des_prev) / DT
        except AttributeError:
            raw_dx = np.zeros(2)
        # limitar velocidad deseada para evitar saltos bruscos
        max_v = self.max_speed
        if np.linalg.norm(raw_dx) > max_v:
            raw_dx = raw_dx * (max_v / np.linalg.norm(raw_dx))
        dx_des = raw_dx
        self.x_des_prev = x_des.copy()

        # 2) Cinematica directa: posicion actual del efector
        x_ef = fk_3r(self.q)
        
        # 3) Obtener velocidad del efector aproximada (v = J * dq)
        J = jacobian_3r(self.q)            # 2x3
        v_ef = J @ self.dq                 # 2-vector
        
        # 4) Calcular fuerza de contacto
        F_contact, state_str, in_contact = self.contact_model.compute_contact_force(x_ef)
        self.Fe = np.array(F_contact, dtype=float)
        self.contact_state = state_str
        self.last_dist_to_hole = float(np.linalg.norm(x_ef - HOLE_CENTER))
        self.last_lateral_error = abs(float(x_ef[0] - HOLE_CENTER[0]))
        self.last_depth = max(0.0, float(HOLE_CENTER[1] - x_ef[1]))
        if self.contact_state != self.prev_contact_state:
            print(f"Transicion estado: {self.prev_contact_state} -> {self.contact_state}")
            self.prev_contact_state = self.contact_state

        # Iniciar cronometro cuando realmente empieza la tarea de insercion (contacto)
        if self.task_start_time is None and self.contact_state in ('CONTACTO', 'INSERCION'):
            self.task_start_time = time.time()
            print('Timer de insercion iniciado (primer contacto).')

        # Confirmacion robusta de COMPLETADO (evita picos instantaneos)
        inserted_now = (
            self.last_lateral_error <= self.complete_lateral_tol
            and self.last_depth >= self.complete_depth_target
        )
        if inserted_now:
            self.complete_counter += 1
        elif self.completion_state == 'ESPERANDO':
            self.complete_counter = 0

        if self.completion_state == 'ESPERANDO' and self.complete_counter >= self.complete_hold_steps:
            self.completion_state = 'COMPLETADO'
            t0 = self.task_start_time if self.task_start_time is not None else time.time()
            self.completion_elapsed = time.time() - t0
            status = 'OK < 60 s' if self.completion_elapsed <= self.demo_limit_s else 'FUERA DE TIEMPO'
            self.contact_state = 'COMPLETADO'
            print(f"Estado: COMPLETADO | t={self.completion_elapsed:.2f}s | {status}")

        if self.completion_state == 'COMPLETADO':
            self.contact_state = 'COMPLETADO'

        # 5) CONTROL DE IMPEDANCIA (cartesiano -> torque/joint)
        result = impedance_control(self.q, self.dq, x_des, dx_des, F_contact)

        # impedance_control returns (tau, F_total, e_x)
        tau_imp_raw = result[0]
        F_total = result[1] if len(result) > 1 else None
        e_x = result[2] if len(result) > 2 else np.zeros(2)

        # Convert to torques if needed (fallback defensivo)
        tau = np.asarray(tau_imp_raw)
        if tau.size == 2:
            tau = J.T @ tau
        elif tau.size != 3:
            tau = np.zeros(3)
        tau = np.clip(tau, -30.0, 30.0)  # permitir un rango mayor para seguimiento preciso

        # 7) INTEGRAR DINAMICA
        self.q, self.dq = integrate_dynamics(self.q, self.dq, tau)

        # 8) ENVIAR/REGISTRAR DATOS (incluye x_ef para sincronizaciÃ³n)
        try:
            self.net.send_force(self.Fe, bool(in_contact), x_ef)
        except Exception:
            pass

        # Guardar historicos con indices consistentes
        i = self.idx % 500
        self.hist_t[i] = self.t
        self.hist_tau[i] = tau
        self.hist_Fc[i] = self.Fe
        self.hist_x[i] = x_ef
        self.hist_ex[i] = e_x if e_x is not None else np.zeros(2)
        self.hist_phase[i] = self.contact_model.phase
        
        self.idx += 1
        self.t += DT


def setup_slave_plots(robot):
    fig = plt.figure(figsize=(14, 10), facecolor='#0d1117')
    fig.suptitle('TE3001B Robot Esclavo 3R | Control de Impedancia + Peg-in-Hole', color='white', fontsize=14, fontweight='bold', y=0.98)
    bg = '#0d1117'
    C = ['#FF6B6B', '#69FF47', '#00BFFF', '#FFD700', '#FF69B4', '#00FFD0']
    ax_robot = fig.add_subplot(2, 2, 1, facecolor=bg)
    ax_force = fig.add_subplot(2, 2, 2, facecolor=bg)
    ax_tau = fig.add_subplot(2, 2, 3, facecolor=bg)
    ax_err = fig.add_subplot(2, 2, 4, facecolor=bg)
    for ax in [ax_robot, ax_force, ax_tau, ax_err]:
        ax.tick_params(colors='#aaa')
        for spine in ax.spines.values():
            spine.set_edgecolor('#333')
        ax.grid(True, color='#1e2530', linestyle='--', alpha=0.5)
        ax.title.set_color('white')
        ax.xaxis.label.set_color('#aaa'); ax.yaxis.label.set_color('#aaa')

    ax_robot.set_xlim(-0.9, 0.9); ax_robot.set_ylim(-0.5, 1.0)
    ax_robot.set_aspect('equal')
    ax_robot.set_title('Esclavo Peg-in-Hole', fontsize=11)
    ax_robot.set_xlabel('x [m]'); ax_robot.set_ylabel('y [m]')
    hole_patch = plt.Rectangle((HOLE_CENTER[0] - 0.04, HOLE_CENTER[1] - 0.005), 0.08, 0.04, color='#2a2a4a', zorder=1)
    ax_robot.add_patch(hole_patch)
    ax_robot.plot(HOLE_CENTER[0], HOLE_CENTER[1], 'x', color='#FFD700', markersize=10, markeredgewidth=2, zorder=3)
    ax_robot.text(HOLE_CENTER[0] + 0.02, HOLE_CENTER[1] + 0.015, 'HOLE', color='#FFD700', fontsize=8)
    link_line, = ax_robot.plot([], [], 'o-', color=C[2], linewidth=3, markersize=8, markerfacecolor=C[0])
    peg_line, = ax_robot.plot([], [], '-', color=C[3], linewidth=5, zorder=4)
    state_text = ax_robot.text(0.02, 0.96, '', transform=ax_robot.transAxes, color='#FFD700', fontsize=10, fontweight='bold', verticalalignment='top')
    force_arrow = ax_robot.annotate('', xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='red', lw=2.5), zorder=5)

    ax_force.set_title('Fuerzas de Contacto [N]', fontsize=11)
    ax_force.set_xlabel('Tiempo [s]'); ax_force.set_ylabel('F [N]')
    line_Fx, = ax_force.plot([], [], color=C[4], linewidth=2.0, label='Fx contacto')
    line_Fy, = ax_force.plot([], [], color=C[5], linewidth=2.0, label='Fy contacto')
    ax_force.axhline(y=0, color='#444', linewidth=0.8)
    ax_force.axhline(y=F_THRESHOLD, color='#FF4444', linewidth=1.2, linestyle='--', label=f'Umbral {F_THRESHOLD} N')
    ax_force.legend(loc='upper right', fontsize=8, facecolor='#1a1a2e')

    ax_tau.set_title('Torques Articulares [Nm]', fontsize=11)
    ax_tau.set_xlabel('Tiempo [s]'); ax_tau.set_ylabel('[Nm]')
    lines_tau = [ax_tau.plot([], [], color=C[i], linewidth=1.5, label=f'{i+1}')[0] for i in range(3)]
    ax_tau.axhline(y=0, color='#444', linewidth=0.8)
    ax_tau.legend(loc='upper right', fontsize=8, facecolor='#1a1a2e')

    ax_err.set_title('Error Cartesiano |e| [mm]', fontsize=11)
    ax_err.set_xlabel('Tiempo [s]'); ax_err.set_ylabel('Error [mm]')
    line_ex, = ax_err.plot([], [], color=C[0], linewidth=1.8, label='|e|')
    line_ey, = ax_err.plot([], [], color=C[1], linewidth=1.8, label='|e_y|')
    ax_err.axhline(y=1.0, color='#888', linewidth=1.0, linestyle=':', label='1 mm (meta)')
    ax_err.legend(loc='upper right', fontsize=8, facecolor='#1a1a2e')

    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    return (fig, (ax_robot, ax_force, ax_tau, ax_err), (link_line, peg_line, state_text, force_arrow), (line_Fx, line_Fy), lines_tau, (line_ex, line_ey))


def main(master_ip):
    robot = SlaveRobot(master_ip)
    (fig, axes, robot_artists, force_lines, lines_tau, err_lines) = setup_slave_plots(robot)
    ax_robot, ax_force, ax_tau, ax_err = axes
    link_line, peg_line, state_text, force_arrow = robot_artists
    line_Fx, line_Fy = force_lines
    line_ex, line_ey = err_lines

    running = [True]

    def sim_loop():
        while running[0]:
            robot.step()
            time.sleep(DT)

    threading.Thread(target=sim_loop, daemon=True).start()

    def on_close(event):
        try:
            fig.savefig('slave_snapshot.png')
        except Exception:
            pass
        # guardar sÃ³lo si el usuario pidiÃ³ explÃ­citamente con la tecla de salida
        if robot.save_requested:
            save_history(robot, 'slave')
        else:
            print("Cierre sin guardar historial; presione 'p' para exportar datos antes de cerrar")
    fig.canvas.mpl_connect('close_event', on_close)

    def animate(frame):
        if robot.idx < 500:
            idx = np.arange(robot.idx)
        else:
            i0 = robot.idx % 500
            idx = np.arange(i0, i0 + 500) % 500
        n = len(idx)
        if n == 0:
            return ([link_line, peg_line] + lines_tau + [line_Fx, line_Fy, line_ex, line_ey])
        t = robot.hist_t[idx]
        tau = robot.hist_tau[idx]
        Fc = robot.hist_Fc[idx]
        ex = robot.hist_ex[idx]
        t_win = 5.0
        mask = (t > robot.t - t_win)

        pts = fk_3r_full(robot.q)
        link_line.set_data(pts[:, 0], pts[:, 1])
        ef = pts[-1]
        peg_dir = pts[-1] - pts[-2]
        if np.linalg.norm(peg_dir) > 0:
            peg_dir = peg_dir / np.linalg.norm(peg_dir)
        peg_start = ef
        peg_end = ef + peg_dir * PEG_LENGTH
        peg_line.set_data([peg_start[0], peg_end[0]], [peg_start[1], peg_end[1]])
        # Mostrar modo: MANUAL, MASTER (si mensajes recientes) o HOLD
        if robot.user_override:
            mode_str = 'MANUAL'
        else:
            last_recv = getattr(robot.net, 'last_recv_time', 0.0)
            if getattr(robot.net, 'source_mode', 'NONE') == 'MASTER' and (time.time() - last_recv < getattr(robot, 'net_recv_timeout', 0.25)):
                mode_str = 'MASTER'
            else:
                mode_str = 'HOLD'
        sat_warning = ''
        # detectar saturacion de torque reciente
        if np.any(np.abs(robot.hist_tau[(robot.idx-1)%500]) >= 19.9):
            sat_warning = ' | TORQUE SAT'
        # Criterio COMPLETADO robusto
        criteria = f' | ex={robot.last_lateral_error*1000:.1f}mm<{robot.complete_lateral_tol*1000:.1f} | z={robot.last_depth*1000:.1f}mm>{robot.complete_depth_target*1000:.1f}'
        if robot.task_start_time is None:
            time_info = ' | t_task=--/60s'
        else:
            elapsed_task = time.time() - robot.task_start_time
            time_info = f' | t_task={elapsed_task:.1f}/{robot.demo_limit_s:.0f}s'

        completion_info = ''
        if robot.completion_state == 'COMPLETADO':
            completion_info = ' | COMPLETADO! Presiona t para terminar'
        elif robot.completion_state == 'TERMINADO':
            completion_info = ' | [TERMINADO]'
        state_text.set_text(f"Estado: {robot.contact_state} | {mode_str}{criteria}{time_info}{completion_info}{sat_warning}")
        # actualizar suptitle con modo actual
        fig.suptitle(f"TE3001B Robot Esclavo 3R | Modo: {mode_str}", color='white', fontsize=14, fontweight='bold', y=0.98)
        f_norm = np.linalg.norm(robot.Fe)
        force_arrow.set_visible(f_norm > 1e-3)
        force_arrow.set_position((ef[0], ef[1]))
        force_arrow.xy = (ef[0] + 0.02 * robot.Fe[0], ef[1] + 0.02 * robot.Fe[1])

        ax_force.set_xlim(max(0, robot.t - t_win), max(t_win, robot.t))
        line_Fx.set_data(t[mask], Fc[mask, 0])
        line_Fy.set_data(t[mask], Fc[mask, 1])
        ax_force.relim(); ax_force.autoscale_view(scalex=False)

        ax_tau.set_xlim(max(0, robot.t - t_win), max(t_win, robot.t))
        for i, ln in enumerate(lines_tau):
            ln.set_data(t[mask], tau[mask, i])
        ax_tau.relim(); ax_tau.autoscale_view(scalex=False)

        ax_err.set_xlim(max(0, robot.t - t_win), max(t_win, robot.t))
        line_ex.set_data(t[mask], np.abs(ex[mask, 0]) * 1000)
        line_ey.set_data(t[mask], np.abs(ex[mask, 1]) * 1000)
        ax_err.relim(); ax_err.autoscale_view(scalex=False)

        return ([link_line, peg_line] + lines_tau + [line_Fx, line_Fy, line_ex, line_ey])

    ani = animation.FuncAnimation(fig, animate, interval=50, blit=False, cache_frame_data=False)

    def on_key_press(event):
        key = (event.key or '').lower()
        if key in robot.key_state:
            robot.key_state[key] = True
        elif key == 'r':
            # reinicio completo de la tarea
            robot.user_override = False
            robot.auto_phase = 'APPROACH'
            robot.auto_timer = 0.0
            robot.completion_state = 'ESPERANDO'
            robot.completion_elapsed = None
            robot.task_start_time = None
            robot.complete_counter = 0
            robot.contact_model.phase = robot.contact_model.APPROACH
            robot.contact_state = 'APROXIMACION'
            robot.prev_contact_state = robot.contact_state
            print('Tarea reiniciada')
        elif key == 'c':
            robot.capture_count += 1
            tag = build_capture_tag(robot.capture_count)
            snap_name = f'slave_snapshot_{tag}.png'
            hist_prefix = f'slave_{tag}'
            try:
                fig.savefig(snap_name)
                save_history(robot, hist_prefix)
                print(f"Captura guardada: {snap_name} + {hist_prefix}_history.png")
            except Exception as ex:
                print(f"No se pudo guardar captura: {ex}")
        elif key == 'p':
            # Cerrar y guardar con 'p'
            robot.save_requested = True
            running[0] = False
            plt.close()

        # No finalizar automaticamente: usar 't' para pasar a TERMINADO.
        if robot.completion_state == 'COMPLETADO' and key == 't':
            robot.completion_state = 'TERMINADO'
            print('Estado: TERMINADO')

    def on_key_release(event):
        key = (event.key or '').lower()
        if key in robot.key_state:
            robot.key_state[key] = False
            if not any(robot.key_state.values()):
                robot.user_override = False

    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.canvas.mpl_connect('key_release_event', on_key_release)

    plt.show()


def save_history(robot, prefix):
    t = robot.hist_t[: robot.idx if robot.idx < robot.hist_t.size else robot.hist_t.size]
    tau = robot.hist_tau[: t.size]
    Fc = robot.hist_Fc[: t.size]
    x = robot.hist_x[: t.size]
    ex = robot.hist_ex[: t.size]

    fig2, axs = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs.flatten()
    axs[0].plot(t, Fc[:, 0], label='Fx'); axs[0].plot(t, Fc[:, 1], label='Fy')
    axs[0].set_title('Fuerzas de contacto'); axs[0].set_xlabel('t [s]'); axs[0].set_ylabel('F [N]')
    axs[0].legend()

    axs[1].plot(t, tau[:, 0], label='Ï„1'); axs[1].plot(t, tau[:, 1], label='Ï„2'); axs[1].plot(t, tau[:, 2], label='Ï„3')
    axs[1].set_title('Torques articulaires'); axs[1].set_xlabel('t [s]'); axs[1].set_ylabel('Ï„ [Nm]')
    axs[1].legend()

    axs[2].plot(t, x[:, 0], label='x'); axs[2].plot(t, x[:, 1], label='y')
    axs[2].set_title('PosiciÃ³n efector'); axs[2].set_xlabel('t [s]'); axs[2].set_ylabel('[m]')
    axs[2].legend()

    axs[3].plot(t, np.abs(ex[:, 0]) * 1000, label='|e_x| [mm]'); axs[3].plot(t, np.abs(ex[:, 1]) * 1000, label='|e_y| [mm]')
    axs[3].set_title('Error cartesiano'); axs[3].set_xlabel('t [s]'); axs[3].set_ylabel('[mm]')
    axs[3].legend()

    plt.tight_layout()
    try:
        fig2.savefig(f'{prefix}_history.png')
    except Exception:
        pass
    plt.close(fig2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='TE3001B Robot Esclavo')
    parser.add_argument('--master-ip', default='127.0.0.1', help='IP del PC maestro (default: loopback)')
    args = parser.parse_args()
    main(args.master_ip)

