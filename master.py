"""

ROBOT MAESTRO TE3001B Peg -in - Hole Teleoperado
Simulaci Ã³n 3- DOF + Computed Torque + Graficaci Ã³n
Prof . Alberto MuÃ±oz Computational Robotics Lab
Tec de Monterrey , 2026


Ejecutar en la PC MAESTRO :
python3 master_robot .py --slave -ip <IP_DEL_ESCLAVO >

Controles :
W/S mover efector final en +y/-y
A/D mover efector final en -x/+x
Q/E abrir / cerrar pinza
P salir y guardar datos
"""

import numpy as np
import matplotlib
matplotlib.use ('TkAgg') # cambiar a â€™Qt5Agg â€™ si TkAgg falla
# quitar atajos por defecto que usarÃ­an 'q' o 's' (cierre/guardado)
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
# PAR Ã METROS DEL ROBOT 3R
L1 , L2 , L3 = 0.35 , 0.30 , 0.20 # longitudes eslabones [m]
M1 , M2 , M3 = 1.5 , 1.0 , 0.5 # masas de eslabones [kg]
G_GRAV = 9.81 # aceleraci Ã³n gravitacional [m/s ^2]

# Ganancias del Computed Torque
KP = np. diag ([120.0 , 100.0 , 80.0]) # rigidez articular
KV = np. diag ([25.0 , 20.0 , 15.0]) # amortiguamiento

# Paso de integraci Ã³n
DT = 0.01 # 100 Hz

# velocidad mÃ¡ xima de referencia cartesiana (m/s)
MAX_CART_SPEED = 0.10  # velocidad doble para mayor respuesta
FORCE_GUIDED_DEMO = False

def build_capture_tag(counter):
    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    return f'{ts}_{counter:03d}'

#
# CINEM Ã TICA DIRECTA 3R PLANAR
#
def fk_3r ( q) :
    """
    Cinem Ã¡ tica directa para robot 3R planar .

    Args :
    q (np. ndarray ): Vector de Ã¡ ngulos articulares [q1 , q2 , q3] en
    radianes .

    Returns :
    tuple : (x, y) posici Ã³n del efector final en metros .
    """
    q1 , q2 , q3 = q
    # Posiciones de cada articulaci Ã³n (Ãº til para visualizaci Ã³n)
    x1 = L1 * np. cos ( q1 )
    y1 = L1 * np. sin ( q1 )
    x2 = x1 + L2 * np. cos ( q1 + q2 )
    y2 = y1 + L2 * np. sin ( q1 + q2 )
    x3 = x2 + L3 * np. cos ( q1 + q2 + q3 )
    y3 = y2 + L3 * np. sin ( q1 + q2 + q3 )
    return np. array ([ x3 , y3 ])


def fk_3r_full ( q) :
    """
    Retorna las posiciones de todas las articulaciones y el EF.

    Returns :
    np. ndarray : Matriz (4 x2) con posiciones [base , j1 , j2 , EF ].
    """


    q1 , q2 , q3 = q
    p0 = np. array ([0.0 , 0.0])
    p1 = np. array ([ L1 *np. cos ( q1 ) , L1 *np. sin ( q1 ) ])
    p2 = p1 + np. array ([ L2 *np. cos ( q1 + q2 ) , L2 *np. sin ( q1 + q2 ) ])
    p3 = p2 + np. array ([ L3 *np. cos ( q1 + q2 + q3 ) , L3 *np. sin ( q1 + q2 + q3 ) ])
    return np. array ([ p0 , p1 , p2 , p3 ])


#
# JACOBIANO ANAL Ã TICO 3R PLANAR (J R ^{2 3 })
#
def jacobian_3r (q ):
    """
    Jacobiano anal Ã­ tico del robot 3R planar .
    Relaciona velocidades articulares con velocidad cartesiana del EF:
    = J(q) q

    Args :
    q (np. ndarray ): Vector de Ã¡ ngulos articulares [q1 , q2 , q3 ].

    Returns :
    np. ndarray : Matriz Jacobiana 2 3 .
    """
    q1 , q2 , q3 = q
    s1 = np. sin ( q1 )
    s12 = np. sin ( q1 + q2 )
    s123 = np. sin ( q1 + q2 + q3 )
    c1 = np. cos ( q1 )
    c12 = np. cos ( q1 + q2 )
    c123 = np. cos ( q1 + q2 + q3 )

    J = np. array ([
    [- L1 * s1 - L2 * s12 - L3 * s123 ,
    -L2 * s12 - L3 * s123 ,
    -L3 * s123 ],
    [ L1 * c1 + L2 * c12 + L3 * c123 ,
    L2 * c12 + L3 * c123 ,
    L3 * c123 ]
    ])
    return J


#
# MODELO DINÃ MICO SIMPLIFICADO ( punto de masa )
#


def inertia_matrix (q ):
    """
    Matriz de inercia M(q) para robot 3R ( modelo punto de masa al
    extremo ).
    Propiedad : sim Ã© trica y positiva definida en todo el espacio de
    trabajo .

    Args :
    q (np. ndarray ): Ã ngulos articulares .

    Returns :
    np. ndarray : Matriz de inercia 3 3 .
    """
    q1 , q2 , q3 = q
    c2 = np. cos ( q2 )
    c3 = np. cos ( q3 )
    c23 = np. cos ( q2 + q3 )

    # TÃ© rminos dominantes ( modelo simplificado punto de masa )
    m11 = ( M1 * L1 **2 + M2 *( L1 **2 + L2 **2 + 2* L1 * L2 * c2 ) +
    M3 *( L1 **2 + L2 **2 + L3 **2 + 2* L1 * L2 * c2 +
    2* L1 * L3 * c23 + 2* L2 * L3 * c3 ))
    m12 = ( M2 *( L2 **2 + L1 * L2 * c2 ) +
    M3 *( L2 **2 + L3 **2 + L1 * L2 * c2 + L1 * L3 * c23 + 2* L2 * L3 * c3 ))
    m13 = M3 *( L3 **2 + L1 * L3 * c23 + L2 * L3 * c3 )
    m22 = M2 * L2 **2 + M3 *( L2 **2 + L3 **2 + 2* L2 * L3 * c3 )
    m23 = M3 *( L3 **2 + L2 * L3 * c3 )
    m33 = M3 * L3 **2

    M = np. array ([[ m11 , m12 , m13 ],
    [ m12 , m22 , m23 ],
    [ m13 , m23 , m33 ]])
    return M


def coriolis_matrix (q , dq ):
    """
    Matriz de Coriolis y fuerzas centr Ã­ fugas C(q, q ).
    Calculada via diferenciaci Ã³n num Ã© rica de M(q) (mÃ© todo Christoffel ).

    Args :
    q (np. ndarray ): Ã ngulos articulares .
    dq (np. ndarray ): Velocidades articulares .

    Returns :
    np. ndarray : Matriz C 3 3 tal que C* q son las fuerzas de
    Coriolis .


    """
    eps = 1e-5
    n = len( q)
    M0 = inertia_matrix (q )
    C = np. zeros (( n , n) )

    # SÃ­ mbolos de Christoffel ( simplificados , solo tÃ© rminos dominantes )
    for k in range (n ):
        qp = q . copy () ; qp [k] += eps
        qm = q . copy () ; qm [k] -= eps
        dM_dk = ( inertia_matrix ( qp ) - inertia_matrix ( qm ) ) / (2* eps )
        C += 0.5 * dM_dk * dq [k]

    return C


def gravity_vector (q ):
    """
    Vector de par gravitacional g(q) para robot 3R planar .
    Asume que la gravedad act Ãºa en direcci Ã³n -y del plano .

    Args :
    q (np. ndarray ): Ã ngulos articulares .

    Returns :
    np. ndarray : Vector de pares gravitacionales 3 1 [Nm ].
    """
    q1 , q2 , q3 = q
    c1 = np. cos ( q1 )
    c12 = np. cos ( q1 + q2 )
    c123 = np. cos ( q1 + q2 + q3 )

    g1 = G_GRAV * (( M1 + M2 + M3 ) * L1 * c1 + ( M2 + M3 )* L2 * c12 + M3 * L3 * c123 )
    g2 = G_GRAV * (( M2 + M3 )* L2 * c12 + M3 * L3 * c123 )
    g3 = G_GRAV * M3 * L3 * c123
    return np. array ([ g1 , g2 , g3 ])


#
# COMPUTED TORQUE CONTROLLER
#
def computed_torque (q , dq , q_des , dq_des , ddq_des ,kp =KP , kv = KV , F_ext = None ):
    """
    Ley de control de Computed Torque ( cancelaci Ã³n exacta de
    no - linealidades ).



    Control : = M(q) a + C(q, q ) q + g(q)
    donde : a = q + Kv( q - q ) + Kp( q - q)

    El sistema en lazo cerrado se reduce a: + K v + K p e = 0

    Args :
    q (np. ndarray ): Posici Ã³n articular actual [ rad ].
    dq (np. ndarray ): Velocidad articular actual [ rad /s].
    q_des (np. ndarray ): Posici Ã³n articular deseada [ rad ].
    dq_des (np. ndarray ): Velocidad articular deseada [ rad /s].
    ddq_des (np. ndarray ): Aceleraci Ã³n articular deseada [ rad / s ].
    kp (np. ndarray ): Ganancia proporcional ( matriz diagonal ).
    kv (np. ndarray ): Ganancia derivativa ( matriz diagonal ).
    F_ext (np. ndarray ): Fuerza externa en EF para feedback
    hÃ¡ ptico [N].

    Returns :
    tuple : (tau , e, de) torques [Nm], error posici Ã³n, error
    velocidad .
    """
    # Error de seguimiento
    e = q_des - q
    de = dq_des - dq

    # Aceleraci Ã³n de referencia ( control PD en espacio articular )
    a_d = ddq_des + kv @ de + kp @ e

    # Matrices din Ã¡ micas
    M_mat = inertia_matrix (q )
    C_mat = coriolis_matrix (q , dq )
    g_vec = gravity_vector (q )

    # Torque de control ( cancela no - linealidades )
    tau = M_mat @ a_d + C_mat @ dq + g_vec

    # Agregar feedback hÃ¡ ptico si hay fuerza externa medida
    if F_ext is not None and np. linalg . norm ( F_ext ) > 0.01:
        J = jacobian_3r ( q)
        tau += J .T @ F_ext # _fb = J F

    # Saturar torques para seguridad
    tau = np. clip ( tau , -20.0 , 20.0)
    return tau , e , de


#


# INTEGRADOR EULER (din Ã¡ mica del robot )
#
def integrate_dynamics (q , dq , tau , dt = DT ):
    """
    Integra la din Ã¡ mica del robot usando Euler expl Ã­ cito .
    Ecuaci Ã³n: M(q) q = - C(q, q ) q - g(q)

    Args :
    q (np. ndarray ): Posici Ã³n articular actual .
    dq (np. ndarray ): Velocidad articular actual .
    tau (np. ndarray ): Vector de torques aplicados .
    dt ( float ): Paso de integraci Ã³n [s].

    Returns :
    tuple : (q_new , dq_new ) nueva posici Ã³n y velocidad articular .
    """
    M_mat = inertia_matrix (q )
    C_mat = coriolis_matrix (q , dq )
    g_vec = gravity_vector (q )

    # Aceleraci Ã³n articular resultante
    ddq = np. linalg . solve ( M_mat , tau - C_mat @ dq - g_vec )

    # Integraci Ã³n Euler
    dq_new = dq + ddq * dt
    q_new = q + dq_new * dt

    # Aplicar lÃ­ mites articulares
    q_limits = np. array ([ np. pi /2 , 2* np. pi /3 , np. pi /2])
    q_new = np. clip ( q_new , - q_limits , q_limits )
    dq_new = np. clip ( dq_new , -3.0 , 3.0) # lÃ­ mite de velocidad [ rad /s]
    return q_new , dq_new


#
# COMUNICACI Ã“N DE RED MAESTRO ( CLIENTE )
#
class MasterNetClient :
    """
    Cliente UDP del maestro . Env Ã­a posici Ã³n cartesiana al esclavo
    y recibe fuerzas de contacto para feedback hÃ¡ ptico .

    Protocolo JSON : {" xd ": [x, y], " gripper ": bool }
    Respuesta : {" Fe ": [Fx , Fy] , " contact ": bool }
    """


    def __init__ (self , slave_ip , port_tx =9001 , port_rx =9002) :
        self . slave_ip = slave_ip
        self . port_tx = port_tx
        self . port_rx = port_rx
        self . sock_tx = socket . socket ( socket . AF_INET , socket . SOCK_DGRAM )
        self . sock_rx = socket . socket ( socket . AF_INET , socket . SOCK_DGRAM )
        self . sock_rx . bind (( '', self . port_rx ))
        self . sock_rx . settimeout (0.005) # no bloquear el loop decontrol
        self . Fe = np. zeros (2) # fuerza recibida del esclavo [N]
        self . contact = False
        self . x_slave = None         # posiciÃ³n del efector del esclavo
        self . last_recv_time = 0.0   # tiempo del Ãºltimo mensaje recibido
        self . _thread = threading . Thread ( target = self . _recv_loop , daemon = True )
        self . _thread . start ()

    def send_command (self , xd , gripper = True ):
        """ Env Ã­a posici Ã³n deseada al esclavo .
        Soporta un campo opcional `mode` para operaciones especiales (por ejemplo 'SYNC').
        """
        # mantener compatibilidad: aceptar modo opcional pasado via kwargs
        # Construir mensaje de forma segura
        if isinstance(xd, np.ndarray):
            xd_list = xd.tolist()
        else:
            xd_list = list(xd)
        msg_dict = {"xd": xd_list, "gripper": int(bool(gripper))}
        # Si el llamador proporcionÃ³ un modo en atributos (por conveniencia), extraerlo
        mode = None
        # algunos call-sites pueden pasar un third arg; pero mantenemos la API simple
        # El caller puede usar send_command(xd, gripper=..., mode='SYNC')
        # sin romper el envio tradicional.
        try:
            # si se pasÃ³ un atributo 'mode' en los kwargs del caller, estarÃ¡ presente como atributo
            mode = getattr(self, '_pending_mode', None)
        except Exception:
            mode = None
        # En caso de que el caller quiera enviar modo explÃ­cito, le damos una forma directa: setear
        # un atributo temporal antes de la llamada (usado abajo por la tecla 'y').
        if mode is not None:
            msg_dict['mode'] = mode
            try:
                delattr(self, '_pending_mode')
            except Exception:
                pass
        msg = json.dumps(msg_dict)
        self . sock_tx . sendto ( msg . encode () , ( self . slave_ip , self . port_tx ) )

    def _recv_loop ( self ) :
        """ Hilo receptor actualiza Fe y estado de contacto . """
        while True :
            try :
                data , _ = self . sock_rx . recvfrom (256)
                parsed = json . loads ( data . decode () )
                parsed_norm = {
                    (k . strip () if isinstance ( k , str ) else k ) : v
                    for k , v in parsed . items ()
                }

                fe_value = parsed_norm . get ("Fe")
                if fe_value is not None :
                    self . Fe = np. array ( fe_value )

                contact_value = parsed_norm . get ("contact")
                if contact_value is not None :
                    self . contact = bool ( contact_value )
                # posiciÃ³n del efector reportada por el esclavo
                xef = parsed_norm.get('x_ef')
                if xef is not None :
                    try:
                        self.x_slave = np.array(xef)
                    except Exception:
                        pass
                self.last_recv_time = time.time()
            except ( socket . timeout , json . JSONDecodeError , KeyError , TypeError , ValueError ) :
                pass


#
# CLASE PRINCIPAL ROBOT MAESTRO
#
class MasterRobot :
    """
    Simulador completo del robot maestro con :
    - DinÃ¡ mica 3R con Computed Torque
    - Control cartesiano por teclado
    - Feedback hÃ¡ ptico desde el esclavo
    - Graficaci Ã³n en tiempo real (4 subplots )
    - Registro de datos para anÃ¡ lisis
    """
    def __init__ (self , slave_ip =" 127.0.0.1 ") :


        # Estado inicial del robot
        self .q = np. array ([0.4 , -0.3 , 0.2]) # posici Ã³n articular [rad ]
        self . dq = np. zeros (3) # velocidad articular
        self . q_des = self . q. copy ()
        self . dq_des = np. zeros (3)
        self . ddq_des = np. zeros (3)

        # Velocidad de movimiento cartesiano ( controlado por teclado )
        self . v_cart = np. zeros (2)
        # lÃ­mite realista para el paso por tecla
        self. max_speed = MAX_CART_SPEED
        self . v_step = self. max_speed

        # Forzar modo manual: sÃ³lo actualizar referencia cuando el usuario pulse teclas
        self.manual_only = True
        # referencia cartesiana deseada (persistente entre pasos)
        self.x_des = fk_3r(self . q)

        # Red
        self . net = MasterNetClient ( slave_ip )
        # flag para guardar datos sÃ³lo cuando el usuario lo pida
        self.save_requested = False
        # Modo de sincronizaciÃ³n: si True, maestro envÃ­a su altura EF al esclavo
        self.sync_mode = False

        # Hist Ã³ rico para graficaci Ã³n ( buffer circular de 500 muestras )
        N = 500
        self . hist_t = np. zeros ( N)
        self . hist_q = np. zeros ((N , 3) )
        self . hist_tau = np. zeros (( N , 3) )
        self . hist_Fe = np. zeros ((N , 2) )
        self . hist_x = np. zeros ((N , 2) )
        self . idx = 0
        self .t = 0.0

        # Estado de la pinza
        self . gripper_open = False

        # Referencia cinem Ã¡ tica inversa ( DLS )
        self . q_des = self . q. copy ()
        self.capture_count = 0

    def ik_dls (self , x_des , damp =0.01) :
        """
        Cinem Ã¡ tica inversa num Ã© rica por Damped Least Squares ( DLS ).
        Actualiza q_des para seguir la posici Ã³n cartesiana deseada .

        Args :
        x_des (np. ndarray ): Posici Ã³n cartesiana deseada [m].
        damp ( float ): Factor de amortiguamiento ( evita
        singularidades ).
        """
        for _ in range (5) : # iteraciones internas
            x_cur = fk_3r ( self . q_des )
            e_x = x_des - x_cur
            if np. linalg . norm ( e_x ) < 1e-4:
                break
            J = jacobian_3r ( self . q_des )
        # Pseudoinversa DLS : J^+ = J ( J J + I )^{ -1}


            JJT = J @ J. T
            Jp = J .T @ np. linalg . inv ( JJT + damp **2 * np. eye (2) )
            self . q_des = self . q_des + Jp @ e_x

    def step ( self ):
        """
        Ejecuta un paso de simulaci Ã³n (dt = DT):
        1. Actualizar referencia cartesiana
        2. IK para obtener q_des
        3. Computed Torque
        4. Integrar din Ã¡ mica q, q
        5. Enviar comando al esclavo
        6. Registrar datos
        """
        # 1. Referencia cartesiana: actualizar sÃ³lo si hay entrada manual
        x_cur = fk_3r ( self . q)
        if not hasattr(self, 'x_des'):
            self.x_des = x_cur.copy()
        if (not getattr(self, 'manual_only', False)) or (np.linalg.norm(self . v_cart) > 1e-8):
            self.x_des = self.x_des + self . v_cart * DT
            # asegurar referencia alcanzable
            r = np.linalg.norm(self.x_des)
            max_r = L1 + L2 + L3 - 0.01
            if r > max_r:
                self.x_des = self.x_des * (max_r / r)

        # 2. Cinem Ã¡ tica inversa hacia la referencia deseada
        self . ik_dls ( self.x_des )

        # 3. Computed Torque con feedback hÃ¡ ptico
        tau , e , de = computed_torque (
            self .q , self . dq , self . q_des , self . dq_des , self . ddq_des ,
            F_ext = 0.3 * self . net . Fe # factor de feedback hÃ¡ ptico
        )

        # 4. Integrar din Ã¡ mica
        self .q , self . dq = integrate_dynamics ( self .q , self . dq , tau )

        # 5. Enviar la referencia cartesiana deseada al esclavo (modo manual)
        x_ef = fk_3r ( self . q)
        # tambiÃ©n mandamos una copia acotada para que el esclavo no reciba valores locos
        xd_to_send = self.x_des.copy()
        r = np.linalg.norm(xd_to_send)
        max_r = L1 + L2 + L3 - 0.01
        if r > max_r:
            xd_to_send = xd_to_send * (max_r / r)
        # Si el usuario activÃ³ sync_mode, forzar que el esclavo iguale la altura (y)
        if getattr(self, 'sync_mode', False):
            try:
                xd_to_send[1] = float(x_ef[1])
            except Exception:
                pass
            # indicar al cliente que este envÃ­o es un SYNC
            try:
                self.net._pending_mode = 'SYNC'
            except Exception:
                pass
        try:
            self . net . send_command ( xd_to_send , gripper = not self . gripper_open )
        except Exception:
            # asegurar envÃ­o defensivo en caso de error de red
            try:
                self.net._pending_mode = None
            except Exception:
                pass
            self . net . send_command ( x_ef , gripper = not self . gripper_open )
        # advertencia si no hay respuesta reciente del esclavo
        if time.time() - self.net.last_recv_time > 0.5:
            print("ATENCIÃ“N: no hay respuesta del esclavo en >0.5 s")

        # 6. Registrar datos para grÃ¡ ficas
        i = self . idx % 500
        self . hist_t [ i] = self .t
        self . hist_q [ i] = self .q
        self . hist_tau [ i] = tau
        self . hist_Fe [i ] = self . net . Fe
        self . hist_x [ i] = x_ef
        self . idx += 1
        self .t += DT


#
# GRAFICACI Ã“N EN TIEMPO REAL


#
def setup_plots ( robot ) :
    """
    Configura la figura de graficaci Ã³n con 4 paneles :
    1. Vista 2D del robot ( cinem Ã¡ tica )
    2. Torques articulares , , vs tiempo
    3. Fuerzas de contacto Fx , Fy vs tiempo
    4. Ã ngulos articulares q , q , q vs tiempo
    """
    fig = plt . figure (figsize =(14 , 10) ,
    facecolor ='#0a0a1a')
    fig . suptitle ('TE3001B Robot Maestro 3R | Peg -in - Hole Teleoperado',
    color ='white', fontsize =14 , fontweight ='bold', y =0.98)

    # Paleta de colores
    C = ['#00BFFF', '#FF6B6B', '#69FF47', '#FFD700', '#FF69B4',
    '#00FFD0']
    bg = '#0d1117'
    grid_c = '#1e2530'

    ax_robot = fig . add_subplot (2 , 2, 1, facecolor = bg )
    ax_tau = fig . add_subplot (2 , 2, 2, facecolor = bg )
    ax_force = fig . add_subplot (2 , 2, 3, facecolor = bg )
    ax_q = fig . add_subplot (2 , 2, 4, facecolor = bg )

    for ax in [ ax_robot , ax_tau , ax_force , ax_q ]:
        ax . tick_params ( colors ='#aaa')
        ax . xaxis . label . set_color ('#aaa')
        ax . yaxis . label . set_color ('#aaa')
        ax.title.set_color('white')
        for spine in ax . spines . values ():
            spine . set_edgecolor ('#333')
        ax . grid (True , color = grid_c , linestyle ='--', alpha =0.5)

    # Panel 1: Robot 2D
    ax_robot . set_xlim ( -0.9 , 0.9)
    ax_robot . set_ylim ( -0.9 , 0.9)
    ax_robot . set_aspect ('equal')
    ax_robot . set_title ('Vista Cinem Ã¡ tica 3R', fontsize =11)
    ax_robot . set_xlabel ('x [m]') ; ax_robot . set_ylabel ('y [m]')
    link_line , = ax_robot . plot ([] , [] , 'o-', color =C [0] ,
    linewidth =3 , markersize =8 ,
    markerfacecolor =C [1])
    ef_dot , = ax_robot . plot ([] , [] , 's', color =C [2] , markersize =12 ,
    markerfacecolor =C [3] , zorder =5)
    slave_marker = None
    if not FORCE_GUIDED_DEMO:
        slave_marker , = ax_robot . plot ([] , [] , 'rx', markersize =6 , label='slave')
        hole_x , hole_y = 0.55 , 0.10
        ax_robot . add_patch (plt.Circle(( hole_x , hole_y ) , 0.025 ,
        color ='#FFD700', alpha =0.6) )
        ax_robot . plot ( hole_x , hole_y , 'x', color ='white', markersize =8 ,
        markeredgewidth =2)
        ax_robot . text ( hole_x +0.03 , hole_y +0.03 , 'HOLE ', color ='#FFD700',
        fontsize =8)
    else:
        ax_robot . set_title ('Vista Cinem ?? tica 3R (guiado por fuerza)', fontsize =11)

    # Panel 2: Torques articulares
    ax_tau . set_title ('Torques Articulares [Nm]', fontsize =11)
    ax_tau . set_xlabel ('Tiempo [s]'); ax_tau . set_ylabel (' [Nm]')
    lines_tau = [ ax_tau . plot ([] , [] , color =C[ i] , linewidth =1.5 ,
    label =f' {i +1} ') [0] for i in range (3) ]
    ax_tau . legend ( loc ='upper right', fontsize =9 ,
    facecolor ='#1a1a2e', labelcolor ='white')
    ax_tau . axhline (y =0 , color ='#444', linewidth =0.8)

    # Panel 3: Fuerzas de contacto
    ax_force . set_title ('Fuerzas de Contacto Reflejadas [N]',
    fontsize =11)
    ax_force . set_xlabel ('Tiempo [s]') ; ax_force . set_ylabel ('F [N]')
    line_Fx , = ax_force . plot ([] , [] , color = C [4] , linewidth =1.8 ,
    label ='Fx ')
    line_Fy , = ax_force . plot ([] , [] , color = C [5] , linewidth =1.8 ,
    label ='Fy ')
    ax_force . legend ( loc ='upper right', fontsize =9 ,
    facecolor ='#1a1a2e', labelcolor ='white')
    ax_force . axhline ( y =0 , color ='#444', linewidth =0.8)

    # Panel 4: Ã ngulos articulares
    ax_q . set_title ('Ã ngulos Articulares q [ rad ]', fontsize =11)
    ax_q . set_xlabel ('Tiempo [s]'); ax_q . set_ylabel ('q [ rad ]')
    lines_q = [ ax_q . plot ([] , [] , color =C[ i] , linewidth =1.5 ,
    label =f'q{i +1} ',
    linestyle =[ 'solid','dashed','dotted'][ i ]) [0]
    for i in range (3) ]
    ax_q . legend ( loc ='upper right', fontsize =9 ,
    facecolor ='#1a1a2e', labelcolor ='white')

    plt . tight_layout ( rect =[0 , 0.02 , 1 , 0.96])

    return fig , ( ax_robot , ax_tau , ax_force , ax_q ) , \
    ( link_line , ef_dot , slave_marker ) , lines_tau , ( line_Fx , line_Fy ) , lines_q


def main ( slave_ip ):
    robot = MasterRobot ( slave_ip )
    fig , axes , arm_lines , lines_tau , force_lines , lines_q = setup_plots ( robot )
    ax_robot , ax_tau , ax_force , ax_q = axes
    link_line , ef_dot , slave_marker = arm_lines
    line_Fx , line_Fy = force_lines

    # Hilo de simulaci Ã³n en background
    running = [ True ]
    def sim_loop () :
        while running [0]:
            robot . step ()
            time . sleep ( DT )
    sim_thread = threading . Thread ( target = sim_loop , daemon = True )
    sim_thread . start ()

    def animate ( frame ):
        n = min ( robot . idx , 500)
        i0 = robot . idx % 500
        # Reordenar buffer circular
        idx_range = np. arange ( i0 , i0 + n) % 500
        t = robot . hist_t [ idx_range ]
        tau = robot . hist_tau [ idx_range ]
        Fe = robot . hist_Fe [ idx_range ]
        q_h = robot . hist_q [ idx_range ]
        # calcular error de sincronizaciÃ³n si disponemos de la posiciÃ³n del esclavo
        sync_err = None
        if (not FORCE_GUIDED_DEMO) and robot.net.x_slave is not None:
            sync_err = np.linalg.norm(robot.x_des - robot.net.x_slave)
        # mostrar advertencia textualmente
        if sync_err is not None and sync_err > 0.02:
            print(f"ADVERTENCIA: desincronizaciÃ³n {sync_err:.3f} m")

        # Cinem Ã¡ tica actual
        pts = fk_3r_full ( robot . q)
        link_line . set_data ( pts [: ,0] , pts [: ,1])
        ef_dot . set_data ([ pts [ -1 ,0]] , [ pts [ -1 ,1]])
        if (slave_marker is not None) and (robot.net.x_slave is not None):
            slave_marker.set_data([robot.net.x_slave[0]], [robot.net.x_slave[1]])
        # actualizar tÃ­tulo con error de sincronizaciÃ³n
        if (not FORCE_GUIDED_DEMO) and (robot.net.x_slave is not None):
            err = np.linalg.norm(robot.x_des - robot.net.x_slave)
            fig.suptitle(f"TE3001B Robot Maestro 3R | Sync error {err:.3f} m", color='white', fontsize=14, fontweight='bold', y=0.98)

        # Ventana de tiempo visible (Ãº ltimos 5 s)
        t_win = 5.0
        mask = ( t > robot .t - t_win ) if robot .t > t_win else np. ones (n , bool )

        for i , ln in enumerate ( lines_tau ):
            ln . set_data ( t[ mask ], tau [ mask , i ])
        ax_tau . set_xlim (max (0 , robot .t - t_win ) , max ( t_win , robot . t) )
        ax_tau . relim () ; ax_tau . autoscale_view ( scalex = False )

        line_Fx . set_data (t[ mask ] , Fe [ mask , 0])
        line_Fy . set_data (t[ mask ] , Fe [ mask , 1])
        ax_force . set_xlim (max (0 , robot .t - t_win ) , max ( t_win , robot .t ))
        ax_force . relim () ; ax_force . autoscale_view ( scalex = False )

        for i , ln in enumerate ( lines_q ):
            ln . set_data ( t[ mask ], q_h [ mask , i ])
        ax_q . set_xlim (max (0 , robot .t - t_win ) , max ( t_win , robot .t) )
        ax_q . relim () ; ax_q . autoscale_view ( scalex = False )



        return [ link_line , ef_dot ] + lines_tau + [ line_Fx , line_Fy ] +lines_q

    # Captura de teclas ( movimiento cartesiano )
    def on_key_press ( event ):
        k = (event.key or '').lower()
        v = robot . v_step
        if k == 'w': robot . v_cart = np. array ([0 , v ])
        elif k == 's': robot . v_cart = np. array ([0 , -v ])
        elif k == 'd': robot . v_cart = np. array ([v , 0])
        elif k == 'a': robot . v_cart = np. array ([ -v , 0])
        elif k == 'q': robot . gripper_open = True
        elif k == 'e': robot . gripper_open = False
        elif k == 'p':
            # marcar guardado y cerrar
            robot.save_requested = True
            running [0] = False
            plt . close ()
        elif k == 'c':
            robot.capture_count += 1
            tag = build_capture_tag(robot.capture_count)
            snap_name = f'master_snapshot_{tag}.png'
            history_prefix = f'master_{tag}'
            try:
                fig.savefig(snap_name)
                save_master_history(robot, history_prefix)
                print(f"Captura guardada: {snap_name} + {history_prefix}_history.png")
            except Exception as ex:
                print(f"No se pudo guardar captura: {ex}")
        elif k == 'y':
            robot.sync_mode = not robot.sync_mode
            print(f"SYNC mode {'ON' if robot.sync_mode else 'OFF'}")
        # no cerrar ni guardar con q/e

    def on_key_release ( event ):
        robot . v_cart = np. zeros (2)

    fig . canvas . mpl_connect ('key_press_event', on_key_press )
    fig . canvas . mpl_connect ('key_release_event', on_key_release )

    def on_close(event):
        try:
            fig.savefig('master_snapshot.png')
        except Exception:
            pass
        if robot.save_requested:
            save_master_history(robot, 'master')
        else:
            print("Cierre sin guardar historial; presione 'p' para exportar datos antes de cerrar")

    fig.canvas.mpl_connect('close_event', on_close)

    ani = animation . FuncAnimation ( fig , animate , interval =50 , # 20 fps
        blit =False , cache_frame_data = False )
    plt . show ()


def save_master_history(robot, prefix):
    t = robot.hist_t[: robot.idx if robot.idx < robot.hist_t.size else robot.hist_t.size]
    tau = robot.hist_tau[: t.size]
    Fe = robot.hist_Fe[: t.size]
    x = robot.hist_x[: t.size]
    q = robot.hist_q[: t.size]

    fig2, axs = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs.flatten()
    axs[0].plot(t, Fe[:, 0], label='Fx'); axs[0].plot(t, Fe[:, 1], label='Fy')
    axs[0].set_title('Fuerzas reflejadas'); axs[0].set_xlabel('t [s]'); axs[0].set_ylabel('F [N]')
    axs[0].legend()

    axs[1].plot(t, tau[:, 0], label='Ï„1'); axs[1].plot(t, tau[:, 1], label='Ï„2'); axs[1].plot(t, tau[:, 2], label='Ï„3')
    axs[1].set_title('Torques articulaires'); axs[1].set_xlabel('t [s]'); axs[1].set_ylabel('Ï„ [Nm]')
    axs[1].legend()

    axs[2].plot(t, x[:, 0], label='x'); axs[2].plot(t, x[:, 1], label='y')
    axs[2].set_title('PosiciÃ³n efector'); axs[2].set_xlabel('t [s]'); axs[2].set_ylabel('[m]')
    axs[2].legend()

    axs[3].plot(t, np.linalg.norm(q, axis=1), label='|q|')
    axs[3].set_title('Magnitud Ã¡ngulos articulares'); axs[3].set_xlabel('t [s]'); axs[3].set_ylabel('rad')
    axs[3].legend()

    plt.tight_layout()
    try:
        fig2.savefig(f'{prefix}_history.png')
    except Exception:
        pass
    plt.close(fig2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="TE3001B Robot Maestro Peg-in-Hole"
    )
    parser.add_argument(
        "--slave-ip",
        default="127.0.0.1",
        help="IP del PC esclavo (default: loopback)",
    )
    args = parser.parse_args()
    main(args.slave_ip)