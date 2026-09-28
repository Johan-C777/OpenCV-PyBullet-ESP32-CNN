"""
Brazo robótico (brazo.urdf) que dibuja en una pizarra el carácter elegido en el
ESP32 (joystick + OLED), usando cinemática inversa, con rastro en 3D y visión
artificial con OpenCV.

Cinemática del brazo (verificada contra PyBullet):
  joint 0  joint_1        revolute, eje Z  -> giro de la base (yaw)
  joint 1  joint_2        revolute, eje Y  -> inclinación del brazo (pitch), hombro 0.50 m sobre la base
  joint 2  joint_gripper  prismatic, eje Z -> EXTENSIÓN telescópica del brazo (0 a 0.15 m)
  joint 3/4               dedos de la pinza (no participan en el dibujo)
Es un brazo esférico (R-R-P): la punta del lápiz = hombro + (0.42 + d) · dirección del brazo.
El "lápiz" es la punta de los dedos: 0.12 m delante del link gripper_base (link 2).

Teclado en la ventana de visión: 0-9, a-d, * y # dibujan sin hardware;
L limpia la pizarra; Q sale.
"""

import math
import time
from collections import deque

import cv2
import numpy as np
import pybullet as p
import pybullet_data
import serial

# ============================================================
# CONFIGURACIÓN
# ============================================================
PUERTO_SERIAL = 'COM3'
BAUDRATE = 115200
URDF_BRAZO = "brazo.urdf"
POS_BASE = [0, 0, 0]          # base apoyada en el piso

CARACTERES_VALIDOS = "0123456789ABCD*#"

DT = 1.0 / 240.0

# Joints (índices de PyBullet)
J_BASE, J_HOMBRO, J_EXTENSION, J_DEDO_IZQ, J_DEDO_DER = 0, 1, 2, 3, 4
JOINTS_BRAZO = [J_BASE, J_HOMBRO, J_EXTENSION]
LINK_PINZA = 2              # gripper_base: último link de la cadena del brazo
OFFSET_PUNTA = 0.12         # punta de los dedos respecto a gripper_base (m)
FUERZA = {J_BASE: 5000, J_HOMBRO: 5000, J_EXTENSION: 5000} # Fuerza brutal para que el brazo no se quede atrás
VEL_MAX = {J_BASE: 10.0, J_HOMBRO: 10.0, J_EXTENSION: 10.0} 
GANANCIA_POS = 1.0

# Pizarra y plano de dibujo (plano vertical x = X_PLANO, frente al robot)
X_PLANO = 0.48              
X_ALZADO = 0.44             
CENTRO_Y, CENTRO_Z = 0.0, 0.50  
ANCHO_CAR, ALTO_CAR = 0.18, 0.28   

V_TRAZO = 0.40              # Velocidad rápida pero trazable matemáticamente
V_AIRE = 0.80               # Movimiento aéreo rápido
T_ARTICULAR = 0.5           # Acercamiento a la pizarra en medio segundo
TOL_LLEGADA = 0.01          
T_ESPERA = 0.1             
TOL_CONTACTO = 0.02         # Tolerancia amplia para que siempre suelte tinta
PASO_TINTA = 0.002          # Gotas muy juntas

Q_REPOSO = [-2.3, 1.3, 0.0]   
SEMILLA_PIZARRA = [0.0, 1.57, 0.05]

# Cámara sintética (mira la pizarra por encima y detrás del robot)
CAM_ANCHO, CAM_ALTO = 480, 360
CAM_OJO = [-0.45, 0.0, 0.90]
CAM_OBJETIVO = [X_PLANO, CENTRO_Y, CENTRO_Z]
CAM_FOV = 38
CAM_CADA_N_PASOS = 8          # ~30 imágenes por segundo


# ============================================================
# FUENTE VECTORIAL: cada carácter es una lista de trazos; cada
# trazo es una lista de puntos (u, v) en un cuadro de 0 a 1
# (u = izquierda -> derecha, v = abajo -> arriba).
# ============================================================
def arco(cx, cy, rx, ry, a0, a1, n=None):
    """Puntos de un arco de elipse entre los ángulos a0 y a1 (grados)."""
    if n is None:
        n = max(8, int(abs(a1 - a0) / 8))
    return [(cx + rx * math.cos(math.radians(a0 + (a1 - a0) * k / n)),
             cy + ry * math.sin(math.radians(a0 + (a1 - a0) * k / n))) for k in range(n + 1)]


FUENTE = {
    '0': [arco(0.5, 0.5, 0.45, 0.5, 90, 450)],
    '1': [[(0.2, 0.78), (0.55, 1.0), (0.55, 0.0)], [(0.2, 0.0), (0.9, 0.0)]],
    '2': [arco(0.5, 0.72, 0.45, 0.28, 160, -40) + [(0.0, 0.0), (1.0, 0.0)]],
    '3': [arco(0.5, 0.75, 0.42, 0.25, 150, -90) + arco(0.5, 0.25, 0.48, 0.25, 90, -150)],
    '4': [[(0.72, 0.0), (0.72, 1.0), (0.0, 0.3), (1.0, 0.3)]],
    '5': [[(0.95, 1.0), (0.12, 1.0), (0.08, 0.55)] + arco(0.5, 0.32, 0.45, 0.3, 125, -140)],
    '6': [arco(0.5, 0.5, 0.45, 0.5, 60, 180) + [(0.05, 0.3)] + arco(0.5, 0.3, 0.45, 0.3, 180, 540)],
    '7': [[(0.0, 1.0), (1.0, 1.0), (0.35, 0.0)]],
    '8': [arco(0.5, 0.75, 0.38, 0.25, -90, 270) + arco(0.5, 0.25, 0.45, 0.25, 90, -270)],
    '9': [arco(0.5, 0.7, 0.45, 0.3, 0, 360) + [(0.95, 0.3)] + arco(0.5, 0.3, 0.45, 0.3, 0, -150)],
    'A': [[(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)], [(0.22, 0.42), (0.78, 0.42)]],
    'B': [[(0.1, 0.0), (0.1, 1.0), (0.55, 1.0)] + arco(0.55, 0.76, 0.35, 0.24, 90, -90)
          + [(0.1, 0.52), (0.6, 0.52)] + arco(0.6, 0.26, 0.38, 0.26, 90, -90) + [(0.1, 0.0)]],
    'C': [arco(0.55, 0.5, 0.48, 0.5, 45, 315)],
    'D': [[(0.1, 0.0), (0.1, 1.0), (0.45, 1.0)] + arco(0.45, 0.5, 0.5, 0.5, 90, -90) + [(0.1, 0.0)]],
    '*': [[(0.5, 0.15), (0.5, 0.85)], [(0.2, 0.3), (0.8, 0.7)], [(0.2, 0.7), (0.8, 0.3)]],
    '#': [[(0.35, 0.0), (0.45, 1.0)], [(0.6, 0.0), (0.7, 1.0)],
          [(0.1, 0.35), (0.95, 0.35)], [(0.05, 0.65), (0.9, 0.65)]],
}


def aMundo(u, v, x):
    """(u, v) de la fuente -> punto 3D en el plano x. Visto desde el robot,
    la derecha es -Y, por eso u crece hacia -Y."""
    return np.array([x, CENTRO_Y - (u - 0.5) * ANCHO_CAR, CENTRO_Z + (v - 0.5) * ALTO_CAR])


# ============================================================
# MUNDO
# ============================================================
def iniciarMundo(modo=p.GUI):
    p.connect(modo)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF_BRAZO, POS_BASE, useFixedBase=True)

    # Pizarra (solo visual, sin colisión): marco negro + superficie blanca
    marco = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.005, 0.20, 0.22], rgbaColor=[0.05, 0.05, 0.05, 1])
    blanca = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.004, 0.16, 0.18], rgbaColor=[1, 1, 1, 1])
    p.createMultiBody(0, -1, marco, [X_PLANO + 0.012, CENTRO_Y, CENTRO_Z])
    alto_pata = CENTRO_Z - 0.22
    pata = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.02, alto_pata / 2], rgbaColor=[0.3, 0.3, 0.3, 1])
    p.createMultiBody(0, -1, pata, [X_PLANO + 0.02, CENTRO_Y, alto_pata / 2])
    pizarra = p.createMultiBody(0, -1, blanca, [X_PLANO + 0.006, CENTRO_Y, CENTRO_Z])

    for j in range(p.getNumJoints(robot)):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=0, force=20)
    for j, q in zip(JOINTS_BRAZO, Q_REPOSO):
        p.resetJointState(robot, j, q)

    p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=-55, cameraPitch=-15,
                                 cameraTargetPosition=[0.28, 0.0, 0.47])
    return robot, pizarra


class Cinematica:
    def __init__(self, robot):
        self.robot = robot
        info = [p.getJointInfo(robot, j) for j in range(p.getNumJoints(robot))]
        self.moviles = [i[0] for i in info if i[3] > -1]
        self.ll = [info[j][8] for j in self.moviles]
        self.ul = [info[j][9] for j in self.moviles]
        self.hombro = np.array(p.getLinkState(robot, J_HOMBRO, computeForwardKinematics=1)[4])

    def punta(self):
        """Posición real de la punta del lápiz (frente de los dedos)."""
        pos, orn = p.getLinkState(self.robot, LINK_PINZA, computeForwardKinematics=1)[4:6]
        eje = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)[:, 2]
        return np.array(pos) + OFFSET_PUNTA * eje

    def articulaciones(self):
        return [p.getJointState(self.robot, j)[0] for j in JOINTS_BRAZO]

    def ik(self, punta_deseada, semilla=None):
        """Ángulos (base, hombro, extensión) para llevar la punta del lápiz al punto.
        La punta está sobre la línea hombro -> pinza, así que el objetivo de la
        pinza es el mismo punto retrocedido OFFSET_PUNTA en esa dirección.
        La IK parte de la postura actual; 'semilla' permite partir de otra (un brazo
        esférico tiene dos soluciones: (base, hombro) y (base ± 180°, -hombro))."""
        guardado = None
        if semilla is not None:
            guardado = [p.getJointState(self.robot, j)[:2] for j in JOINTS_BRAZO]
            for j, v in zip(JOINTS_BRAZO, semilla):
                p.resetJointState(self.robot, j, v)
        d = punta_deseada - self.hombro
        objetivo_pinza = punta_deseada - OFFSET_PUNTA * d / np.linalg.norm(d)
        # Sin parámetros de nullspace: con 3 GDL para una posición 3D la solución es
        # única, y el modo nullspace de PyBullet calcula mal el joint prismático.
        q = p.calculateInverseKinematics(self.robot, LINK_PINZA, objetivo_pinza.tolist(),
                                         maxNumIterations=200, residualThreshold=1e-6)
        q = [q[self.moviles.index(j)] for j in JOINTS_BRAZO]
        if guardado is not None:
            for j, (pos, vel) in zip(JOINTS_BRAZO, guardado):
                p.resetJointState(self.robot, j, pos, vel)
        return [min(max(v, self.ll[self.moviles.index(j)]), self.ul[self.moviles.index(j)])
                for v, j in zip(q, JOINTS_BRAZO)]


def moverBrazo(robot, q):
    for j, v in zip(JOINTS_BRAZO, q):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=v,
                                force=FUERZA[j], maxVelocity=VEL_MAX[j], positionGain=GANANCIA_POS)


# ============================================================
# PLANIFICACIÓN DE LA TRAYECTORIA
# ============================================================
def densificar(a, b, v):
    """Puntos entre a y b separados lo que avanza el lápiz en un paso a velocidad v."""
    n = max(1, int(math.ceil(np.linalg.norm(b - a) / (v * DT))))
    return [a + (b - a) * k / n for k in range(1, n + 1)]


def rutaCartesiana(caracter):
    """Waypoints 3D (uno por paso de simulación) para dibujar el carácter:
    por cada trazo: ir levantado al inicio, bajar, recorrer, levantar."""
    ruta = []
    actual = None
    for trazo in FUENTE[caracter]:
        inicio_alto = aMundo(*trazo[0], X_ALZADO)
        if actual is None:
            ruta.append(inicio_alto)
        else:
            ruta += densificar(actual, inicio_alto, V_AIRE)
        actual = inicio_alto
        for (u, v) in trazo:                         # bajar y recorrer el trazo
            siguiente = aMundo(u, v, X_PLANO)
            ruta += densificar(actual, siguiente, V_TRAZO)
            actual = siguiente
        fin_alto = aMundo(*trazo[-1], X_ALZADO)      # levantar
        ruta += densificar(actual, fin_alto, V_TRAZO)
        actual = fin_alto
    return ruta


def suave(s):
    return s * s * (3 - 2 * s)      # smoothstep: arranca y frena sin tirones


class Dibujante:
    """Máquina de estados no bloqueante: REPOSO -> IR -> DIBUJAR -> ESPERAR -> VOLVER -> REPOSO.
    Se llama paso() una vez por cada paso de simulación."""

    def __init__(self, robot, cin):
        self.robot, self.cin = robot, cin
        self.cola = deque()
        self.estado = 'REPOSO'
        self.caracter = None
        self.q_objetivo = list(Q_REPOSO)
        self.tinta_ids, self.linea_ids = [], []
        self.ultima_tinta = None
        self.forma_tinta = p.createVisualShape(p.GEOM_SPHERE, radius=0.0035, rgbaColor=[0.05, 0.1, 0.35, 1])

    def encolar(self, c):
        self.cola.append(c)
        print(f"Recibido '{c}'" + (" (en cola)" if self.estado != 'REPOSO' else ""))

    def limpiar(self):
        for b in self.tinta_ids:
            p.removeBody(b)
        for l in self.linea_ids:
            p.removeUserDebugItem(l)
        self.tinta_ids, self.linea_ids, self.ultima_tinta = [], [], None

    def _iniciar(self, c):
        self.limpiar()
        self.caracter = c
        self.ruta = rutaCartesiana(c)
        self.q_ini = self.cin.articulaciones()
        self.q_fin = self.cin.ik(self.ruta[0], semilla=SEMILLA_PIZARRA)
        self.t, self.k = 0.0, 0
        self.estado = 'IR'
        print(f"Dibujando '{c}' ({len(self.ruta) * DT:.1f} s de trazo)")

    def paso(self):
        if self.estado == 'REPOSO':
            if self.cola:
                self._iniciar(self.cola.popleft())
        elif self.estado in ('IR', 'VOLVER'):
            self.t += DT
            s = suave(min(1.0, self.t / T_ARTICULAR))
            self.q_objetivo = [a + (b - a) * s for a, b in zip(self.q_ini, self.q_fin)]
            if self.t >= T_ARTICULAR:
                if self.estado == 'VOLVER':
                    self.estado, self.caracter = 'REPOSO', None
                # IR: empieza a dibujar solo cuando la punta llegó al inicio (máx. 1 s extra)
                elif (np.linalg.norm(self.cin.punta() - self.ruta[0]) < TOL_LLEGADA
                      or self.t >= T_ARTICULAR + 1.0):
                    self.estado = 'DIBUJAR'
        elif self.estado == 'DIBUJAR':
            self.q_objetivo = self.cin.ik(self.ruta[self.k])
            self.k += 1
            if self.k >= len(self.ruta):
                self.estado, self.t = 'ESPERAR', 0.0
        elif self.estado == 'ESPERAR':
            self.t += DT
            if self.t >= T_ESPERA:
                self.q_ini, self.q_fin = self.cin.articulaciones(), list(Q_REPOSO)
                self.estado, self.t = 'VOLVER', 0.0

        moverBrazo(self.robot, self.q_objetivo)

    def entintar(self):
        """Deja tinta donde la punta REAL toca la pizarra (como un lápiz de verdad):
        una gota visible para la cámara y una línea de depuración en el GUI."""
        
        # --- BLOQUEO ANTI-REBOTE AÑADIDO AQUÍ ---
        if self.estado != 'DIBUJAR':
            self.ultima_tinta = None
            return
        # ---------------------------------------

        punta = self.cin.punta()
        if abs(punta[0] - X_PLANO) > TOL_CONTACTO:
            self.ultima_tinta = None                 # lápiz levantado: corta la línea
            return
            
        gota = np.array([X_PLANO - 0.002, punta[1], punta[2]])
        if self.ultima_tinta is not None and np.linalg.norm(gota - self.ultima_tinta) < PASO_TINTA:
            return
            
        self.tinta_ids.append(p.createMultiBody(0, -1, self.forma_tinta, gota.tolist()))
        if self.ultima_tinta is not None:
            self.linea_ids.append(p.addUserDebugLine(self.ultima_tinta.tolist(), gota.tolist(),
                                                     [0.1, 0.2, 0.9], lineWidth=4))
        self.ultima_tinta = gota

# ============================================================
# SERIAL
# ============================================================
class LectorSerial:
    """Lectura no bloqueante; entrega solo líneas completas."""

    def __init__(self, ser):
        self.ser, self.buffer = ser, b""

    def lineas(self):
        n = self.ser.in_waiting
        if n <= 0:
            return []
        self.buffer += self.ser.read(n)
        *completas, self.buffer = self.buffer.split(b"\n")
        return [l.decode('utf-8', errors='ignore').strip() for l in completas]


# ============================================================
# VISIÓN ARTIFICIAL
# ============================================================
class Vision:
    def __init__(self, pizarra_id):
        self.pizarra_id = pizarra_id
        self.vm = p.computeViewMatrix(CAM_OJO, CAM_OBJETIVO, [0, 0, 1])
        self.pm = p.computeProjectionMatrixFOV(CAM_FOV, CAM_ANCHO / CAM_ALTO, 0.02, 5.0)

    def procesar(self, tinta_ids, texto_estado):
        _, _, rgb, _, seg = p.getCameraImage(CAM_ANCHO, CAM_ALTO, self.vm, self.pm)
        bgr = cv2.cvtColor(np.reshape(rgb, (CAM_ALTO, CAM_ANCHO, 4))[:, :, :3].astype(np.uint8),
                           cv2.COLOR_RGB2BGR)
        seg = np.reshape(seg, (CAM_ALTO, CAM_ANCHO))

        # 1) Bordes de toda la escena (Canny)
        gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        suavizada = cv2.GaussianBlur(gris, (5, 5), 0)
        bordes = cv2.Canny(suavizada, 50, 150)

        # 2) Región de la pizarra (segmentación) y umbral: la tinta es oscura sobre blanco
        roi = np.isin(seg, [self.pizarra_id] + tinta_ids).astype(np.uint8) * 255
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        _, oscuro = cv2.threshold(suavizada, 110, 255, cv2.THRESH_BINARY_INV)
        trazo = cv2.bitwise_and(oscuro, roi)
        trazo = cv2.morphologyEx(trazo, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        # 3) Contornos del trazo + caja envolvente
        contornos, _ = cv2.findContours(trazo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contornos = [c for c in contornos if cv2.contourArea(c) > 15]
        vista = bgr.copy()
        cv2.drawContours(vista, contornos, -1, (0, 200, 0), 2)
        if contornos:
            x, y, w, h = cv2.boundingRect(np.vstack(contornos))
            cv2.rectangle(vista, (x - 4, y - 4), (x + w + 4, y + h + 4), (0, 0, 255), 2)
            info = f"Contornos: {len(contornos)}  Caja: {w}x{h} px"
        else:
            info = "Contornos: 0"

        panel_bordes = cv2.cvtColor(bordes, cv2.COLOR_GRAY2BGR)
        for img, titulo in ((vista, "Camara + contornos"), (panel_bordes, "Bordes (Canny)")):
            cv2.rectangle(img, (0, 0), (CAM_ANCHO, 26), (40, 40, 40), -1)
            cv2.putText(img, titulo, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        for img, texto, fila in ((vista, texto_estado, 1), (vista, info, 2)):
            cv2.putText(img, texto, (8, CAM_ALTO - 12 - 22 * (2 - fila)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, texto, (8, CAM_ALTO - 12 - 22 * (2 - fila)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
        return np.hstack([vista, panel_bordes]), len(contornos)


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================
VENTANA = "Vision artificial - camara sintetica"

if __name__ == "__main__":
    robot, pizarra = iniciarMundo()
    cin = Cinematica(robot)
    dibujante = Dibujante(robot, cin)
    vision = Vision(pizarra)
    texto_id = p.addUserDebugText("Esperando caracter...", [X_PLANO, CENTRO_Y, CENTRO_Z + 0.25],
                                  [0, 0, 0], textSize=1.4)

    try:
        ser = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=0)
        lector = LectorSerial(ser)
        print(f"ESP32 conectado en {PUERTO_SERIAL}")
    except Exception as e:
        ser = lector = None
        print(f"Sin hardware serial ({e}). Usa el teclado en la ventana de visión.")

    cv2.namedWindow(VENTANA, cv2.WINDOW_NORMAL)
    print("Teclas en la ventana de visión: 0-9, a-d, *, # dibujan | L limpia | Q sale")

    paso, estado_previo = 0, None
    t_siguiente = time.perf_counter()
    try:
        while p.isConnected():
            # --- Serial: cada línea válida es un carácter a dibujar ---
            if ser:
                try:
                    for linea in lector.lineas():
                        c = linea.upper()
                        if len(c) == 1 and c in CARACTERES_VALIDOS:
                            dibujante.encolar(c)
                except (serial.SerialException, OSError):
                    print("ESP32 desconectado")
                    ser = None

            dibujante.paso()
            p.stepSimulation()
            dibujante.entintar()

            estado = (dibujante.estado, dibujante.caracter)
            if estado != estado_previo:
                texto = f"Dibujando: {dibujante.caracter}" if dibujante.caracter else "Esperando caracter..."
                texto_id = p.addUserDebugText(texto, [X_PLANO, CENTRO_Y, CENTRO_Z + 0.25], [0, 0, 0],
                                              textSize=1.4, replaceItemUniqueId=texto_id)
                estado_previo = estado

            if paso % CAM_CADA_N_PASOS == 0:
                txt = f"Estado: {dibujante.estado}" + (f"  '{dibujante.caracter}'" if dibujante.caracter else "")
                panel, _ = vision.procesar(dibujante.tinta_ids, txt)
                cv2.imshow(VENTANA, panel)
                tecla = cv2.waitKey(1) & 0xFF
                if tecla in (ord('q'), ord('Q')):
                    break
                if tecla in (ord('l'), ord('L')) and dibujante.estado == 'REPOSO':
                    dibujante.limpiar()
                elif tecla != 255 and chr(tecla).upper() in CARACTERES_VALIDOS:
                    dibujante.encolar(chr(tecla).upper())
                try:
                    if cv2.getWindowProperty(VENTANA, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
            paso += 1

            # Tiempo real: un paso de simulación cada 1/240 s
            t_siguiente += DT
            espera = t_siguiente - time.perf_counter()
            if espera > 0:
                time.sleep(espera)
            elif espera < -0.1:
                t_siguiente = time.perf_counter()
    except p.error:
        pass            # ventana de PyBullet cerrada
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        try:
            if p.isConnected():
                p.disconnect()
        except p.error:
            pass