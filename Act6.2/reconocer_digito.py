# reconocer_digito.py
# Reconoce dígitos escritos a mano usando la webcam y un modelo CNN MNIST.

# ------------------------------------------------------------------
# 0. Silenciar mensajes de TensorFlow (deben ir ANTES de importar TF)
# ------------------------------------------------------------------
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # Solo errores
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'   # Desactiva oneDNN

import cv2
import numpy as np
import tensorflow as tf
import time      # >>> NUEVO
import serial    # >>> NUEVO  (pip install pyserial)


# ------------------------------------------------------------------
# 1. Función de preprocesamiento estilo MNIST
# ------------------------------------------------------------------
def preprocesar_digito(roi):
    """
    Convierte un recorte BGR de la webcam en una imagen 28x28 tipo MNIST:
      - Fondo negro, dígito blanco
      - Centrado por centro de masa
      - Escalado a 20x20 con padding
    Devuelve (imagen_normalizada, caja) o (None, None) si no hay dígito.
    """
    if roi is None or roi.size == 0:
        return None, None

    # 1.1 Escala de grises
    gris = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # 1.2 Desenfoque suave (kernel pequeño para no cerrar el hueco del 0)
    blur = cv2.GaussianBlur(gris, (5, 5), 0)

    # 1.3 Umbral adaptativo (más robusto que uno fijo)
    umbral = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        11, 2
    )

    # 1.4 Encontrar el contorno más grande (el dígito)
    contornos, _ = cv2.findContours(umbral, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contornos:
        return None, None

    c = max(contornos, key=cv2.contourArea)
    if cv2.contourArea(c) < 500:      # Ignorar ruido
        return None, None

    # 1.5 Recortar el dígito
    x, y, w, h = cv2.boundingRect(c)
    digito = umbral[y:y+h, x:x+w]

    # 1.6 Reescalar manteniendo proporción a 20x20 (MNIST: 20x20 + margen)
    if w > h:
        nuevo_w = 20
        nuevo_h = max(1, int(round(20 * h / w)))
    else:
        nuevo_h = 20
        nuevo_w = max(1, int(round(20 * w / h)))

    digito = cv2.resize(digito, (nuevo_w, nuevo_h),
                        interpolation=cv2.INTER_AREA)

    # 1.7 Lienzo 28x28 negro y pegar centrado por centro de masa
    lienzo = np.zeros((28, 28), dtype=np.uint8)

    M = cv2.moments(digito)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = nuevo_w // 2, nuevo_h // 2

    desplaz_x = 14 - cx
    desplaz_y = 14 - cy

    for i in range(nuevo_h):
        for j in range(nuevo_w):
            yi = i + desplaz_y
            xj = j + desplaz_x
            if 0 <= yi < 28 and 0 <= xj < 28:
                lienzo[yi, xj] = digito[i, j]

    # 1.8 Normalizar a [0,1]
    lienzo = lienzo / 255.0

    return lienzo, (x, y, w, h)


# ------------------------------------------------------------------
# >>> NUEVO  1.5 Enlace serial con el ESP-A (maestro SPI)
# ------------------------------------------------------------------
PUERTO = 'COM5'
BAUDIOS = 115200
CONFIANZA_MIN = 60     # % mínimo para enviar
VOTOS_MIN = 4          # de las últimas VENTANA predicciones, cuántas deben coincidir
T_OLVIDO = 1.0         # s sin dígito en el recuadro para permitir reenviar el mismo número


class EnlaceESP:
    """Envía el dígito al ESP-A solo cuando cambia (no en cada frame) y
    lee las confirmaciones del ESP-A (OK:d / ERR:d) sin bloquear."""

    def __init__(self, puerto, baudios):
        self.ser = None
        self.buffer = b""
        self.ultimo_enviado = None
        self.t_visto = time.time()
        self.respuesta = ""            # última confirmación del ESP-A
        try:
            self.ser = serial.Serial(puerto, baudios, timeout=0)
            print(f"ESP-A conectado en {puerto}")
        except serial.SerialException as e:
            print(f"No se pudo abrir {puerto}: {e}\nSe reconocerá sin enviar al ESP-A.")

    def procesar(self, digito, votos, confianza):
        """Llamar en cada frame. digito=None si no hay dígito en el recuadro."""
        ahora = time.time()
        if digito is None:
            if ahora - self.t_visto > T_OLVIDO:
                self.ultimo_enviado = None   # si vuelves a mostrar el mismo número, se reenvía
            return
        self.t_visto = ahora
        if (votos >= VOTOS_MIN and confianza > CONFIANZA_MIN
                and digito != self.ultimo_enviado):
            self.enviar(digito)

    def enviar(self, digito):
        self.ultimo_enviado = digito
        if self.ser is None:
            print(f"(sin ESP-A) se enviaría: {digito}")
            return
        try:
            self.ser.write(f"{digito}\n".encode())
            print(f"Enviado al ESP-A: {digito}")
        except serial.SerialException as e:
            print(f"Se perdió la conexión con el ESP-A: {e}")
            self.ser = None

    def leer_respuestas(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self.buffer += self.ser.read(n)
                *lineas, self.buffer = self.buffer.split(b"\n")
                for linea in lineas:
                    texto = linea.decode("utf-8", errors="ignore").strip()
                    if texto:
                        print(f"ESP-A -> {texto}")
                        if texto.startswith(("OK:", "ERR:")):
                            self.respuesta = texto
        except serial.SerialException:
            self.ser = None

    def cerrar(self):
        if self.ser is not None:
            self.ser.close()


# Se abre ANTES de cargar el modelo: al abrir el puerto el ESP-A se reinicia,
# y así termina de arrancar mientras TensorFlow carga.
enlace = EnlaceESP(PUERTO, BAUDIOS)


# ------------------------------------------------------------------
# 2. Cargar el modelo entrenado
# ------------------------------------------------------------------
modelo = tf.keras.models.load_model('modelo_mnist_cnn.h5')
print("Modelo cargado. Presiona 'q' para salir.")


# ------------------------------------------------------------------
# 3. Abrir la cámara
# ------------------------------------------------------------------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

# Región de interés (ajústala según la resolución de tu cámara)
x1, y1, x2, y2 = 300, 100, 600, 400

# Variables para suavizar la predicción (evita parpadeos)
historial = []
VENTANA = 5

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3.1 >>> CORREGIDO: recortar la ROI ANTES de dibujar el recuadro guía.
    #     Si se dibuja primero, el borde verde queda dentro de la ROI y con la
    #     hoja en blanco se detecta como "dígito" (se enviarían números falsos).
    roi = frame[y1:y2, x1:x2].copy()
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 3.2 Preprocesar la ROI
    digito_procesado, bbox = preprocesar_digito(roi)

    if digito_procesado is not None:
        # 3.3 Predecir
        entrada = digito_procesado.reshape(1, 28, 28, 1)
        prediccion = modelo.predict(entrada, verbose=0)
        clase = int(np.argmax(prediccion))
        confianza = float(np.max(prediccion)) * 100

        # 3.4 Suavizar con votación mayoritaria
        historial.append(clase)
        if len(historial) > VENTANA:
            historial.pop(0)

        # Solo mostramos la clase si es la más repetida y confiable
        clase_estable = max(set(historial), key=historial.count)
        if confianza > 60:
            texto = f"Numero: {clase_estable} ({confianza:.1f}%)"
            cv2.putText(frame, texto, (x1, y1 - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)

        # >>> NUEVO: enviar al ESP-A si el dígito es estable, confiable y cambió
        votos = historial.count(clase_estable)
        enlace.procesar(clase_estable, votos,
                        confianza if clase == clase_estable else 0)

        # 3.5 Mostrar el dígito preprocesado ampliado
        vista = (digito_procesado * 255).astype(np.uint8)
        vista = cv2.resize(vista, (200, 200),
                           interpolation=cv2.INTER_NEAREST)
        cv2.imshow('Digito procesado (28x28 ampliado)', vista)

    else:
        # >>> NUEVO: sin dígito en el recuadro, los votos viejos no deben contar
        historial.clear()
        enlace.procesar(None, 0, 0)

    # >>> NUEVO: confirmaciones del ESP-A y estado del envío en pantalla
    enlace.leer_respuestas()
    if enlace.ultimo_enviado is not None:
        color = (0, 0, 255) if enlace.respuesta.startswith("ERR") else (255, 200, 0)
        cv2.putText(frame, f"Enviado al ESP-A: {enlace.ultimo_enviado}  {enlace.respuesta}",
                    (10, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # 3.6 Mostrar frame principal
    cv2.imshow('Reconocimiento de digitos', frame)

    # 3.7 Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
enlace.cerrar()   # >>> NUEVO