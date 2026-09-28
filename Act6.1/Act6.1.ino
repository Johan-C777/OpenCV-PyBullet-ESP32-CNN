/*
  Selector de caracteres para el brazo que dibuja en PyBullet
  ESP32 + joystick analógico + OLED SSD1306 128x64 (I2C)

  - Eje Y del joystick: recorre los caracteres 0-9, A-D, *, #
  - Botón SW del joystick: envía el carácter por Serial (115200) y muestra "Enviando..."
  - Trama enviada: un solo carácter por línea, p. ej. "7\n"

  CONEXIONES
    OLED  VCC -> 3V3     GND -> GND     SDA -> GPIO 21     SCL -> GPIO 22
    Joy   VCC -> 3V3 (no 5 V)   GND -> GND   VRy -> GPIO 34   SW -> GPIO 25
    (VRx no se usa)

  LIBRERÍAS (Gestor de librerías de Arduino IDE)
    "Adafruit SSD1306" (instala también "Adafruit GFX Library")
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Pines y pantalla ----------------
const int PIN_VRY = 34;           // ADC1, solo entrada
const int PIN_SW  = 25;           // botón del joystick (activo en bajo)
const int PIN_SDA = 21;
const int PIN_SCL = 23;
const uint8_t OLED_DIRECCION = 0x3C;   // algunas pantallas usan 0x3D

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ---------------- Caracteres ----------------
const char CARACTERES[] = "0123456789ABCD*#";
const int  N_CARACTERES = sizeof(CARACTERES) - 1;

// ---------------- Ajustes del joystick ----------------
const int  UMBRAL      = 1200;    // cuánto hay que mover el stick (desde el centro) para cambiar
const int  REARME      = 400;     // debe volver a menos de esto del centro para contar un nuevo toque
const unsigned long T_PRIMERA_REPETICION = 500;  // ms sosteniendo antes de empezar a avanzar solo
const unsigned long T_REPETICION         = 220;  // ms entre avances sosteniendo el stick
const bool INVERTIR_Y  = false;   // cámbialo si "arriba" avanza al revés de lo que quieres

// ---------------- Ajustes del botón y pantalla ----------------
const unsigned long T_ANTIRREBOTE = 30;   // ms que la lectura debe mantenerse estable
const unsigned long T_ENVIANDO    = 800;  // ms que se muestra "Enviando..."

// ---------------- Estado ----------------
int  centroY = 2048;
int  indice = 0;
int  direccionActiva = 0;          // 0 = stick en el centro (rearmado)
bool primeraRepeticion = true;
unsigned long tUltimoPaso = 0;

bool swEstable = HIGH, swUltimaLectura = HIGH;
unsigned long tCambioSw = 0;

bool enviando = false;
unsigned long tEnvio = 0;
bool redibujar = true;
bool oledOk = false;

// ============================================================
int leerY() {                       // promedio de 8 lecturas para quitar ruido del ADC
  long suma = 0;
  for (int i = 0; i < 8; i++) suma += analogRead(PIN_VRY);
  return suma / 8;
}

void calibrarCentro() {             // el reposo real del joystick casi nunca es 2048
  long suma = 0;
  for (int i = 0; i < 32; i++) { suma += leerY(); delay(3); }
  int c = suma / 32;
  centroY = (abs(c - 2048) < 1000) ? c : 2048;   // si alguien lo estaba moviendo, usa 2048
}

void cambiarCaracter(int paso) {
  indice = (indice + paso + N_CARACTERES) % N_CARACTERES;
  redibujar = true;
}

void imprimirCentrado(const char *texto, int tam, int y) {
  oled.setTextSize(tam);
  int ancho = strlen(texto) * 6 * tam;
  oled.setCursor((128 - ancho) / 2, y);
  oled.print(texto);
}

void pantallaSeleccion() {
  char grande[2] = { CARACTERES[indice], '\0' };
  char anterior[2] = { CARACTERES[(indice - 1 + N_CARACTERES) % N_CARACTERES], '\0' };
  char siguiente[2] = { CARACTERES[(indice + 1) % N_CARACTERES], '\0' };
  char posicion[8];
  snprintf(posicion, sizeof(posicion), "%d/%d", indice + 1, N_CARACTERES);

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("Caracter");
  oled.setCursor(128 - strlen(posicion) * 6, 0);
  oled.print(posicion);

  oled.setTextSize(2);                 // vecinos pequeños a los lados
  oled.setCursor(4, 30);
  oled.print(anterior);
  oled.setCursor(128 - 16, 30);
  oled.print(siguiente);

  imprimirCentrado(grande, 6, 14);     // carácter seleccionado: 36x48 px
  oled.display();
}

void pantallaEnviando() {
  char grande[2] = { CARACTERES[indice], '\0' };
  oled.clearDisplay();
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);     // barra de título invertida
  oled.setTextColor(SSD1306_BLACK);
  imprimirCentrado("Enviando...", 1, 2);
  oled.setTextColor(SSD1306_WHITE);
  imprimirCentrado(grande, 6, 15);
  oled.display();
}

void enviarCaracter() {
  Serial.println(CARACTERES[indice]);   // una línea = un carácter para Python
  enviando = true;
  tEnvio = millis();
  redibujar = true;
}

// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VRY, ADC_11db);    // rango completo 0-3.3 V

  Wire.begin(PIN_SDA, PIN_SCL);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_DIRECCION);
  if (oledOk) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    imprimirCentrado("Calibrando", 1, 24);
    imprimirCentrado("no toques el joystick", 1, 36);
    oled.display();
  } else {
    Serial.println("OLED no encontrada: revisa SDA=21, SCL=22 y la direccion 0x3C");
  }

  calibrarCentro();
  Serial.println("Selector listo");
}

void loop() {
  unsigned long ahora = millis();

  // ---------- Botón con antirrebote (dispara al presionar) ----------
  bool lectura = digitalRead(PIN_SW);
  if (lectura != swUltimaLectura) {
    swUltimaLectura = lectura;
    tCambioSw = ahora;
  }
  if (ahora - tCambioSw >= T_ANTIRREBOTE && lectura != swEstable) {
    swEstable = lectura;
    if (swEstable == LOW && !enviando) enviarCaracter();
  }

  // ---------- Fin del mensaje "Enviando..." ----------
  if (enviando && ahora - tEnvio >= T_ENVIANDO) {
    enviando = false;
    redibujar = true;
  }

  // ---------- Scroll con umbral, rearme y repetición al sostener ----------
  if (!enviando) {
    int y = leerY() - centroY;
    if (INVERTIR_Y) y = -y;
    int direccion = 0;
    if (y < -UMBRAL) direccion = +1;        // stick hacia valores bajos: siguiente
    else if (y > UMBRAL) direccion = -1;    // stick hacia valores altos: anterior

    if (direccion != 0) {
      if (direccionActiva == 0) {                     // toque nuevo: avanza uno
        cambiarCaracter(direccion);
        direccionActiva = direccion;
        primeraRepeticion = true;
        tUltimoPaso = ahora;
      } else if (direccion == direccionActiva) {      // sostenido: avanza solo, más lento al principio
        unsigned long espera = primeraRepeticion ? T_PRIMERA_REPETICION : T_REPETICION;
        if (ahora - tUltimoPaso >= espera) {
          cambiarCaracter(direccion);
          primeraRepeticion = false;
          tUltimoPaso = ahora;
        }
      }
    } else if (abs(y) < REARME) {
      direccionActiva = 0;                            // volvió al centro: listo para otro toque
    }
  }

  // ---------- Pantalla (solo cuando algo cambió) ----------
  if (redibujar && oledOk) {
    if (enviando) pantallaEnviando();
    else pantallaSeleccion();
    redibujar = false;
  }
}
