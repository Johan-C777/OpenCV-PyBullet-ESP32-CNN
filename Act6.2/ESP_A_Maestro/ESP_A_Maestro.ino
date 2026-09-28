/* ESP-A | MAESTRO SPI (ESP32-C3 Mini) */
#include <SPI.h>

// Pines SPI para la C3 Mini
const int PIN_SCK  = 4;
const int PIN_MISO = 5;
const int PIN_MOSI = 6;
const int PIN_CS   = 7;

// Protocolo compatible con el Esclavo
const uint8_t CAB_MAESTRO = 0xA5, COLA_MAESTRO = 0x5A;
const uint8_t CAB_ESCLAVO = 0xB6, COLA_ESCLAVO = 0x6B;
const uint8_t CONSULTA = 0xFF;
const uint8_t NINGUNO  = 0xFF;
const uint32_t SPI_HZ  = 1000000;
const int INTENTOS     = 3;
const unsigned long T_SINCRONIZAR = 2000;

uint8_t ultimoDigito = NINGUNO;
bool esclavoRespondia = true;
unsigned long tSincronizacion = 0;

char linea[16];
int largoLinea = 0;

void transferir(uint8_t dato, uint8_t respuesta[4]) {
  uint8_t trama[4] = { CAB_MAESTRO, dato, (uint8_t)~dato, COLA_MAESTRO };
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(20);
  for (int i = 0; i < 4; i++) respuesta[i] = SPI.transfer(trama[i]);
  delayMicroseconds(5);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

int consultarEsclavo() {
  uint8_t r[4];
  transferir(CONSULTA, r);
  bool valida = r[0] == CAB_ESCLAVO && r[3] == COLA_ESCLAVO && (uint8_t)(r[1] ^ r[2]) == 0xFF;
  return valida ? r[1] : -1;
}

bool enviarDigito(uint8_t d) {
  uint8_t basura[4];
  for (int intento = 0; intento < INTENTOS; intento++) {
    transferir(d, basura);
    delay(10);
    if (consultarEsclavo() == d) return true;
    delay(20);
  }
  return false;
}

void procesarLinea() {
  linea[largoLinea] = '\0';
  for (int i = 0; i < largoLinea; i++) {
    char c = linea[i];
    if (c == ' ' || c == '\r' || c == '\t') continue;
    if (c >= '0' && c <= '9') {
      uint8_t d = c - '0';
      ultimoDigito = d;
      bool ok = enviarDigito(d);
      esclavoRespondia = ok;
      Serial.printf("%s:%d\n", ok ? "OK" : "ERR", d);
      tSincronizacion = millis();
    }
    break;
  }
  largoLinea = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  
  // Inicialización del bus SPI en la C3
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  Serial.println("ESP-A (C3 Mini) listo (Maestro SPI)");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') procesarLinea();
    else if (largoLinea < (int)sizeof(linea) - 1) linea[largoLinea++] = c;
    else largoLinea = 0;
  }

  if (millis() - tSincronizacion >= T_SINCRONIZAR) {
    tSincronizacion = millis();
    int mostrado = consultarEsclavo();
    if (ultimoDigito != NINGUNO && mostrado != ultimoDigito) {
      bool ok = enviarDigito(ultimoDigito);
      if (ok != esclavoRespondia || ok) {
        Serial.printf("%s:%d\n", ok ? "OK" : "ERR", ultimoDigito);
      }
      esclavoRespondia = ok;
    }
  }
}