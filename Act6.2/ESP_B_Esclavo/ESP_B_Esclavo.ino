/*
  ESP-B  |  ESCLAVO SPI (VSPI) + OLED SSD1306 I2C
  Recibe del ESP-A el dígito reconocido por la CNN y lo muestra gigante y centrado.

  CONEXIONES
    Bus SPI con el ESP-A (pin con pin):  MOSI 23 | MISO 19 | SCK 18 | CS 5 | GND con GND (obligatorio)
    OLED 128x64:  VCC -> 3V3   GND -> GND   SDA -> GPIO 21   SCL -> GPIO 22   (dirección 0x3C)

  LIBRERÍA: "U8g2" (by oliver) desde el Gestor de librerías.
  El esclavo SPI usa el driver oficial del ESP-IDF (driver/spi_slave.h), que ya viene
  con el paquete de placas ESP32; no requiere librerías extra.

  TRAMAS (4 bytes, SPI modo 0, MSB primero)
    Maestro -> Esclavo : A5 | dato | ~dato | 5A     dato = 0..9 (dígito) o FF (consulta)
    Esclavo -> Maestro : B6 | dígito mostrado (FF = ninguno) | ~dígito | 6B
    (en SPI la respuesta viaja en la transacción SIGUIENTE: por eso el maestro
     envía el dígito y luego una consulta para confirmar)
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"

// ---------------- Pines ----------------
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK  18
#define PIN_CS   5
#define PIN_SDA  21
#define PIN_SCL  22
#define BUS_SPI  SPI3_HOST          // SPI3_HOST = VSPI en el ESP32

// ---------------- Protocolo ----------------
const uint8_t CAB_MAESTRO  = 0xA5, COLA_MAESTRO  = 0x5A;
const uint8_t CAB_ESCLAVO  = 0xB6, COLA_ESCLAVO  = 0x6B;
const uint8_t CONSULTA     = 0xFF;
const uint8_t NINGUNO      = 0xFF;
const int     TAM_TRAMA    = 4;
const unsigned long T_SIN_MAESTRO = 5000;   // ms sin tramas -> aviso "sin maestro"

WORD_ALIGNED_ATTR uint8_t bufRx[TAM_TRAMA];
WORD_ALIGNED_ATTR uint8_t bufTx[TAM_TRAMA];
spi_slave_transaction_t transaccion;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);

uint8_t digitoActual = NINGUNO;
unsigned long tUltimaTrama = 0;
unsigned long tramasValidas = 0, tramasInvalidas = 0;
bool maestroActivo = false;

// ============================================================
void prepararRespuesta() {
  bufTx[0] = CAB_ESCLAVO;
  bufTx[1] = digitoActual;
  bufTx[2] = (uint8_t)~digitoActual;
  bufTx[3] = COLA_ESCLAVO;
}

void encolarTransaccion() {
  // El esclavo solo recibe si ya tiene una transacción en cola cuando el maestro baja CS
  memset(bufRx, 0, sizeof(bufRx));
  memset(&transaccion, 0, sizeof(transaccion));
  transaccion.length = TAM_TRAMA * 8;       // en bits
  transaccion.tx_buffer = bufTx;
  transaccion.rx_buffer = bufRx;
  spi_slave_queue_trans(BUS_SPI, &transaccion, portMAX_DELAY);
}

void mostrarPantalla() {
  oled.clearBuffer();

  // Estado pequeño en las esquinas (no estorba al dígito, que va centrado)
  oled.setFont(u8g2_font_5x7_tf);
  oled.drawStr(0, 7, "SPI");
  if (!maestroActivo) {                    // columna izquierda: no choca con el dígito centrado
    oled.drawStr(0, 55, "sin");
    oled.drawStr(0, 63, "enlace");
  }

  if (digitoActual <= 9) {
    char texto[2] = { (char)('0' + digitoActual), '\0' };
    oled.setFont(u8g2_font_logisoso58_tn);  // números de ~58 px de alto
    int ancho = oled.getStrWidth(texto);
    int alto = oled.getAscent();
    oled.drawStr((128 - ancho) / 2, (64 + alto) / 2, texto);
  } else {
    oled.setFont(u8g2_font_6x12_tf);
    const char *l1 = "Esperando digito";
    const char *l2 = "desde el ESP-A...";
    oled.drawStr((128 - oled.getStrWidth(l1)) / 2, 30, l1);
    oled.drawStr((128 - oled.getStrWidth(l2)) / 2, 44, l2);
  }
  oled.sendBuffer();
}

// ============================================================
void setup() {
  Serial.begin(115200);                      // solo para depurar si conectas el ESP-B al PC

  oled.setBusClock(400000);
  oled.setI2CAddress(0x3C * 2);
  oled.begin();
  mostrarPantalla();

  // Pull-ups: evitan que ruido en cables sueltos parezca una trama cuando el maestro no está
  gpio_set_pull_mode((gpio_num_t)PIN_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_SCK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_CS, GPIO_PULLUP_ONLY);

  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = PIN_MISO;
  bus.sclk_io_num = PIN_SCK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;

  spi_slave_interface_config_t esclavo = {};
  esclavo.spics_io_num = PIN_CS;
  esclavo.flags = 0;
  esclavo.queue_size = 1;
  esclavo.mode = 0;                          // debe coincidir con SPI_MODE0 del maestro

  esp_err_t err = spi_slave_initialize(BUS_SPI, &bus, &esclavo, SPI_DMA_DISABLED);
  if (err != ESP_OK) {
    Serial.printf("Error iniciando SPI esclavo: %s\n", esp_err_to_name(err));
  }

  prepararRespuesta();
  encolarTransaccion();
  Serial.println("ESP-B listo (esclavo SPI)");
}

void loop() {
  spi_slave_transaction_t *hecha;
  if (spi_slave_get_trans_result(BUS_SPI, &hecha, pdMS_TO_TICKS(20)) == ESP_OK) {
    uint8_t trama[TAM_TRAMA];
    memcpy(trama, bufRx, TAM_TRAMA);
    bool completa = hecha->trans_len >= TAM_TRAMA * 8;
    bool valida = completa && trama[0] == CAB_MAESTRO && trama[3] == COLA_MAESTRO
                  && (uint8_t)(trama[1] ^ trama[2]) == 0xFF;

    bool redibujar = false;
    if (valida) {
      tramasValidas++;
      tUltimaTrama = millis();
      if (!maestroActivo) { maestroActivo = true; redibujar = true; }
      if (trama[1] <= 9 && trama[1] != digitoActual) {
        digitoActual = trama[1];
        redibujar = true;
        Serial.printf("Digito recibido por SPI: %d\n", digitoActual);
      }
    } else {
      tramasInvalidas++;
    }

    // Primero dejar lista la siguiente transacción (con la respuesta actualizada)
    // y después actualizar la OLED, que tarda ~25 ms por I2C.
    prepararRespuesta();
    encolarTransaccion();
    if (redibujar) mostrarPantalla();
  }

  if (maestroActivo && millis() - tUltimaTrama > T_SIN_MAESTRO) {
    maestroActivo = false;                   // el ESP-A envía un latido cada 2 s; si no llega nada, se avisa
    mostrarPantalla();
  }
}
