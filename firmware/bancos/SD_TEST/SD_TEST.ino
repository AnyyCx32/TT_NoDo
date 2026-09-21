// ============================================================================
//  SD_TEST - BANCO DE PRUEBAS EXCLUSIVO DE LA microSD DEL NODO R11
//  Raspberry Pi Pico (RP2040) + microSD por SPI0
//
//  MOTIVO:
//    En el nodo con el firmware de operacion, SD.begin() falla en los cinco
//    intentos y el registro queda apagado mientras el resto del sistema opera.
//    SD.begin() devuelve un unico falso y no dice en que punto se rompio la
//    cadena. Este programa parte esa cadena en capas independientes:
//
//      pines -> la tarjeta responde a SPI crudo -> inicializacion SD ->
//      sistema de archivos -> lectura y escritura real
//
//    Cada capa se puede correr sola. La primera que falle senala al culpable:
//
//      Falla el nivel 1 (MISO en bajo con pull-up)  -> corto o tarjeta trabada
//      Falla el nivel 2 (CMD0 nunca contesta 0x01)  -> cableado, alimentacion
//                                                      o tarjeta ausente/muerta
//      Pasa 2 y falla 3 (responde pero no monta)    -> formato o particion
//      Pasa 3 y falla 5 (escritura)                 -> tarjeta gastada o
//                                                      protegida contra escritura
//
//  NO toca sensores, LTE, OLED ni navegacion. Solo la microSD.
//
//  PINES: los mismos de ProgramaFinal_R11_General_ENU.ino. Ese programa solo
//  nombra CS y deja que el core rp2040 asigne SPI0 por omision, asi que aqui se
//  escriben los cuatro para poder verificarlos: CS=GP17, MISO=GP16, SCK=GP18,
//  MOSI=GP19.
//
//  USO: consola USB a 115200. Al arrancar corre la prueba completa; 'h' repite
//  el menu.
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>

// ============================================================================
// PINES Y PARAMETROS
// ============================================================================
static uint8_t csPin   = 17;
static uint8_t misoPin = 16;
static uint8_t sckPin  = 18;
static uint8_t mosiPin = 19;

// Velocidad del bus para las pruebas con libreria. La inicializacion cruda
// siempre ocurre a 400 kHz, como manda la especificacion SD.
static uint32_t spiKhz = 4000;
static const uint32_t INIT_KHZ = 400;

// El LED de a bordo late mientras el programa vive, para distinguir un cuelgue
// de una prueba lenta.
static const uint8_t LED_PIN = 25;

// ============================================================================
// ESTADO
// ============================================================================
static bool montada = false;       // la libreria SD tiene un volumen montado
static bool tarjetaCruda = false;  // la tarjeta respondio a la sonda SPI
// La tarjeta contesta, pero con respuestas que se contradicen entre si. Pasa
// cuando se reinicio la Pico sin cortarle la alimentacion: la tarjeta quedo a
// media transferencia y ya no dice nada coherente. No es un diagnostico, es una
// senal de que hay que repetir la prueba desde un arranque en frio.
static bool estadoTrabado = false;
static uint8_t tipoTarjeta = 0;    // 1=SDSC v1, 2=SDSC v2, 3=SDHC/SDXC
static uint64_t capacidadBytes = 0;

// ============================================================================
// UTILERIAS DE IMPRESION
// ============================================================================
void titulo(const char *texto) {
  Serial.println();
  Serial.println("============================================================");
  Serial.print("  ");
  Serial.println(texto);
  Serial.println("============================================================");
}

void resultado(const char *etiqueta, bool ok, const char *detalle = nullptr) {
  Serial.print(ok ? "  [OK]    " : "  [FALLA] ");
  Serial.print(etiqueta);
  if (detalle != nullptr && detalle[0] != '\0') {
    Serial.print(" -> ");
    Serial.print(detalle);
  }
  Serial.println();
}

void imprimeHex(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

// ============================================================================
// NIVEL 1 - ESTADO ELECTRICO DE LOS PINES
// ============================================================================
// Sin SPI todavia. Se observa MISO con las resistencias internas del RP2040.
// Una microSD alimentada y con CS en alto deja MISO en alta impedancia, asi que
// el pin debe seguir a la resistencia interna. Si no la sigue, el defecto es
// fisico y no tiene caso subir de nivel.
bool nivelPines() {
  titulo("NIVEL 1 - ESTADO ELECTRICO DE LOS PINES");

  Serial.print("  CS=GP");   Serial.print(csPin);
  Serial.print("  MISO=GP"); Serial.print(misoPin);
  Serial.print("  SCK=GP");  Serial.print(sckPin);
  Serial.print("  MOSI=GP"); Serial.println(mosiPin);

  SD.end(true);
  montada = false;
  SPI.end();

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  pinMode(sckPin, OUTPUT);
  digitalWrite(sckPin, LOW);
  pinMode(mosiPin, OUTPUT);
  digitalWrite(mosiPin, HIGH);

  pinMode(misoPin, INPUT_PULLUP);
  delay(5);
  bool conPullup = digitalRead(misoPin);

  pinMode(misoPin, INPUT_PULLDOWN);
  delay(5);
  bool conPulldown = digitalRead(misoPin);

  pinMode(misoPin, INPUT);

  Serial.print("  MISO con pull-up interno:   ");
  Serial.println(conPullup ? "ALTO" : "BAJO");
  Serial.print("  MISO con pull-down interno: ");
  Serial.println(conPulldown ? "ALTO" : "BAJO");

  bool ok = conPullup;
  if (!ok) {
    resultado("MISO sigue al pull-up", false,
              "clavado en bajo: corto a GND, tarjeta trabada o alguien mas "
              "tomando el bus");
  } else {
    resultado("MISO sigue al pull-up", true);
  }

  if (conPulldown) {
    // El modulo y la tarjeta insertada ponen su propio pull-up en MISO, asi que
    // la linea gana contra el pull-down interno. Es la senal de que hay algo
    // conectado del otro lado.
    Serial.println("  MISO se queda alto aun con el pull-down interno: del otro");
    Serial.println("  lado hay un pull-up, o sea modulo y tarjeta haciendo");
    Serial.println("  contacto. (Un corto a 3V3 daria lo mismo, pero es raro.)");
  } else if (conPullup) {
    // Seguir a las dos resistencias internas significa que la linea esta
    // flotando: nadie la sostiene. Con una tarjeta bien asentada eso no pasa,
    // porque el modulo la mantiene arriba.
    Serial.println("  ATENCION: MISO sigue tanto al pull-up como al pull-down,");
    Serial.println("  o sea que esta flotando. Nada lo sostiene del otro lado.");
    Serial.println("  Con una tarjeta bien asentada la linea se queda arriba");
    Serial.println("  sola. Esto apunta a tarjeta ausente o mal insertada, o al");
    Serial.println("  cable de MISO (DO del modulo -> GP16) suelto.");
  }

  if (ok) {
    Serial.println("  Los pines no muestran un defecto evidente. Esta prueba no");
    Serial.println("  detecta un cable abierto: para eso esta el nivel 2, que");
    Serial.println("  exige respuesta de la tarjeta.");
  }
  return ok;
}

// ============================================================================
// NIVEL 2 - SONDA SPI CRUDA
// ============================================================================
// Se le habla a la tarjeta sin libreria, con la secuencia de arranque del modo
// SPI que define la especificacion SD. Es la prueba decisiva: separa "la
// tarjeta no contesta" de "contesta pero la libreria no la monta".
static void csBajo() {
  digitalWrite(csPin, LOW);
}

static void csAlto() {
  digitalWrite(csPin, HIGH);
  SPI.transfer(0xFF); // ocho relojes extra para que la tarjeta suelte el bus
}

// Envia un comando y devuelve R1. Los bytes adicionales de R3/R7 se guardan en
// 'extra' cuando se piden.
uint8_t comandoSd(uint8_t cmd, uint32_t arg, uint8_t crc,
                  uint8_t *extra = nullptr, uint8_t nExtra = 0) {
  SPI.transfer(0xFF);
  SPI.transfer(0x40 | cmd);
  SPI.transfer((uint8_t)(arg >> 24));
  SPI.transfer((uint8_t)(arg >> 16));
  SPI.transfer((uint8_t)(arg >> 8));
  SPI.transfer((uint8_t)arg);
  SPI.transfer(crc);

  uint8_t r1 = 0xFF;
  for (uint8_t i = 0; i < 20 && (r1 & 0x80); ++i) {
    r1 = SPI.transfer(0xFF);
  }
  for (uint8_t i = 0; i < nExtra; ++i) {
    extra[i] = SPI.transfer(0xFF);
  }
  return r1;
}

// Lee un bloque de datos precedido por el token 0xFE.
bool leeBloque(uint8_t *destino, uint16_t largo) {
  uint8_t token = 0xFF;
  uint32_t limite = millis() + 300;
  while (millis() < limite) {
    token = SPI.transfer(0xFF);
    if (token != 0xFF) break;
  }
  if (token != 0xFE) return false;
  for (uint16_t i = 0; i < largo; ++i) {
    destino[i] = SPI.transfer(0xFF);
  }
  SPI.transfer(0xFF); // CRC de 16 bits, se descarta
  SPI.transfer(0xFF);
  return true;
}

// Deja el bus listo y lleva la tarjeta hasta el estado operativo. La usan el
// nivel 2 y el nivel 8.
bool preparaBusCrudo(bool &v2, uint8_t *ocrSalida) {
  // Tomar el bus en crudo desmonta el volumen. Hay que anotarlo: si no, los
  // niveles que siguen creerian que la tarjeta sigue montada y abririan
  // archivos contra un sistema de archivos que ya no esta.
  SD.end(true);
  montada = false;
  delay(50);

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  SPI.setRX(misoPin);
  SPI.setCS(csPin);
  SPI.setSCK(sckPin);
  SPI.setTX(mosiPin);
  SPI.begin(false);
  SPI.beginTransaction(SPISettings(INIT_KHZ * 1000UL, MSBFIRST, SPI_MODE0));

  // Secuencia de encendido: al menos 74 relojes con CS en alto.
  digitalWrite(csPin, HIGH);
  for (uint8_t i = 0; i < 12; ++i) SPI.transfer(0xFF);

  // Se reintenta con un tren de relojes previo: si la Pico se reinicio a la
  // mitad de una transferencia, la tarjeta sigue alimentada y hay que sacarla
  // de ese estado antes de que acepte CMD0.
  uint8_t r1Cmd0 = 0xFF;
  bool responde = false;
  for (uint8_t intento = 1; intento <= 10 && !responde; ++intento) {
    digitalWrite(csPin, HIGH);
    for (uint8_t i = 0; i < 12; ++i) SPI.transfer(0xFF);
    csBajo();
    r1Cmd0 = comandoSd(0, 0, 0x95);
    csAlto();
    responde = (r1Cmd0 == 0x01);
    if (!responde) delay(20);
  }
  // R1=0x00 es la tarjeta contestando ya inicializada; sirve igual.
  if (!responde && r1Cmd0 != 0x00) return false;

  uint8_t r7[4];
  csBajo();
  v2 = (comandoSd(8, 0x000001AA, 0x87, r7, 4) == 0x01);
  csAlto();

  uint32_t arranque = millis();
  bool lista = false;
  while (millis() - arranque < 1500) {
    csBajo();
    comandoSd(55, 0, 0x65);
    lista = (comandoSd(41, v2 ? 0x40000000UL : 0, 0x77) == 0x00);
    csAlto();
    if (lista) break;
    delay(10);
  }
  if (!lista) return false;

  if (ocrSalida != nullptr) {
    csBajo();
    comandoSd(58, 0, 0xFD, ocrSalida, 4);
    csAlto();
  }
  return true;
}

bool nivelSondaCruda() {
  titulo("NIVEL 2 - SONDA SPI CRUDA (SIN LIBRERIA)");

  tarjetaCruda = false;
  estadoTrabado = false;
  tipoTarjeta = 0;
  capacidadBytes = 0;

  // La libreria pudo dejar el bus tomado en una prueba anterior.
  SD.end(true);
  montada = false;
  delay(50);

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  SPI.setRX(misoPin);
  SPI.setCS(csPin);
  SPI.setSCK(sckPin);
  SPI.setTX(mosiPin);
  SPI.begin(false);
  SPI.beginTransaction(SPISettings(INIT_KHZ * 1000UL, MSBFIRST, SPI_MODE0));

  // Antes de hablarle a nadie: que hay en MISO con CS en alto. Con el bus en
  // reposo y la tarjeta sin seleccionar, nadie debe estar manejando la linea, y
  // el pull-up la mantiene arriba: lo que debe volver es 0xFF.
  //
  // Esta lectura separa dos cosas que producen sintomas identicos mas adelante.
  // Si todo vuelve en 0x00, cada respuesta posterior sera 0x00 y el programa
  // creera que la tarjeta contesta: CMD0 "responde", ACMD41 "queda lista", el
  // OCR sale en ceros. Nada de eso seria cierto; seria la linea clavada en bajo
  // o el pin de recepcion mal asignado.
  digitalWrite(csPin, HIGH);
  uint8_t eco[8];
  for (uint8_t i = 0; i < 12; ++i) {
    uint8_t b = SPI.transfer(0xFF);
    if (i >= 4) eco[i - 4] = b;
  }

  bool todoCero = true, todoUno = true;
  for (uint8_t i = 0; i < 8; ++i) {
    if (eco[i] != 0x00) todoCero = false;
    if (eco[i] != 0xFF) todoUno = false;
  }

  Serial.print("  Bus en reposo (CS alto, 8 bytes): ");
  for (uint8_t i = 0; i < 8; ++i) {
    imprimeHex(eco[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("  Nivel del pin MISO leido como GPIO: ");
  Serial.println(digitalRead(misoPin) ? "ALTO" : "BAJO");

  if (todoUno) {
    resultado("Bus en reposo en alto, como debe ser", true);
  } else if (todoCero) {
    resultado("Bus en reposo", false, "MISO clavado en bajo con CS en alto");
    Serial.println();
    Serial.println("  Con la tarjeta sin seleccionar nadie debe manejar MISO, y");
    Serial.println("  el pull-up deberia dejarlo en 0xFF. Que llegue 0x00 quiere");
    Serial.println("  decir que la linea esta forzada a cero. A partir de aqui");
    Serial.println("  toda respuesta valdria 0x00 y pareceria que la tarjeta");
    Serial.println("  contesta: no hay que creerle a nada de lo que sigue.");
    Serial.println("  Causas: MISO en corto a GND, el pin de recepcion mal");
    Serial.println("  asignado, o el modulo dejando la salida tomada.");
    SPI.endTransaction();
    estadoTrabado = true;
    return false;
  } else {
    resultado("Bus en reposo", false, "MISO devuelve un patron que no es reposo");
    Serial.println("  El bus no esta ni en alto ni en bajo estable. Suele ser");
    Serial.println("  ruido, cables largos o una tarjeta que no solto la linea.");
  }

  // --- CMD0: ir a estado inactivo. La tarjeta debe contestar 0x01 ---
  uint8_t r1 = 0xFF;
  bool idle = false;
  for (uint8_t intento = 1; intento <= 10 && !idle; ++intento) {
    // Cada reintento arranca con CS en alto y relojes libres: es lo que
    // destraba a una tarjeta que quedo a media transferencia cuando la Pico se
    // reinicio sin cortarle la alimentacion.
    digitalWrite(csPin, HIGH);
    for (uint8_t i = 0; i < 12; ++i) SPI.transfer(0xFF);
    csBajo();
    r1 = comandoSd(0, 0, 0x95);
    csAlto();
    if (r1 == 0x01) idle = true;
    else delay(20);
  }

  Serial.print("  CMD0 (GO_IDLE_STATE) -> R1 = 0x");
  imprimeHex(r1);
  Serial.println();

  if (!idle && r1 == 0x00) {
    // La tarjeta esta ahi y contesta; solo no acepto volver a inactivo porque
    // nunca se le corto la alimentacion. Se sigue adelante.
    resultado("CMD0: la tarjeta contesta (R1=0x00, ya inicializada)", true);
    Serial.println("  No volvio a estado inactivo porque sigue alimentada desde");
    Serial.println("  la sesion anterior. Para una prueba limpia hay que");
    Serial.println("  desconectar el nodo, no solo reiniciar la Pico.");
    idle = true;
  }

  if (!idle) {
    if (r1 == 0xFF) {
      resultado("La tarjeta responde a CMD0", false, "silencio total en MISO");
      Serial.println();
      Serial.println("  La tarjeta no esta hablando. Causas en orden de");
      Serial.println("  probabilidad:");
      Serial.println("    1. No hay tarjeta insertada, o no hace contacto en el");
      Serial.println("       zocalo.");
      Serial.println("    2. MISO, MOSI, SCK o CS abierto o intercambiado.");
      Serial.println("    3. El modulo no recibe 3V3, o su regulador esta");
      Serial.println("       caido.");
      Serial.println("    4. Tarjeta muerta.");
    } else {
      resultado("La tarjeta responde a CMD0", false,
                "respuesta fuera de lo esperado");
    }
    SPI.endTransaction();
    return false;
  }

  if (r1 == 0x01) {
    resultado("CMD0: la tarjeta entro en estado inactivo", true);
  }

  // --- CMD8: version de la interfaz y rango de voltaje ---
  uint8_t r7[4] = {0, 0, 0, 0};
  csBajo();
  r1 = comandoSd(8, 0x000001AA, 0x87, r7, 4);
  csAlto();

  bool v2 = false;
  if (r1 == 0x01) {
    if (r7[2] == 0x01 && r7[3] == 0xAA) {
      v2 = true;
      resultado("CMD8: tarjeta v2.0 o mayor, acepta 2.7-3.6 V", true);
    } else {
      resultado("CMD8: eco del patron de voltaje", false,
                "la tarjeta no acepta 3.3 V o el bus tiene ruido");
      SPI.endTransaction();
      return false;
    }
  } else {
    resultado("CMD8: tarjeta v1.x (SDSC antigua)", true);
  }

  // --- ACMD41: salir de inactivo. Aqui se cae una tarjeta con alimentacion
  //     insuficiente: contesta CMD0 y nunca termina esta etapa. ---
  uint32_t arranque = millis();
  bool lista = false;
  while (millis() - arranque < 1500) {
    csBajo();
    comandoSd(55, 0, 0x65);
    r1 = comandoSd(41, v2 ? 0x40000000UL : 0, 0x77);
    csAlto();
    if (r1 == 0x00) {
      lista = true;
      break;
    }
    delay(10);
  }

  if (!lista) {
    Serial.print("  ACMD41 -> R1 = 0x");
    imprimeHex(r1);
    Serial.println();
    resultado("ACMD41: la tarjeta completo su inicializacion", false,
              "nunca salio de estado inactivo");
    Serial.println("  Sintoma tipico de 3V3 debil o con rizado, o de tarjeta");
    Serial.println("  degradada. Conviene repetir la prueba alimentando por USB");
    Serial.println("  y por bateria, y comparar.");
    SPI.endTransaction();
    return false;
  }
  Serial.print("  ACMD41 completo en ");
  Serial.print(millis() - arranque);
  Serial.println(" ms");
  resultado("ACMD41: la tarjeta quedo lista", true);

  // --- CMD58: OCR. El bit CCS distingue SDHC/SDXC de SDSC ---
  uint8_t ocr[4] = {0, 0, 0, 0};
  csBajo();
  r1 = comandoSd(58, 0, 0xFD, ocr, 4);
  csAlto();
  bool ocrVacio = (ocr[0] == 0 && ocr[1] == 0 && ocr[2] == 0 && ocr[3] == 0);
  if (r1 == 0x00 && !ocrVacio) {
    bool ccs = (ocr[0] & 0x40) != 0;
    tipoTarjeta = v2 ? (ccs ? 3 : 2) : 1;
    Serial.print("  OCR = 0x");
    for (uint8_t i = 0; i < 4; ++i) imprimeHex(ocr[i]);
    Serial.println();
    Serial.print("  Tipo: ");
    if (tipoTarjeta == 3) {
      Serial.println("SDHC / SDXC (direccionamiento por bloque)");
    } else if (tipoTarjeta == 2) {
      Serial.println("SDSC v2 (direccionamiento por byte)");
    } else {
      Serial.println("SDSC v1");
    }
    resultado("CMD58: registro OCR leido", true);
  } else if (ocrVacio) {
    // El bit 31 del OCR vale 1 en toda tarjeta que termino de encender. Un OCR
    // en ceros no lo produce ninguna tarjeta, viva o muerta: es la linea
    // contestando por inercia.
    resultado("CMD58: registro OCR leido", false,
              "OCR todo en ceros, respuesta imposible");
    estadoTrabado = true;
  } else {
    resultado("CMD58: registro OCR leido", false, "sin respuesta");
    estadoTrabado = true;
  }

  // --- CMD9: CSD, de donde sale la capacidad real de la tarjeta ---
  uint8_t csd[16];
  csBajo();
  r1 = comandoSd(9, 0, 0xFF);
  bool csdOk = (r1 == 0x00) && leeBloque(csd, 16);
  csAlto();

  if (csdOk) {
    uint8_t estructura = csd[0] >> 6;
    if (estructura == 1) {
      uint32_t cSize = ((uint32_t)(csd[7] & 0x3F) << 16) |
                       ((uint32_t)csd[8] << 8) | csd[9];
      capacidadBytes = (uint64_t)(cSize + 1) * 512ULL * 1024ULL;
    } else {
      uint16_t cSize = ((uint16_t)(csd[6] & 0x03) << 10) |
                       ((uint16_t)csd[7] << 2) | (csd[8] >> 6);
      uint8_t cSizeMult = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
      uint8_t readBlLen = csd[5] & 0x0F;
      capacidadBytes = (uint64_t)(cSize + 1) *
                       (1ULL << (cSizeMult + 2)) *
                       (1ULL << readBlLen);
    }
    Serial.print("  Capacidad segun CSD: ");
    Serial.print((uint32_t)(capacidadBytes / (1024ULL * 1024ULL)));
    Serial.println(" MB");
    // Se imprime el registro crudo: una capacidad rara solo se puede juzgar
    // viendo si los 16 bytes tienen forma de CSD o son ruido.
    Serial.print("  CSD crudo = ");
    for (uint8_t i = 0; i < 16; ++i) {
      imprimeHex(csd[i]);
      Serial.print(' ');
    }
    Serial.println();
    resultado("CMD9: registro CSD leido", true);
  } else {
    resultado("CMD9: registro CSD leido", false, "sin respuesta");
    estadoTrabado = true;
  }

  // --- CMD10: CID, la identidad del fabricante. Sirve para dejar constancia
  //     de con cual tarjeta se hizo cada sesion. ---
  uint8_t cid[16];
  csBajo();
  r1 = comandoSd(10, 0, 0xFF);
  bool cidOk = (r1 == 0x00) && leeBloque(cid, 16);
  csAlto();

  if (cidOk) {
    Serial.print("  Fabricante (MID) = 0x");
    imprimeHex(cid[0]);
    Serial.print("   OEM = ");
    Serial.print((char)cid[1]);
    Serial.println((char)cid[2]);
    Serial.print("  Producto = ");
    for (uint8_t i = 3; i <= 7; ++i) Serial.print((char)cid[i]);
    Serial.print("   rev ");
    Serial.print(cid[8] >> 4);
    Serial.print('.');
    Serial.println(cid[8] & 0x0F);
    uint32_t serie = ((uint32_t)cid[9] << 24) | ((uint32_t)cid[10] << 16) |
                     ((uint32_t)cid[11] << 8) | cid[12];
    Serial.print("  Numero de serie = 0x");
    Serial.println(serie, HEX);
    uint16_t mdt = ((uint16_t)(cid[13] & 0x0F) << 8) | cid[14];
    Serial.print("  Fabricada en ");
    Serial.print(mdt & 0x0F);
    Serial.print('/');
    Serial.println(2000 + (mdt >> 4));
    Serial.print("  CID crudo = ");
    bool cidVacio = true;
    for (uint8_t i = 0; i < 16; ++i) {
      imprimeHex(cid[i]);
      Serial.print(' ');
      if (cid[i] != 0x00) cidVacio = false;
    }
    Serial.println();
    resultado("CMD10: registro CID leido", true);
    if (cidVacio) {
      // Toda tarjeta legitima trae fabricante, producto y serie grabados de
      // fabrica. Un CID en ceros no lo produce una tarjeta sana.
      Serial.println("  ALERTA: el CID esta completamente en ceros. Ninguna");
      Serial.println("  tarjeta legitima sale de fabrica asi. Apunta a tarjeta");
      Serial.println("  apocrifa o con el controlador danado.");
    }
  }

  SPI.endTransaction();
  tarjetaCruda = !estadoTrabado;

  Serial.println();
  if (estadoTrabado) {
    Serial.println("  ESTADO TRABADO: la tarjeta contesta, pero sus respuestas");
    Serial.println("  se contradicen. Ninguna tarjeta, ni sana ni muerta, da");
    Serial.println("  esto desde un arranque limpio.");
    Serial.println("  No hay diagnostico posible con estos datos. Desconecta el");
    Serial.println("  nodo del USB por completo, espera unos segundos, vuelve a");
    Serial.println("  conectarlo y repite la prueba. Reiniciar la Pico no basta:");
    Serial.println("  la tarjeta conserva la alimentacion y sigue trabada.");
    return false;
  }
  Serial.println("  La tarjeta responde al protocolo SD. Si el nivel 3 falla");
  Serial.println("  despues de esto, el problema no es fisico: esta en el");
  Serial.println("  formato, la particion o la velocidad del bus.");
  return true;
}

// ============================================================================
// NIVEL 3 - MONTAJE CON LA LIBRERIA, A VARIAS VELOCIDADES
// ============================================================================
// R11 monta con SD.begin(SD_CS_PIN) y se queda con la velocidad por omision. Si
// la tarjeta responde en crudo pero solo monta despacio, el defecto son cables
// largos, sin blindaje o mal referenciados a tierra.
bool intentaMontar(uint32_t khz, bool verboso) {
  SD.end(true);
  delay(60);
  bool ok = SD.begin(csPin, khz * 1000UL);
  if (verboso) {
    char detalle[24];
    snprintf(detalle, sizeof(detalle), "%lu kHz", (unsigned long)khz);
    resultado(ok ? "SD.begin monto la tarjeta" : "SD.begin fallo", ok, detalle);
  }
  montada = ok;
  return ok;
}

bool nivelMontaje() {
  titulo("NIVEL 3 - MONTAJE CON LA LIBRERIA SD");

  static const uint32_t velocidades[] = {400, 1000, 4000, 8000, 16000};
  uint32_t mejor = 0;

  for (uint8_t i = 0; i < sizeof(velocidades) / sizeof(velocidades[0]); ++i) {
    if (intentaMontar(velocidades[i], true)) {
      mejor = velocidades[i];
    }
  }

  if (mejor == 0) {
    Serial.println();
    if (tarjetaCruda) {
      Serial.println("  La tarjeta contesta el protocolo SD pero la libreria no");
      Serial.println("  la monta a ninguna velocidad. Eso apunta al sistema de");
      Serial.println("  archivos, no al cableado:");
      Serial.println("    - Formatear en FAT32 con una sola particion primaria.");
      Serial.println("    - Evitar exFAT: las tarjetas de mas de 32 GB vienen");
      Serial.println("      asi de fabrica y SdFat no las monta aqui.");
    } else {
      Serial.println("  No monta, y el nivel 2 tampoco obtuvo respuesta.");
      Serial.println("  Revisar primero lo fisico.");
    }
    montada = false;
    return false;
  }

  Serial.println();
  Serial.print("  Velocidad mas alta con montaje estable: ");
  Serial.print(mejor);
  Serial.println(" kHz");
  if (mejor < 4000) {
    Serial.println("  ADVERTENCIA: solo monta por debajo de 4 MHz. El nodo");
    Serial.println("  escribe vibracion en rafagas; a esta velocidad hay riesgo");
    Serial.println("  de perder muestras. Acortar los cables del bus.");
  }

  // Se deja montada a la velocidad de trabajo para los niveles siguientes.
  intentaMontar(spiKhz, false);
  if (!montada) intentaMontar(mejor, false);
  return montada;
}

// ============================================================================
// NIVEL 4 - INFORMACION DEL SISTEMA DE ARCHIVOS
// ============================================================================
bool nivelInfo() {
  titulo("NIVEL 4 - SISTEMA DE ARCHIVOS");

  if (!montada && !intentaMontar(spiKhz, false)) {
    resultado("Tarjeta montada", false, "no hay volumen que consultar");
    return false;
  }

  uint8_t fat = SD.fatType();
  Serial.print("  Tipo de FAT: FAT");
  Serial.println(fat);
  Serial.print("  Tamano de bloque: ");
  Serial.print((uint32_t)SD.blockSize());
  Serial.println(" bytes");
  Serial.print("  Bloques por cluster: ");
  Serial.println((uint32_t)SD.blocksPerCluster());
  Serial.print("  Clusters totales: ");
  Serial.println((uint32_t)SD.totalClusters());
  Serial.print("  Capacidad del volumen: ");
  Serial.print((uint32_t)(SD.size64() / (1024ULL * 1024ULL)));
  Serial.println(" MB");

  if (capacidadBytes > 0) {
    uint32_t mbCsd = (uint32_t)(capacidadBytes / (1024ULL * 1024ULL));
    uint32_t mbFs = (uint32_t)(SD.size64() / (1024ULL * 1024ULL));
    // Un volumen mucho mas chico que la tarjeta suele ser una particion vieja
    // que sobrevivio a un formateo rapido.
    if (mbFs > 0 && mbCsd > mbFs + (mbCsd / 5)) {
      Serial.println("  NOTA: el volumen es bastante menor que la tarjeta");
      Serial.println("        fisica. Puede quedar una particion antigua");
      Serial.println("        ocupando el resto.");
    }
  }

  resultado("Sistema de archivos legible", fat != 0);
  return fat != 0;
}

// ============================================================================
// NIVEL 5 - ESCRITURA Y LECTURA REAL
// ============================================================================
// Montar no es lo mismo que escribir. Esta prueba reproduce lo que hace el
// nodo: abrir en modo agregar, escribir lineas, cerrar y releer para comprobar
// que los bytes sobrevivieron al cierre.
bool nivelEscritura() {
  titulo("NIVEL 5 - ESCRITURA Y LECTURA");

  if (!montada && !intentaMontar(spiKhz, false)) {
    resultado("Tarjeta montada", false, "no se puede escribir");
    return false;
  }

  static const char RUTA[] = "/SDTEST.TXT";
  static const int LINEAS = 50;

  SD.remove(RUTA);

  File f = SD.open(RUTA, FILE_WRITE);
  if (!f) {
    resultado("Crear /SDTEST.TXT", false,
              "SD.open en modo escritura devolvio nulo");
    Serial.println("  Revisar el seguro de proteccion contra escritura del");
    Serial.println("  adaptador, si se esta usando uno.");
    return false;
  }

  uint32_t t0 = millis();
  for (int i = 1; i <= LINEAS; ++i) {
    f.print("linea ");
    f.print(i);
    f.println(" de prueba SD del nodo R11");
  }
  f.flush();
  uint32_t bytesEscritos = f.size();
  f.close();
  uint32_t msEscritura = millis() - t0;

  Serial.print("  Escritas ");
  Serial.print(LINEAS);
  Serial.print(" lineas (");
  Serial.print(bytesEscritos);
  Serial.print(" bytes) en ");
  Serial.print(msEscritura);
  Serial.println(" ms");
  resultado("Escritura y cierre", bytesEscritos > 0);

  // Relectura: se cuentan y verifican las lineas recuperadas.
  f = SD.open(RUTA, FILE_READ);
  if (!f) {
    resultado("Reabrir para lectura", false, "SD.open devolvio nulo");
    return false;
  }

  int leidas = 0;
  bool contenidoOk = true;
  while (f.available()) {
    String linea = f.readStringUntil('\n');
    linea.trim();
    if (linea.length() == 0) continue;
    leidas++;
    if (!linea.startsWith("linea ")) contenidoOk = false;
  }
  uint32_t tamano = f.size();
  f.close();

  Serial.print("  Releidas ");
  Serial.print(leidas);
  Serial.print(" lineas, ");
  Serial.print(tamano);
  Serial.println(" bytes");

  bool ok = (leidas == LINEAS) && contenidoOk && (tamano == bytesEscritos);
  resultado("Los datos sobrevivieron al cierre", ok,
            ok ? nullptr : "lo leido no coincide con lo escrito");

  SD.remove(RUTA);
  return ok;
}

// ============================================================================
// NIVEL 6 - VELOCIDAD SOSTENIDA
// ============================================================================
// El nodo escribe vibracion en rafagas. Si la tarjeta sostiene menos de unos
// pocos kB/s, la bitacora se vuelve el cuello de botella del lazo.
bool nivelVelocidad() {
  titulo("NIVEL 6 - VELOCIDAD DE ESCRITURA SOSTENIDA");

  if (!montada && !intentaMontar(spiKhz, false)) {
    resultado("Tarjeta montada", false, "no se puede medir");
    return false;
  }

  static const char RUTA[] = "/SDSPEED.BIN";
  static const uint16_t BLOQUE = 512;
  static const uint16_t BLOQUES = 128; // 64 kB en total

  static uint8_t buffer[BLOQUE];
  for (uint16_t i = 0; i < BLOQUE; ++i) buffer[i] = (uint8_t)(i & 0xFF);

  SD.remove(RUTA);
  File f = SD.open(RUTA, FILE_WRITE);
  if (!f) {
    resultado("Crear el archivo de medicion", false, "SD.open devolvio nulo");
    return false;
  }

  uint32_t t0 = millis();
  uint32_t peorBloqueMs = 0;
  uint32_t escritos = 0;
  for (uint16_t i = 0; i < BLOQUES; ++i) {
    uint32_t tb = millis();
    escritos += f.write(buffer, BLOQUE);
    uint32_t dt = millis() - tb;
    if (dt > peorBloqueMs) peorBloqueMs = dt;
  }
  f.flush();
  f.close();
  uint32_t dtTotal = millis() - t0;

  float kbs = (dtTotal > 0) ? (escritos / 1024.0f) / (dtTotal / 1000.0f) : 0.0f;

  Serial.print("  Escritos ");
  Serial.print(escritos);
  Serial.print(" bytes en ");
  Serial.print(dtTotal);
  Serial.println(" ms");
  Serial.print("  Tasa media: ");
  Serial.print(kbs, 1);
  Serial.println(" kB/s");
  // El peor bloque importa mas que la media: es la pausa que el lazo tendria
  // que absorber cuando la tarjeta reacomoda sus bloques internos.
  Serial.print("  Peor bloque de 512 B: ");
  Serial.print(peorBloqueMs);
  Serial.println(" ms");

  SD.remove(RUTA);

  bool ok = (escritos == (uint32_t)BLOQUE * BLOQUES);
  resultado("Escritura sostenida completa", ok);
  if (peorBloqueMs > 200) {
    Serial.println("  ADVERTENCIA: hubo una pausa mayor a 200 ms en un solo");
    Serial.println("  bloque. Una tarjeta gastada hace esto, y puede atorar el");
    Serial.println("  lazo del nodo.");
  }
  return ok;
}

// ============================================================================
// NIVEL 7 - CONTENIDO DE LA RAIZ
// ============================================================================
// Confirma que las sesiones anteriores dejaron archivos, y que la tarjeta
// puesta es la que se cree.
bool nivelListado() {
  titulo("NIVEL 7 - CONTENIDO DE LA RAIZ");

  if (!montada && !intentaMontar(spiKhz, false)) {
    resultado("Tarjeta montada", false, "no hay nada que listar");
    return false;
  }

  File raiz = SD.open("/");
  if (!raiz) {
    resultado("Abrir la raiz", false, "SD.open(\"/\") devolvio nulo");
    return false;
  }

  int cuenta = 0;
  uint32_t bytes = 0;
  File entrada = raiz.openNextFile();
  while (entrada) {
    Serial.print("  ");
    Serial.print(entrada.name());
    if (entrada.isDirectory()) {
      Serial.println("   <DIR>");
    } else {
      Serial.print("   ");
      Serial.print((uint32_t)entrada.size());
      Serial.println(" bytes");
      bytes += entrada.size();
    }
    cuenta++;
    entrada.close();
    entrada = raiz.openNextFile();
  }
  raiz.close();

  Serial.print("  Total: ");
  Serial.print(cuenta);
  Serial.print(" entradas, ");
  Serial.print(bytes);
  Serial.println(" bytes en archivos de la raiz");
  resultado("Raiz recorrida", true);
  return true;
}

// ============================================================================
// NIVEL 8 - SECTOR 0 EN CRUDO
// ============================================================================
// Lee el primer sector sin libreria. Si trae la firma 0x55AA hay tabla de
// particiones; si no, la tarjeta esta formateada sin particion o esta vacia,
// que es una de las razones por las que SdFat se niega a montar.
bool nivelSector0() {
  titulo("NIVEL 8 - SECTOR 0 EN CRUDO");

  bool v2 = false;
  uint8_t ocr[4] = {0, 0, 0, 0};
  if (!preparaBusCrudo(v2, ocr)) {
    resultado("Preparar la tarjeta para lectura cruda", false,
              "la tarjeta no llego al estado operativo");
    SPI.endTransaction();
    return false;
  }

  // El sector 0 esta en la direccion 0 tanto por bloque como por byte, asi que
  // el argumento de CMD17 es el mismo para SDHC y para SDSC.
  static uint8_t sector[512];
  csBajo();
  uint8_t r1 = comandoSd(17, 0, 0xFF);
  bool ok = (r1 == 0x00) && leeBloque(sector, 512);
  csAlto();
  SPI.endTransaction();

  if (!ok) {
    resultado("Leer el sector 0", false, "CMD17 no entrego datos");
    return false;
  }

  Serial.println("  Primeros 64 bytes:");
  for (uint16_t i = 0; i < 64; ++i) {
    if ((i % 16) == 0) {
      Serial.print("    ");
      if (i < 16) Serial.print('0');
      Serial.print(i, HEX);
      Serial.print("  ");
    }
    imprimeHex(sector[i]);
    Serial.print(' ');
    if ((i % 16) == 15) Serial.println();
  }

  bool firma = (sector[510] == 0x55 && sector[511] == 0xAA);
  Serial.print("  Firma en 0x1FE: 0x");
  imprimeHex(sector[510]);
  imprimeHex(sector[511]);
  Serial.println();
  resultado("Sector 0 con firma valida", firma,
            firma ? nullptr : "sin tabla de particiones ni sector de arranque");

  if (firma) {
    // Tipo de la primera particion: byte 4 de su entrada en la tabla.
    uint8_t tipo = sector[446 + 4];
    Serial.print("  Tipo de la particion 1: 0x");
    imprimeHex(tipo);
    Serial.print("  -> ");
    switch (tipo) {
      case 0x00: Serial.println("vacia"); break;
      case 0x01: Serial.println("FAT12"); break;
      case 0x04:
      case 0x06:
      case 0x0E: Serial.println("FAT16"); break;
      case 0x0B:
      case 0x0C: Serial.println("FAT32"); break;
      case 0x07: Serial.println("exFAT o NTFS"); break;
      default:   Serial.println("desconocido"); break;
    }
    if (tipo == 0x07) {
      Serial.println("  ESTA ES LA CAUSA si el nivel 3 fallo: SdFat no monta");
      Serial.println("  exFAT en esta configuracion. Reformatear en FAT32.");
    }
  }
  return ok;
}

// ============================================================================
// NIVEL 9 - CONSISTENCIA DE LECTURA
// ============================================================================
// Un sector 0 con basura admite dos explicaciones muy distintas: la tarjeta
// guarda de verdad esos bytes (no tiene formato), o el bus los esta corrompiendo
// al vuelo. Se distinguen leyendo el mismo sector tres veces: si las tres
// lecturas coinciden, la tarjeta entrega lo que tiene y el problema es de
// formato; si difieren, el bus o la tarjeta estan fallando en la lectura misma
// y formatear no arreglaria nada.
bool leeSector(uint32_t lba, bool porBloque, uint8_t *destino) {
  uint32_t direccion = porBloque ? lba : (lba * 512UL);
  csBajo();
  uint8_t r1 = comandoSd(17, direccion, 0xFF);
  bool ok = (r1 == 0x00) && leeBloque(destino, 512);
  csAlto();
  return ok;
}

uint16_t cuentaDiferencias(const uint8_t *a, const uint8_t *b) {
  uint16_t n = 0;
  for (uint16_t i = 0; i < 512; ++i) {
    if (a[i] != b[i]) n++;
  }
  return n;
}

bool nivelConsistencia() {
  titulo("NIVEL 9 - CONSISTENCIA DE LECTURA");

  bool v2 = false;
  uint8_t ocr[4] = {0, 0, 0, 0};
  if (!preparaBusCrudo(v2, ocr)) {
    resultado("Preparar la tarjeta", false, "no llego al estado operativo");
    SPI.endTransaction();
    return false;
  }
  bool porBloque = (ocr[0] & 0x40) != 0;

  static uint8_t a[512], b[512], c[512];
  bool okA = leeSector(0, porBloque, a);
  bool okB = leeSector(0, porBloque, b);
  bool okC = leeSector(0, porBloque, c);

  if (!okA || !okB || !okC) {
    resultado("Tres lecturas del sector 0", false,
              "alguna lectura no entrego datos");
    SPI.endTransaction();
    return false;
  }

  uint16_t dAB = cuentaDiferencias(a, b);
  uint16_t dAC = cuentaDiferencias(a, c);

  Serial.print("  Lectura 1 contra lectura 2: ");
  Serial.print(dAB);
  Serial.println(" bytes distintos de 512");
  Serial.print("  Lectura 1 contra lectura 3: ");
  Serial.print(dAC);
  Serial.println(" bytes distintos de 512");

  bool estable = (dAB == 0 && dAC == 0);
  resultado("El sector 0 se lee igual las tres veces", estable);

  // Otros sectores: si toda la tarjeta devuelve el mismo patron, no es un
  // contenido real sino lo que el controlador inventa cuando no tiene memoria
  // detras. Es la firma de una tarjeta apocrifa.
  static uint8_t s1[512], s2[512];
  bool ok1 = leeSector(1, porBloque, s1);
  bool ok2 = leeSector(2048, porBloque, s2);
  SPI.endTransaction();

  bool patronFijo = false;
  if (ok1 && ok2) {
    Serial.print("  Sector 1    (16 bytes): ");
    for (uint8_t i = 0; i < 16; ++i) {
      imprimeHex(s1[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.print("  Sector 2048 (16 bytes): ");
    for (uint8_t i = 0; i < 16; ++i) {
      imprimeHex(s2[i]);
      Serial.print(' ');
    }
    Serial.println();

    uint16_t d01 = cuentaDiferencias(a, s1);
    uint16_t d02 = cuentaDiferencias(a, s2);
    Serial.print("  Sector 1 contra sector 0: ");
    Serial.print(d01);
    Serial.println(" bytes distintos");
    Serial.print("  Sector 2048 contra sector 0: ");
    Serial.print(d02);
    Serial.println(" bytes distintos");
    patronFijo = (d01 == 0 && d02 == 0);
    if (patronFijo) {
      Serial.println("  ALERTA: sectores muy separados devuelven exactamente lo");
      Serial.println("  mismo. El controlador entrega un patron fijo en lugar");
      Serial.println("  de contenido: no hay memoria real detras.");
    }
  } else {
    Serial.println("  No se pudieron leer los sectores 1 y 2048.");
  }

  Serial.println();
  if (!estable) {
    Serial.println("  Las lecturas del mismo sector no coinciden. La tarjeta");
    Serial.println("  entrega datos distintos cada vez: formatearla no");
    Serial.println("  arreglaria nada. Es tarjeta danada o apocrifa.");
  } else if (patronFijo) {
    Serial.println("  Las lecturas son repetibles, pero todos los sectores");
    Serial.println("  devuelven el mismo patron. Eso no es contenido: es un");
    Serial.println("  controlador contestando sin memoria detras. Formatear no");
    Serial.println("  sirve; hay que cambiar la tarjeta.");
  } else {
    Serial.println("  Las lecturas son repetibles y los sectores difieren entre");
    Serial.println("  si: la tarjeta entrega de verdad lo que tiene guardado. Si");
    Serial.println("  ese contenido no es un FAT valido, el camino es formatear.");
  }
  return estable && !patronFijo;
}

// ============================================================================
// NIVEL W - ESCRITURA Y RELECTURA DE UN SECTOR EN CRUDO
// ============================================================================
// DESTRUCTIVA: pisa el sector 1 de la tarjeta. Se corre solo cuando ya se
// acepto reformatear.
//
// Es la prueba que decide entre las dos explicaciones que quedan cuando la
// tarjeta lee bien en la computadora pero aqui devuelve basura:
//
//   El patron vuelve intacto  -> la tarjeta guarda y el nodo lee bien. Lo que
//                                se leyo antes era estado trabado, no un
//                                defecto permanente.
//   El patron vuelve alterado -> la ruta de datos del nodo corrompe los
//                                bloques: alimentacion del modulo, nivel
//                                logico o cables. La tarjeta es inocente.
//
// El patron lleva una cuenta ascendente mas una firma, para que un corrimiento
// de un byte se note de inmediato en el volcado.
bool nivelEscrituraCruda() {
  titulo("NIVEL W - ESCRITURA CRUDA EN EL SECTOR 1 (DESTRUCTIVA)");

  bool v2 = false;
  uint8_t ocr[4] = {0, 0, 0, 0};
  if (!preparaBusCrudo(v2, ocr)) {
    resultado("Preparar la tarjeta", false, "no llego al estado operativo");
    SPI.endTransaction();
    return false;
  }
  bool porBloque = (ocr[0] & 0x40) != 0;
  uint32_t direccion = porBloque ? 1UL : 512UL;

  static uint8_t patron[512];
  for (uint16_t i = 0; i < 512; ++i) patron[i] = (uint8_t)(i & 0xFF);
  patron[0] = 0x5D; patron[1] = 0x11; patron[2] = 0xA5; patron[3] = 0x3C;
  patron[508] = 0x3C; patron[509] = 0xA5; patron[510] = 0x11; patron[511] = 0x5D;

  // --- CMD24: escribir un solo bloque ---
  csBajo();
  uint8_t r1 = comandoSd(24, direccion, 0xFF);
  if (r1 != 0x00) {
    csAlto();
    SPI.endTransaction();
    Serial.print("  CMD24 -> R1 = 0x");
    imprimeHex(r1);
    Serial.println();
    resultado("CMD24: la tarjeta acepto el comando de escritura", false,
              r1 == 0xFF ? "sin respuesta" : "rechazo el comando");
    return false;
  }

  SPI.transfer(0xFF);
  SPI.transfer(0xFE); // token de inicio de bloque
  for (uint16_t i = 0; i < 512; ++i) SPI.transfer(patron[i]);
  SPI.transfer(0xFF); // CRC, la tarjeta lo ignora en modo SPI
  SPI.transfer(0xFF);

  uint8_t respuesta = 0xFF;
  for (uint8_t i = 0; i < 10 && (respuesta & 0x11) != 0x01; ++i) {
    respuesta = SPI.transfer(0xFF);
  }
  bool aceptado = ((respuesta & 0x1F) == 0x05);
  Serial.print("  Respuesta de datos = 0x");
  imprimeHex(respuesta);
  Serial.print("  -> ");
  switch (respuesta & 0x1F) {
    case 0x05: Serial.println("aceptado"); break;
    case 0x0B: Serial.println("rechazado por CRC"); break;
    case 0x0D: Serial.println("rechazado por error de escritura"); break;
    default:   Serial.println("desconocido"); break;
  }

  // La tarjeta mantiene MISO en cero mientras graba.
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    if (SPI.transfer(0xFF) != 0x00) break;
  }
  uint32_t esperaMs = millis() - t0;
  csAlto();

  Serial.print("  La tarjeta estuvo ocupada ");
  Serial.print(esperaMs);
  Serial.println(" ms grabando");
  resultado("CMD24: bloque escrito", aceptado);
  if (!aceptado) {
    SPI.endTransaction();
    return false;
  }

  // --- Relectura ---
  static uint8_t vuelta[512];
  bool leido = leeSector(1, porBloque, vuelta);
  SPI.endTransaction();

  if (!leido) {
    resultado("Releer el sector 1", false, "CMD17 no entrego datos");
    return false;
  }

  uint16_t diferencias = cuentaDiferencias(patron, vuelta);
  Serial.print("  Primeros 16 bytes escritos: ");
  for (uint8_t i = 0; i < 16; ++i) { imprimeHex(patron[i]); Serial.print(' '); }
  Serial.println();
  Serial.print("  Primeros 16 bytes leidos:   ");
  for (uint8_t i = 0; i < 16; ++i) { imprimeHex(vuelta[i]); Serial.print(' '); }
  Serial.println();
  Serial.print("  Bytes distintos: ");
  Serial.print(diferencias);
  Serial.println(" de 512");

  bool ok = (diferencias == 0);
  resultado("El patron volvio intacto", ok);

  Serial.println();
  if (ok) {
    Serial.println("  La tarjeta guarda y devuelve lo que se le escribe, y la");
    Serial.println("  ruta de datos del nodo es fiel. Lo que fallaba antes era");
    Serial.println("  estado trabado por no cortar la alimentacion, o el");
    Serial.println("  formato del volumen.");
  } else {
    Serial.println("  Lo escrito no vuelve igual. Con una tarjeta que la");
    Serial.println("  computadora lee bien, el defecto esta en el nodo:");
    Serial.println("    - alimentacion del modulo microSD (3V3 flojo o el");
    Serial.println("      modulo esperando 5 V),");
    Serial.println("    - adaptador de nivel logico del modulo,");
    Serial.println("    - cables largos o sin tierra de retorno.");
  }
  return ok;
}

// ============================================================================
// PRUEBA DE INTERMITENCIA
// ============================================================================
// La falla del 29 y 30 de agosto de 2026 fue intermitente y dependiente de la
// fuente. Montar veinte veces seguidas y contar los exitos distingue un defecto
// duro de uno marginal.
void pruebaIntermitencia(uint8_t repeticiones) {
  titulo("REPETICION DE MONTAJE");

  uint8_t exitos = 0;
  uint32_t peorMs = 0;
  for (uint8_t i = 1; i <= repeticiones; ++i) {
    uint32_t t0 = millis();
    bool ok = intentaMontar(spiKhz, false);
    uint32_t dt = millis() - t0;
    if (dt > peorMs) peorMs = dt;
    if (ok) exitos++;
    Serial.print("  Intento ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(ok ? "OK" : "FALLA");
    Serial.print("  (");
    Serial.print(dt);
    Serial.println(" ms)");
    delay(100);
  }

  Serial.println();
  Serial.print("  Exitos: ");
  Serial.print(exitos);
  Serial.print(" de ");
  Serial.println(repeticiones);
  Serial.print("  Montaje mas lento: ");
  Serial.print(peorMs);
  Serial.println(" ms");

  if (exitos == 0) {
    Serial.println("  Falla dura y repetible.");
  } else if (exitos < repeticiones) {
    Serial.println("  Falla intermitente: contacto del zocalo, alimentacion");
    Serial.println("  marginal o cables largos. Conviene repetir alimentando");
    Serial.println("  por bateria, que es cuando aparecio el problema.");
  } else {
    Serial.println("  Montaje estable en todos los intentos.");
  }
}

// ============================================================================
// PRUEBA COMPLETA
// ============================================================================
void pruebaCompleta() {
  titulo("PRUEBA COMPLETA DE microSD");
  Serial.println("  Se detiene en el primer nivel que descarte a los que");
  Serial.println("  siguen.");

  bool pines = nivelPines();
  bool cruda = nivelSondaCruda();

  if (estadoTrabado) {
    titulo("VEREDICTO");
    Serial.println("  Sin veredicto: la tarjeta quedo trabada de la corrida");
    Serial.println("  anterior. Desconecta el nodo del USB, vuelve a conectarlo");
    Serial.println("  y corre la prueba otra vez. Los niveles 3 en adelante no");
    Serial.println("  significan nada mientras la tarjeta este en este estado.");
    return;
  }

  if (!cruda) {
    titulo("VEREDICTO");
    if (!pines) {
      Serial.println("  Defecto fisico en el bus SPI. Revisar el cableado y la");
      Serial.println("  alimentacion del modulo antes de volver a probar.");
    } else {
      Serial.println("  Los pines estan sanos pero la tarjeta no contesta el");
      Serial.println("  protocolo SD. Revisar, en este orden: tarjeta insertada");
      Serial.println("  y asentada, 3V3 en el modulo, continuidad de");
      Serial.println("  MISO/MOSI/SCK/CS, y por ultimo una tarjeta de repuesto.");
    }
    Serial.println("  Los niveles 3 a 7 no se corren: dependen de una tarjeta");
    Serial.println("  que responda.");
    return;
  }

  bool monta = nivelMontaje();
  nivelSector0();

  if (!monta) {
    // Que no monte admite dos causas muy distintas, y solo el nivel 9 las
    // separa: contenido invalido (se formatea) contra tarjeta que no guarda
    // nada (se cambia).
    bool contenidoReal = nivelConsistencia();
    titulo("VEREDICTO");
    if (contenidoReal) {
      Serial.println("  La tarjeta responde y guarda contenido real, pero no es");
      Serial.println("  un FAT que la libreria pueda montar. Formatear en FAT32");
      Serial.println("  con una sola particion primaria.");
    } else {
      Serial.println("  La tarjeta contesta el protocolo SD pero no entrega");
      Serial.println("  contenido creible: no es un problema de formato ni de");
      Serial.println("  cableado. Cambiar la tarjeta y repetir la prueba.");
    }
    return;
  }

  nivelInfo();
  bool escribe = nivelEscritura();
  nivelVelocidad();
  nivelListado();

  titulo("VEREDICTO");
  if (escribe) {
    Serial.println("  La microSD esta sana de punta a punta: responde, monta,");
    Serial.println("  escribe y relee. Si el firmware de operacion sigue");
    Serial.println("  fallando con esta misma tarjeta, el defecto esta en como");
    Serial.println("  R11 la inicializa, no en la tarjeta.");
  } else {
    Serial.println("  Monta pero no conserva lo escrito. Tarjeta gastada o");
    Serial.println("  protegida contra escritura. Probar con otra.");
  }
}

// ============================================================================
// MENU
// ============================================================================
void menu() {
  Serial.println();
  Serial.println("------------------------------------------------------------");
  Serial.println("  SD_TEST - banco de pruebas de la microSD del nodo R11");
  Serial.println("------------------------------------------------------------");
  Serial.println("  a  prueba completa");
  Serial.println("  1  pines (estado electrico de MISO)");
  Serial.println("  2  sonda SPI cruda (CMD0/CMD8/ACMD41/CMD58/CSD/CID)");
  Serial.println("  3  montaje con la libreria a varias velocidades");
  Serial.println("  4  informacion del sistema de archivos");
  Serial.println("  5  escritura y lectura de un archivo");
  Serial.println("  6  velocidad de escritura sostenida");
  Serial.println("  7  listado de la raiz");
  Serial.println("  8  sector 0 en crudo (particion y formato)");
  Serial.println("  9  consistencia de lectura (tarjeta real o apocrifa)");
  Serial.println("  w  escribir y releer el sector 1 (DESTRUCTIVA)");
  Serial.println("  r  repetir el montaje 20 veces (intermitencia)");
  Serial.println("  p  cambiar el pin CS         ejemplo: p13");
  Serial.println("  f  cambiar la velocidad kHz  ejemplo: f1000");
  Serial.println("  h  este menu");
  Serial.print("  Ahora: CS=GP");
  Serial.print(csPin);
  Serial.print("  bus=");
  Serial.print(spiKhz);
  Serial.println(" kHz");
  Serial.println("------------------------------------------------------------");
}

void atiende(const String &entrada) {
  if (entrada.length() == 0) return;
  char c = entrada.charAt(0);

  switch (c) {
    case 'a': pruebaCompleta(); break;
    case '1': nivelPines(); break;
    case '2': nivelSondaCruda(); break;
    case '3': nivelMontaje(); break;
    case '4': nivelInfo(); break;
    case '5': nivelEscritura(); break;
    case '6': nivelVelocidad(); break;
    case '7': nivelListado(); break;
    case '8': nivelSector0(); break;
    case '9': nivelConsistencia(); break;
    case 'w': nivelEscrituraCruda(); break;
    case 'r': pruebaIntermitencia(20); break;
    case 'p': {
      int v = entrada.substring(1).toInt();
      if (v >= 0 && v <= 28) {
        csPin = (uint8_t)v;
        montada = false;
        Serial.print("  CS ahora en GP");
        Serial.println(csPin);
      } else {
        Serial.println("  Pin fuera de rango (0 a 28).");
      }
      break;
    }
    case 'f': {
      long v = entrada.substring(1).toInt();
      if (v >= 100 && v <= 25000) {
        spiKhz = (uint32_t)v;
        Serial.print("  Velocidad del bus ahora ");
        Serial.print(spiKhz);
        Serial.println(" kHz");
      } else {
        Serial.println("  Velocidad fuera de rango (100 a 25000 kHz).");
      }
      break;
    }
    case 'h': menu(); break;
    default:
      Serial.print("  Comando desconocido: ");
      Serial.println(c);
      break;
  }
  Serial.println();
  Serial.print("> ");
}

// ============================================================================
// ARRANQUE Y LAZO
// ============================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  // Se espera a la consola, pero sin quedarse colgado si el nodo arranca solo.
  uint32_t limite = millis() + 5000;
  while (!Serial && millis() < limite) {
    digitalWrite(LED_PIN, (millis() / 100) % 2);
    delay(10);
  }
  digitalWrite(LED_PIN, LOW);
  delay(300);

  Serial.println();
  Serial.println("SD_TEST listo. Solo prueba la microSD; no toca sensores, LTE");
  Serial.println("ni navegacion.");
  menu();

  pruebaCompleta();
  Serial.println();
  Serial.print("> ");
}

void loop() {
  static String entrada;

  // Latido: si el LED deja de parpadear, el programa se colgo en una prueba.
  digitalWrite(LED_PIN, (millis() / 500) % 2);

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String copia = entrada;
      copia.trim();
      entrada = "";
      atiende(copia);
    } else if (entrada.length() < 16) {
      entrada += c;
    }
  }
}
