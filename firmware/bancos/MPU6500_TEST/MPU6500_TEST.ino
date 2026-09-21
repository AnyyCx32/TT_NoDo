// ============================================================================
//  MPU6500_TEST - prueba aislada del acelerometro fuera del nodo
//  Raspberry Pi Pico (RP2040) + MPU6500 por I2C
//
//  Responde una sola pregunta: el MPU6500 esta vivo o no.
//  En el nodo R11 no aparecia en 0x68, y hay tres explicaciones posibles:
//     a) el chip esta muerto o mal alimentado
//     b) AD0 esta en alto y el chip vive en 0x69, donde ya esta la ICM-20948
//        (choque de direcciones: el bus responde, pero contesta la ICM)
//     c) el cableado del nodo esta abierto
//  Este programa distingue las tres.
//
//  NO hace falta saber en que pines quedo conectado: recorre todas las
//  parejas SDA/SCL validas del RP2040 hasta encontrarlo.
//
//  USO: Monitor Serie a 115200. Al arrancar barre el bus solo.
//       Despues, escribir la letra y ENTER (h para el menu).
// ============================================================================

#include <Wire.h>
#include <math.h>

static const char TEST_VERSION[] = "MPU6500_TEST v1.0";

// Registros del MPU6500 (los mismos que usa R11).
static const uint8_t RA_WHO_AM_I     = 0x75;
static const uint8_t RA_PWR_MGMT_1   = 0x6B;
static const uint8_t RA_PWR_MGMT_2   = 0x6C;
static const uint8_t RA_SMPLRT_DIV   = 0x19;
static const uint8_t RA_CONFIG       = 0x1A;
static const uint8_t RA_GYRO_CONFIG  = 0x1B;
static const uint8_t RA_ACCEL_CONFIG = 0x1C;
static const uint8_t RA_ACCEL_XOUT_H = 0x3B;
static const uint8_t RA_TEMP_OUT_H   = 0x41;
static const uint8_t RA_GYRO_XOUT_H  = 0x43;

static const float LSB_PER_G   = 8192.0f;   // +/-4 g, igual que R11
static const float LSB_PER_DPS = 131.0f;    // +/-250 dps

// Parejas SDA/SCL validas en el RP2040. El RP2040 exige SDA en pines
// congruentes a 0 modulo 4 (I2C0) o a 2 (I2C1), y SCL en el pin siguiente.
struct I2cPins {
  TwoWire *bus;
  uint8_t sda;
  uint8_t scl;
  const char *busName;
};

I2cPins candidates[] = {
  { &Wire,   0,  1, "I2C0" },
  { &Wire,   4,  5, "I2C0" },   // el cableado del nodo R11
  { &Wire,   8,  9, "I2C0" },
  { &Wire,  12, 13, "I2C0" },
  { &Wire,  16, 17, "I2C0" },
  { &Wire,  20, 21, "I2C0" },
  { &Wire1,  2,  3, "I2C1" },
  { &Wire1,  6,  7, "I2C1" },
  { &Wire1, 10, 11, "I2C1" },
  { &Wire1, 14, 15, "I2C1" },
  { &Wire1, 18, 19, "I2C1" },
  { &Wire1, 26, 27, "I2C1" },
};
static const uint8_t CANDIDATE_COUNT = sizeof(candidates) / sizeof(candidates[0]);

// Donde quedo localizado el sensor tras el barrido.
TwoWire *foundBus = nullptr;
uint8_t foundSda = 0, foundScl = 0, foundAddr = 0, foundWho = 0;
const char *foundBusName = "";
bool sensorConfigured = false;

// ============================================================================
// ACCESO I2C
// ============================================================================
bool i2cWrite8(TwoWire *bus, uint8_t addr, uint8_t reg, uint8_t value) {
  bus->beginTransmission(addr);
  bus->write(reg);
  bus->write(value);
  return bus->endTransmission() == 0;
}

bool i2cReadN(TwoWire *bus, uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t count) {
  bus->beginTransmission(addr);
  bus->write(reg);
  if (bus->endTransmission(false) != 0) return false;
  uint8_t received = bus->requestFrom((int)addr, (int)count, (int)true);
  if (received != count) return false;
  for (uint8_t i = 0; i < count; ++i) buffer[i] = bus->read();
  return true;
}

// Nombre del chip segun WHO_AM_I. Sirve para saber si lo que contesta es
// realmente el MPU6500 o algo distinto ocupando esa direccion.
const char *whoAmIName(uint8_t who) {
  switch (who) {
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    case 0x68: return "MPU6050";
    case 0x75: return "MPU6515";
    case 0xEA: return "ICM-20948";
    case 0x12: return "ICM-20602";
    case 0x11: return "ICM-20601";
    case 0x00: return "sin respuesta util";
    case 0xFF: return "bus flotante (sin pull-up o sin alimentacion)";
  }
  return "desconocido";
}

void banner(const char *title) {
  Serial.println();
  Serial.println("============================================================");
  Serial.print("  "); Serial.println(title);
  Serial.println("============================================================");
}

// ============================================================================
// BARRIDO DE TODAS LAS PAREJAS DE PINES
// ============================================================================
void sweepAllPins() {
  banner("Barrido de todas las parejas SDA/SCL del RP2040");
  Serial.println("  Se prueban las 12 combinaciones validas. En cada una se");
  Serial.println("  interroga 0x68 (AD0 a masa) y 0x69 (AD0 en alto).");
  Serial.println();

  foundBus = nullptr;
  uint8_t hits = 0;

  for (uint8_t i = 0; i < CANDIDATE_COUNT; ++i) {
    I2cPins &c = candidates[i];

    c.bus->end();
    c.bus->setSDA(c.sda);
    c.bus->setSCL(c.scl);
    c.bus->begin();
    c.bus->setClock(100000);   // 100 kHz: mas tolerante a cables largos
    c.bus->setTimeout(50);
    delay(20);

    // Si SDA esta clavado en bajo, el maestro lee la linea abajo durante el bit
    // de confirmacion y cree que contesta cada direccion del mapa. Eso es un
    // corto, no 112 sensores: conviene detectarlo antes de imprimir la lista.
    uint8_t respuestas = 0;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
      c.bus->beginTransmission(addr);
      if (c.bus->endTransmission() == 0) respuestas++;
    }
    if (respuestas > 8) {
      Serial.print("  ");
      Serial.print(c.busName);
      Serial.print("  SDA=GP"); Serial.print(c.sda);
      Serial.print(" SCL=GP"); Serial.print(c.scl);
      Serial.print("  ->  BUS AVERIADO: contestan ");
      Serial.print(respuestas);
      Serial.println(" direcciones de 112");
      Serial.println("      Sintoma de SDA en corto a masa. Ejecutar  p  para ubicarlo.");
      c.bus->end();
      continue;
    }

    // Barrido completo, no solo 0x68/0x69: si el chip quedo en otra direccion
    // o hay algo mas colgado del bus, conviene verlo.
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
      c.bus->beginTransmission(addr);
      if (c.bus->endTransmission() != 0) continue;

      uint8_t who = 0;
      bool readOk = i2cReadN(c.bus, addr, RA_WHO_AM_I, &who, 1);

      Serial.print("  ");
      Serial.print(c.busName);
      Serial.print("  SDA=GP"); Serial.print(c.sda);
      Serial.print(" SCL=GP"); Serial.print(c.scl);
      Serial.print("  ->  0x");
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      Serial.print("  WHO_AM_I=");
      if (readOk) {
        Serial.print("0x");
        if (who < 16) Serial.print('0');
        Serial.print(who, HEX);
        Serial.print("  ");
        Serial.print(whoAmIName(who));
      } else {
        Serial.print("(no se pudo leer)");
      }
      Serial.println();
      hits++;

      // Se guarda el primero que se identifique como familia MPU.
      if (readOk && foundBus == nullptr &&
          (who == 0x70 || who == 0x71 || who == 0x73 || who == 0x68 || who == 0x75)) {
        foundBus = c.bus;
        foundSda = c.sda;
        foundScl = c.scl;
        foundAddr = addr;
        foundWho = who;
        foundBusName = c.busName;
      }
    }

    c.bus->end();
  }

  Serial.println();
  if (foundBus == nullptr) {
    if (hits == 0) {
      Serial.println("  RESULTADO: ningun dispositivo respondio en ninguna pareja de pines.");
      Serial.println();
      Serial.println("  Eso deja tres causas, en orden de probabilidad:");
      Serial.println("   1. Alimentacion: VCC del modulo a 3V3 (pin 36) y GND comun.");
      Serial.println("      Muchos modulos GY-91/GY-6500 traen regulador y aceptan 5V,");
      Serial.println("      pero el nivel logico debe quedar en 3.3 V.");
      Serial.println("   2. Cableado: SDA y SCL cruzados, o en pines que no forman");
      Serial.println("      pareja valida. Este barrido ya descarta la pareja equivocada.");
      Serial.println("   3. El chip esta danado.");
      Serial.println();
      Serial.println("  Comprobacion rapida: con el modulo alimentado, medir 3.3 V");
      Serial.println("  entre VCC y GND del propio modulo, no en la Pico.");
    } else {
      Serial.println("  RESULTADO: hay dispositivos en el bus, pero ninguno es un MPU.");
      Serial.println("  Revisar la lista de arriba: el WHO_AM_I dice que hay en realidad.");
    }
    return;
  }

  Serial.println("  RESULTADO: sensor localizado.");
  Serial.print("    bus       "); Serial.println(foundBusName);
  Serial.print("    SDA       GP"); Serial.println(foundSda);
  Serial.print("    SCL       GP"); Serial.println(foundScl);
  Serial.print("    direccion 0x"); Serial.println(foundAddr, HEX);
  Serial.print("    chip      "); Serial.print(whoAmIName(foundWho));
  Serial.print(" (WHO_AM_I=0x"); Serial.print(foundWho, HEX); Serial.println(")");
  Serial.println();

  if (foundAddr == 0x69) {
    Serial.println("  ATENCION - ESTE ES EL DATO IMPORTANTE:");
    Serial.println("  El sensor esta en 0x69, no en 0x68. En el nodo R11 esa direccion");
    Serial.println("  ya la ocupa la ICM-20948, y R11 busca el MPU6500 exclusivamente");
    Serial.println("  en 0x68, asi que nunca lo encuentra. El chip no esta muerto:");
    Serial.println("  hay un choque de direcciones.");
    Serial.println("  Solucion: llevar AD0/SDO del modulo a GND para moverlo a 0x68.");
  } else if (foundAddr == 0x68) {
    Serial.println("  El sensor responde en 0x68, que es justo donde R11 lo busca.");
    Serial.println("  Si aqui funciona y en el nodo no aparece, el problema esta en");
    Serial.println("  el cableado o la soldadura del nodo, no en el chip.");
  }

  // Se deja el bus abierto en la pareja encontrada para las pruebas siguientes.
  foundBus->end();
  foundBus->setSDA(foundSda);
  foundBus->setSCL(foundScl);
  foundBus->begin();
  foundBus->setClock(400000);   // la velocidad real que usa el nodo
  foundBus->setTimeout(50);
  sensorConfigured = false;
}

// ============================================================================
// CONFIGURACION IGUAL A LA DEL NODO
// ============================================================================
bool configureSensor() {
  if (foundBus == nullptr) return false;

  if (!i2cWrite8(foundBus, foundAddr, RA_PWR_MGMT_1, 0x80)) return false;  // reset
  delay(100);
  if (!i2cWrite8(foundBus, foundAddr, RA_PWR_MGMT_1, 0x00)) return false;  // despierta
  delay(10);
  if (!i2cWrite8(foundBus, foundAddr, RA_PWR_MGMT_2, 0x00)) return false;  // todos los ejes
  if (!i2cWrite8(foundBus, foundAddr, RA_SMPLRT_DIV, 9)) return false;     // ~100 Hz
  if (!i2cWrite8(foundBus, foundAddr, RA_CONFIG, 0x03)) return false;      // DLPF
  if (!i2cWrite8(foundBus, foundAddr, RA_ACCEL_CONFIG, 0x08)) return false;// +/-4 g
  if (!i2cWrite8(foundBus, foundAddr, RA_GYRO_CONFIG, 0x00)) return false; // +/-250 dps
  delay(20);

  sensorConfigured = true;
  return true;
}

bool readAccelG(float &ax, float &ay, float &az) {
  uint8_t b[6];
  if (!i2cReadN(foundBus, foundAddr, RA_ACCEL_XOUT_H, b, 6)) return false;
  ax = (float)(int16_t)((b[0] << 8) | b[1]) / LSB_PER_G;
  ay = (float)(int16_t)((b[2] << 8) | b[3]) / LSB_PER_G;
  az = (float)(int16_t)((b[4] << 8) | b[5]) / LSB_PER_G;
  return true;
}

bool readGyroDps(float &gx, float &gy, float &gz) {
  uint8_t b[6];
  if (!i2cReadN(foundBus, foundAddr, RA_GYRO_XOUT_H, b, 6)) return false;
  gx = (float)(int16_t)((b[0] << 8) | b[1]) / LSB_PER_DPS;
  gy = (float)(int16_t)((b[2] << 8) | b[3]) / LSB_PER_DPS;
  gz = (float)(int16_t)((b[4] << 8) | b[5]) / LSB_PER_DPS;
  return true;
}

bool readTempC(float &tempC) {
  uint8_t b[2];
  if (!i2cReadN(foundBus, foundAddr, RA_TEMP_OUT_H, b, 2)) return false;
  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  tempC = (raw / 333.87f) + 21.0f;
  return true;
}

// ============================================================================
// PRUEBA COMPLETA EN REPOSO
// ============================================================================
void runFullTest() {
  banner("Prueba completa del sensor");

  if (foundBus == nullptr) {
    Serial.println("  Todavia no se ha localizado el sensor. Ejecutar  s  primero.");
    return;
  }

  if (!configureSensor()) {
    Serial.println("  FALLA: el sensor respondio al barrido pero rechaza la configuracion.");
    Serial.println("  Suele indicar alimentacion inestable o un cable de mas de 20 cm.");
    return;
  }
  Serial.println("  Configurado: +/-4 g, +/-250 dps, DLPF, ~100 Hz (igual que R11).");

  // Verificacion de que la configuracion realmente quedo escrita.
  uint8_t back = 0;
  if (i2cReadN(foundBus, foundAddr, RA_ACCEL_CONFIG, &back, 1)) {
    Serial.print("  ACCEL_CONFIG releido = 0x"); Serial.print(back, HEX);
    Serial.println(back == 0x08 ? "  (correcto)" : "  (NO coincide con lo escrito)");
  }

  float tempC = 0;
  if (readTempC(tempC)) {
    Serial.print("  Temperatura interna: "); Serial.print(tempC, 1); Serial.println(" C");
    if (tempC < 0.0f || tempC > 70.0f)
      Serial.println("  Temperatura fuera de lo razonable: el dado puede estar danado.");
  }

  Serial.println("  Tomando 200 muestras en reposo. NO MOVER...");

  double sa[3] = {0, 0, 0}, sa2[3] = {0, 0, 0}, sg[3] = {0, 0, 0};
  int n = 0, failures = 0;
  uint32_t start = millis();

  while (n < 200 && (millis() - start) < 6000) {
    float ax, ay, az, gx, gy, gz;
    if (readAccelG(ax, ay, az) && readGyroDps(gx, gy, gz)) {
      float a[3] = {ax, ay, az};
      float g[3] = {gx, gy, gz};
      for (int i = 0; i < 3; ++i) {
        sa[i] += a[i];
        sa2[i] += (double)a[i] * a[i];
        sg[i] += g[i];
      }
      n++;
    } else failures++;
    delay(5);
  }

  Serial.print("  muestras validas="); Serial.print(n);
  Serial.print("  lecturas fallidas="); Serial.println(failures);

  if (n < 50) {
    Serial.println("  FALLA: el sensor no entrega datos de forma sostenida.");
    return;
  }

  float ma[3], sd[3], mg[3];
  for (int i = 0; i < 3; ++i) {
    ma[i] = sa[i] / n;
    float var = (sa2[i] / n) - (double)ma[i] * ma[i];
    sd[i] = var > 0 ? sqrtf(var) : 0.0f;
    mg[i] = sg[i] / n;
  }
  float magG = sqrtf(ma[0] * ma[0] + ma[1] * ma[1] + ma[2] * ma[2]);
  float noise = (sd[0] + sd[1] + sd[2]) / 3.0f;
  float gyroBias = sqrtf(mg[0] * mg[0] + mg[1] * mg[1] + mg[2] * mg[2]);

  Serial.print("  acel media [g]  X="); Serial.print(ma[0], 4);
  Serial.print(" Y="); Serial.print(ma[1], 4);
  Serial.print(" Z="); Serial.println(ma[2], 4);
  Serial.print("  |a| = "); Serial.print(magG, 4);
  Serial.println(" g   (en reposo debe dar entre 0.95 y 1.05)");
  Serial.print("  ruido acel = "); Serial.print(noise, 5);
  Serial.println(" g   (un valor exactamente 0 significa registro congelado)");
  Serial.print("  giro medio [dps] X="); Serial.print(mg[0], 2);
  Serial.print(" Y="); Serial.print(mg[1], 2);
  Serial.print(" Z="); Serial.print(mg[2], 2);
  Serial.print("   |bias|="); Serial.println(gyroBias, 2);

  int upAxis = 0;
  for (int i = 1; i < 3; ++i) if (fabsf(ma[i]) > fabsf(ma[upAxis])) upAxis = i;
  Serial.print("  gravedad dominante en el eje ");
  Serial.print((char)('X' + upAxis));
  Serial.println(ma[upAxis] > 0 ? " positivo" : " negativo");

  Serial.println();
  if (magG < 0.90f || magG > 1.10f) {
    Serial.println("  VEREDICTO: FALLA. El modulo de la gravedad no es 1 g.");
    Serial.println("  El chip responde pero sus lecturas no son fisicamente validas.");
  } else if (noise < 0.0002f) {
    Serial.println("  VEREDICTO: FALLA. Las lecturas no cambian nunca.");
    Serial.println("  Devuelve un valor congelado, no esta midiendo.");
  } else if (failures > n / 10) {
    Serial.println("  VEREDICTO: INESTABLE. Demasiadas lecturas perdidas.");
    Serial.println("  Revisar cables cortos y pull-ups del bus.");
  } else if (gyroBias > 5.0f) {
    Serial.println("  VEREDICTO: ACEPTABLE con reserva. El giroscopo trae sesgo alto,");
    Serial.println("  corregible por calibracion, pero conviene anotarlo.");
  } else {
    Serial.println("  VEREDICTO: EL SENSOR FUNCIONA.");
    Serial.println("  Gravedad correcta, ruido presente y bus estable.");
    Serial.println("  Si en el nodo no aparece, el problema esta en el nodo.");
  }
}

// ============================================================================
// FLUJO EN VIVO
// ============================================================================
void streamLive(uint32_t seconds) {
  banner("Lectura en vivo: mover el modulo para ver responder los ejes");

  if (foundBus == nullptr) {
    Serial.println("  Todavia no se ha localizado el sensor. Ejecutar  s  primero.");
    return;
  }
  if (!sensorConfigured && !configureSensor()) {
    Serial.println("  No se pudo configurar el sensor.");
    return;
  }

  Serial.println("  Inclinar despacio en cada eje: el que apunte hacia abajo");
  Serial.println("  debe acercarse a -1.00 g, y hacia arriba a +1.00 g.");
  Serial.println("     ax     ay     az   [g]  |    gx     gy     gz  [dps]");

  uint32_t start = millis();
  uint32_t lastPrint = 0;
  uint32_t samples = 0;
  float peakAccel = 0, peakGyro = 0;

  while ((millis() - start) < seconds * 1000UL) {
    float ax, ay, az, gx, gy, gz;
    if (readAccelG(ax, ay, az) && readGyroDps(gx, gy, gz)) {
      samples++;
      float am = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1.0f);
      float gm = sqrtf(gx * gx + gy * gy + gz * gz);
      if (am > peakAccel) peakAccel = am;
      if (gm > peakGyro) peakGyro = gm;

      if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        Serial.print("  ");
        Serial.print(ax, 2); Serial.print("  ");
        Serial.print(ay, 2); Serial.print("  ");
        Serial.print(az, 2); Serial.print("   |  ");
        Serial.print(gx, 1); Serial.print("  ");
        Serial.print(gy, 1); Serial.print("  ");
        Serial.println(gz, 1);
      }
    }
    delay(5);
  }

  Serial.print("  ritmo ~"); Serial.print(samples / (float)seconds, 1);
  Serial.println(" Hz  (el nodo lo muestrea a 50 Hz)");
  Serial.print("  pico de aceleracion sobre 1 g: "); Serial.print(peakAccel, 3); Serial.println(" g");
  Serial.print("  pico de giro: "); Serial.print(peakGyro, 1); Serial.println(" dps");

  if (peakAccel < 0.05f && peakGyro < 5.0f)
    Serial.println("  No se detecto movimiento: repetir moviendo el modulo con la mano.");
  else
    Serial.println("  El sensor responde al movimiento en ambos sensores.");
}

// ============================================================================
// VOLCADO DE REGISTROS
// ============================================================================
void dumpRegisters() {
  banner("Volcado de los registros de configuracion");
  if (foundBus == nullptr) {
    Serial.println("  Todavia no se ha localizado el sensor. Ejecutar  s  primero.");
    return;
  }

  struct { uint8_t reg; const char *name; } regs[] = {
    { RA_SMPLRT_DIV,   "SMPLRT_DIV   (divisor de muestreo)" },
    { RA_CONFIG,       "CONFIG       (DLPF)" },
    { RA_GYRO_CONFIG,  "GYRO_CONFIG  (escala giro)" },
    { RA_ACCEL_CONFIG, "ACCEL_CONFIG (escala acel)" },
    { 0x1D,            "ACCEL_CFG2   (DLPF acel)" },
    { RA_PWR_MGMT_1,   "PWR_MGMT_1   (reloj y reposo)" },
    { RA_PWR_MGMT_2,   "PWR_MGMT_2   (ejes habilitados)" },
    { RA_WHO_AM_I,     "WHO_AM_I     (identidad)" },
  };

  for (uint8_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
    uint8_t value = 0;
    bool ok = i2cReadN(foundBus, foundAddr, regs[i].reg, &value, 1);
    Serial.print("  0x");
    if (regs[i].reg < 16) Serial.print('0');
    Serial.print(regs[i].reg, HEX);
    Serial.print("  ");
    Serial.print(regs[i].name);
    Serial.print("  = ");
    if (ok) {
      Serial.print("0x");
      if (value < 16) Serial.print('0');
      Serial.println(value, HEX);
    } else Serial.println("(lectura fallo)");
  }
}

// ============================================================================
// PRUEBAS ELECTRICAS DEL CABLEADO
// ============================================================================
// Pines de uso interno en la Pico, no llevan a la tira de conexion.
bool pinIsInternal(uint8_t gp) {
  return gp == 23 || gp == 24 || gp == 25 || gp == 29;
}

// Los modulos MPU6500 llevan pull-ups de SDA y SCL a VCC, de 2.2k a 10k.
// El pull-down interno del RP2040 ronda los 50k a 80k, mucho mas debil: si hay
// un pull-up externo, el pin se lee en alto aun con el pull-down puesto.
// Esta prueba distingue "modulo ausente" de "modulo presente pero mudo",
// porque los pull-ups aparecen aunque el chip este danado.
void detectPullups() {
  banner("Deteccion de pull-ups externos en todos los pines");
  Serial.println("  Con pull-down interno puesto, un pin que aun asi se lee alto");
  Serial.println("  tiene un pull-up externo: hay algo conectado Y alimentado.");
  Serial.println();

  uint8_t conPullup = 0, aTierra = 0, alAire = 0;
  uint8_t listaPullup[32], listaTierra[32];

  for (uint8_t gp = 0; gp < 29; ++gp) {
    if (pinIsInternal(gp)) continue;
    pinMode(gp, INPUT_PULLDOWN);
    delayMicroseconds(3000);
    int conDown = digitalRead(gp);
    pinMode(gp, INPUT_PULLUP);
    delayMicroseconds(3000);
    int conUp = digitalRead(gp);
    pinMode(gp, INPUT);

    if (conDown == 1 && conUp == 1) {
      if (conPullup < 32) listaPullup[conPullup] = gp;
      conPullup++;
    } else if (conDown == 0 && conUp == 0) {
      if (aTierra < 32) listaTierra[aTierra] = gp;
      aTierra++;
    } else {
      alAire++;
    }
  }

  Serial.print("  Con pull-up externo: ");
  if (conPullup == 0) Serial.print("ninguno");
  for (uint8_t i = 0; i < conPullup && i < 32; ++i) {
    Serial.print("GP"); Serial.print(listaPullup[i]); Serial.print(' ');
  }
  Serial.println();

  // Un pin clavado a masa es una falla, no un estado normal: en un bus I2C
  // en reposo ambas lineas deben estar altas.
  Serial.print("  Forzados a GND: ");
  if (aTierra == 0) Serial.print("ninguno");
  for (uint8_t i = 0; i < aTierra && i < 32; ++i) {
    Serial.print("GP"); Serial.print(listaTierra[i]); Serial.print(' ');
  }
  Serial.println();

  Serial.print("  Al aire (sin nada conectado): "); Serial.println(alAire);
  Serial.println();

  if (aTierra > 0) {
    Serial.println("  AVISO: hay pines clavados a masa. Si uno de ellos es SDA o SCL,");
    Serial.println("  el bus esta inutilizable: con SDA en bajo el maestro cree que");
    Serial.println("  TODAS las direcciones contestan, porque lee la linea abajo");
    Serial.println("  durante el bit de confirmacion. Ese es un corto, no un sensor.");
    Serial.println();
  }

  if (conPullup == 0) {
    Serial.println("  DIAGNOSTICO: no hay ningun pull-up externo en toda la placa.");
    Serial.println("  El modulo no hace contacto electrico con ningun pin de datos,");
    Serial.println("  o no tiene 3V3. Si estuviera conectado y alimentado, SDA y SCL");
    Serial.println("  apareceran forzosamente en esta lista, incluso con el chip muerto.");
    Serial.println("  Sospechoso principal: la tira de pines del modulo sin soldar.");
  } else {
    Serial.println("  DIAGNOSTICO: ahi esta el bus alimentado y en reposo alto.");
    Serial.println("  Esos pines deberian ser SDA y SCL. Si el barrido I2C no");
    Serial.println("  encuentra nada en esa pareja, el cableado llega pero el chip");
    Serial.println("  no contesta, y entonces si apunta a un sensor danado.");
  }
}

// Convierte la placa en un probador de cables: pone cada pin en alto por turnos
// y lee los demas. Sirve para validar un dupont suelto entre dos GPIO.
void continuityMap() {
  banner("Mapa de continuidad entre pines");
  Serial.println("  Para probar un cable: conectar sus dos extremos a dos GPIO");
  Serial.println("  cualesquiera (por ejemplo GP2 y GP3) y volver a ejecutar.");
  Serial.println();

  uint8_t enlaces = 0;
  for (uint8_t emisor = 0; emisor < 29; ++emisor) {
    if (pinIsInternal(emisor)) continue;

    for (uint8_t gp = 0; gp < 29; ++gp) {
      if (pinIsInternal(gp) || gp == emisor) continue;
      pinMode(gp, INPUT_PULLDOWN);
    }

    pinMode(emisor, OUTPUT);
    digitalWrite(emisor, HIGH);
    delayMicroseconds(5000);

    for (uint8_t gp = emisor + 1; gp < 29; ++gp) {
      if (pinIsInternal(gp)) continue;
      if (digitalRead(gp) == 1) {
        Serial.print("   GP"); Serial.print(emisor);
        Serial.print("  <-->  GP"); Serial.println(gp);
        enlaces++;
      }
    }

    digitalWrite(emisor, LOW);
    pinMode(emisor, INPUT);
  }

  for (uint8_t gp = 0; gp < 29; ++gp) {
    if (!pinIsInternal(gp)) pinMode(gp, INPUT);
  }

  if (enlaces == 0) {
    Serial.println("   No se detecto continuidad entre ningun par de pines.");
    Serial.println("   Si acabas de poner un cable entre dos GPIO y no aparece,");
    Serial.println("   ese cable esta roto por dentro aunque se vea bien.");
  }
}

// ============================================================================
// I2C POR SOFTWARE - PRUEBA DE CABLES CRUZADOS
// ============================================================================
// El periferico I2C del RP2040 obliga a que SDA vaya en un pin congruente a 0
// modulo 4 y SCL en el siguiente. Si los cables estan invertidos, el hardware
// no puede siquiera intentarlo, y el sintoma es identico al de un chip muerto.
// Con I2C por software cualquier pin puede cumplir cualquier papel, asi que se
// pueden probar las dos orientaciones y distinguir un caso del otro.
static uint8_t bbSda = 4, bbScl = 5;

inline void bbDelay() { delayMicroseconds(5); }   // ~100 kHz

// Colector abierto: se suelta el pin para el nivel alto (lo sube el pull-up
// del modulo) y se conduce a masa para el nivel bajo. Nunca se fuerza el alto.
inline void bbSdaHigh() { pinMode(bbSda, INPUT); }
inline void bbSdaLow()  { pinMode(bbSda, OUTPUT); digitalWrite(bbSda, LOW); }
inline void bbSclHigh() { pinMode(bbScl, INPUT); }
inline void bbSclLow()  { pinMode(bbScl, OUTPUT); digitalWrite(bbScl, LOW); }
inline int  bbSdaRead() { pinMode(bbSda, INPUT); return digitalRead(bbSda); }

void bbStart() {
  bbSdaHigh(); bbSclHigh(); bbDelay();
  bbSdaLow();  bbDelay();
  bbSclLow();  bbDelay();
}

void bbStop() {
  bbSdaLow();  bbDelay();
  bbSclHigh(); bbDelay();
  bbSdaHigh(); bbDelay();
}

// Devuelve true si el esclavo confirmo el byte (ACK).
bool bbWriteByte(uint8_t value) {
  for (uint8_t i = 0; i < 8; ++i) {
    if (value & 0x80) bbSdaHigh(); else bbSdaLow();
    bbDelay();
    bbSclHigh(); bbDelay();
    bbSclLow();  bbDelay();
    value <<= 1;
  }
  bbSdaHigh(); bbDelay();          // se libera para que el esclavo conteste
  bbSclHigh(); bbDelay();
  int ack = bbSdaRead();
  bbSclLow(); bbDelay();
  return ack == 0;
}

// Comprueba que el bus este en reposo: ambas lineas altas gracias a los pull-up.
bool bbBusIdle() {
  bbSdaHigh(); bbSclHigh();
  delayMicroseconds(50);
  int sda = digitalRead(bbSda);
  int scl = digitalRead(bbScl);
  if (sda == 1 && scl == 1) return true;
  Serial.print("    bus en reposo anormal: SDA=");
  Serial.print(sda); Serial.print(" SCL="); Serial.println(scl);
  Serial.println("    Una linea baja en reposo indica corto a masa o un");
  Serial.println("    esclavo trabado a media transaccion.");
  return false;
}

uint8_t bbScan(uint8_t sdaPin, uint8_t sclPin) {
  bbSda = sdaPin;
  bbScl = sclPin;

  Serial.print("  Orientacion  SDA=GP"); Serial.print(sdaPin);
  Serial.print("  SCL=GP"); Serial.print(sclPin); Serial.println();

  bbBusIdle();

  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    bbStart();
    bool ack = bbWriteByte((uint8_t)(addr << 1));   // bit 0 = escritura
    bbStop();
    if (ack) {
      Serial.print("    responde 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("    ningun dispositivo confirmo su direccion");

  pinMode(sdaPin, INPUT);
  pinMode(sclPin, INPUT);
  return found;
}

void crossedWiringTest() {
  banner("I2C por software: prueba de las dos orientaciones");

  // Se localizan los dos pines con pull-up externo: ahi esta el bus.
  uint8_t pines[4];
  uint8_t n = 0;
  for (uint8_t gp = 0; gp < 29 && n < 4; ++gp) {
    if (pinIsInternal(gp)) continue;
    pinMode(gp, INPUT_PULLDOWN);
    delayMicroseconds(3000);
    int conDown = digitalRead(gp);
    pinMode(gp, INPUT);
    if (conDown == 1) pines[n++] = gp;
  }

  if (n < 2) {
    Serial.println("  No hay dos pines con pull-up externo, asi que no hay bus");
    Serial.println("  que probar. Ejecutar  p  para ver el estado del cableado.");
    return;
  }

  Serial.print("  Bus detectado en GP"); Serial.print(pines[0]);
  Serial.print(" y GP"); Serial.println(pines[1]);
  Serial.println();

  uint8_t directa = bbScan(pines[0], pines[1]);
  Serial.println();
  uint8_t invertida = bbScan(pines[1], pines[0]);

  Serial.println();
  if (directa == 0 && invertida == 0) {
    Serial.println("  VEREDICTO: el chip no confirma su direccion en ninguna");
    Serial.println("  orientacion. El bus esta cableado y alimentado, los pull-ups");
    Serial.println("  responden, pero el sensor no contesta. Descartado el cruce de");
    Serial.println("  cables, lo que queda es el chip o sus soldaduras al modulo.");
  } else if (invertida > 0 && directa == 0) {
    Serial.println("  VEREDICTO: SDA Y SCL ESTAN CRUZADOS.");
    Serial.print("  El sensor solo contesta con SDA en GP"); Serial.print(pines[1]);
    Serial.print(" y SCL en GP"); Serial.println(pines[0]);
    Serial.println("  El chip esta sano. Intercambiar los dos cables y el nodo");
    Serial.println("  lo vera de inmediato, sin tocar el firmware.");
  } else {
    Serial.println("  VEREDICTO: el sensor contesta en la orientacion directa.");
    Serial.println("  Si el I2C por hardware no lo veia, el problema es de");
    Serial.println("  velocidad o de tiempos, no de cableado. Bajar el reloj del bus.");
  }
}

// ============================================================================
// MENU
// ============================================================================
void printHelp() {
  Serial.println();
  Serial.println("============================================================");
  Serial.print("  "); Serial.println(TEST_VERSION);
  Serial.println("  Prueba del MPU6500 fuera del nodo. Letra y ENTER.");
  Serial.println("============================================================");
  Serial.println("    s  barrer todas las parejas SDA/SCL y localizar el sensor");
  Serial.println("    t  prueba completa en reposo, con veredicto");
  Serial.println("    l  lectura en vivo 15 s  (mover el modulo)");
  Serial.println("    d  volcado de registros de configuracion");
  Serial.println("    p  detectar pull-ups externos  (esta el modulo conectado?)");
  Serial.println("    c  mapa de continuidad entre pines  (probador de cables)");
  Serial.println("    x  I2C por software: probar SDA/SCL cruzados");
  Serial.println("    h  este menu");
  Serial.println("============================================================");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  uint32_t start = millis();
  while (!Serial && (millis() - start) < 4000) delay(50);
  delay(300);

  Serial.println();
  Serial.print("Arranque de "); Serial.println(TEST_VERSION);
  Serial.println("Conectar el modulo a 3V3 y GND. Los pines de datos se detectan solos.");

  sweepAllPins();
  if (foundBus != nullptr) runFullTest();

  printHelp();
  Serial.print("> ");
}

void handleCommand(const String &cmdIn) {
  String cmd = cmdIn;
  cmd.trim();
  if (cmd.length() == 0) { Serial.print("> "); return; }

  switch ((char)tolower(cmd.charAt(0))) {
    case 's': sweepAllPins(); break;
    case 't': runFullTest(); break;
    case 'l': streamLive(15); break;
    case 'd': dumpRegisters(); break;
    case 'p': detectPullups(); break;
    case 'c': continuityMap(); break;
    case 'x': crossedWiringTest(); break;
    case 'h': printHelp(); break;
    default:
      Serial.print("Comando no reconocido: ");
      Serial.println(cmd);
      Serial.println("Escribir  h  para ver el menu.");
      break;
  }
  Serial.println();
  Serial.print("> ");
}

void loop() {
  static String buffer = "";
  static uint32_t lastBlink = 0;

  if (millis() - lastBlink >= 1000) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        String cmd = buffer;
        buffer = "";
        handleCommand(cmd);
      }
    } else {
      buffer += c;
      if (buffer.length() > 40) buffer = "";
    }
  }
  delay(2);
}
