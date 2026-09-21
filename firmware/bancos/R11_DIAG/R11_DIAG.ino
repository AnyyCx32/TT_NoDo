// ============================================================================
//  R11-DIAG  -  BANCO DE PRUEBAS DEL NODO (no es el firmware de operacion)
//  Raspberry Pi Pico (RP2040) + ZED-F9P + ICM-20948 + MPU6500 + microSD
//  + OLED SSD1306 + SIM7600G-H
//
//  PROPOSITO:
//    Observar la cadena  sensores -> navegacion -> microSD -> LTE -> ThingSpeak
//    por consola USB, subiendo por niveles:
//      NIVEL 1  componente     (existe, responde, entrega datos coherentes)
//      NIVEL 2  subsistema     (sostiene el regimen de trabajo real)
//      NIVEL 3  nodo completo  (cadena extremo a extremo con veredicto)
//
//  NO modifica la logica de navegacion de R11. Solo mide y reporta.
//  Los pines, direcciones I2C, escalas y comandos AT son los mismos que usa
//  ProgramaFinal_R11_General_ENU.ino, para que lo observado aqui sea valido alla.
//
//  USO: abrir el Monitor Serie a 115200, escribir el comando y ENTER.
//       Escribir  h  para ver el menu.
// ============================================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <math.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <ICM_20948.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static const char DIAG_VERSION[] = "R11-DIAG v1.0";

// ============================================================================
// PINES  (identicos a R11)
// ============================================================================
#define LED_ONBOARD    25
#define LED_HEART_EXT  15
#define LED_SD         14
#define LED_GPSFAIL     3
#define LED_TS_OK       2

#define SD_CS_PIN      17   // SPI0 por omision: MISO=16 CS=17 SCK=18 MOSI=19

#define MODEM_TX        0
#define MODEM_RX        1
#define PWRKEY          6

#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5

#define VSYS_ADC_PIN   A3   // GP29 = VSYS/3 en la Pico

HardwareSerial *modem = &Serial1;

// ============================================================================
// PARAMETROS DE RED  (identicos a R11)
// ============================================================================
static const char APN[]        = "internet.itelcel.com";
static const char API_KEY[]    = "AQUI_TU_CLAVE_DE_ESCRITURA"   // sustituida al publicar; ver README;
static const char SERVER_URL[]       = "http://api.thingspeak.com/update";
// Variante cifrada, para comprobar si la redireccion 302 desaparece.
static const char SERVER_URL_HTTPS[] = "https://api.thingspeak.com/update";
static const int  ID_TREN = 0;
static const int  ID_NODO = 0;

// ============================================================================
// UMBRALES DE ACEPTACION  (los mismos criterios que aplica R11)
// ============================================================================
static const uint8_t  MIN_GNSS_SATS    = 6;
static const uint32_t HACC_LIMIT_MM    = 10000;
static const uint16_t PDOP_LIMIT_CENTI = 600;
static const float    MPU_LSB_PER_G    = 8192.0f;   // +/-4 g
static const uint32_t I2C_CLOCK_HZ     = 400000;

// Direcciones esperadas en el bus I2C.
static const uint8_t ADDR_OLED = 0x3C;
static const uint8_t ADDR_GNSS = 0x42;
static const uint8_t ADDR_MPU  = 0x68;
static const uint8_t ADDR_ICM  = 0x69;

// Registros MPU6500.
static const uint8_t MPU_RA_WHO_AM_I     = 0x75;
static const uint8_t MPU_RA_PWR_MGMT_1   = 0x6B;
static const uint8_t MPU_RA_SMPLRT_DIV   = 0x19;
static const uint8_t MPU_RA_CONFIG       = 0x1A;
static const uint8_t MPU_RA_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_RA_ACCEL_XOUT_H = 0x3B;

// ============================================================================
// OBJETOS
// ============================================================================
SFE_UBLOX_GNSS myGPS;
ICM_20948_I2C  myICM;
#define ICM_ADDR ICM_20948_I2C_ADDR_AD1
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ============================================================================
// TABLERO DE RESULTADOS
// ============================================================================
enum TestId : uint8_t {
  T_I2C = 0, T_OLED, T_ICM, T_MPU, T_SD, T_GNSS, T_MODEM, T_LED,
  T_IMU_STREAM, T_SD_RATE, T_HTTP, T_CHAIN, T_COUNT
};

enum Verdict : uint8_t { V_NOTRUN = 0, V_PASS, V_WARN, V_FAIL };

struct TestRecord {
  Verdict  verdict;
  char     detail[56];
  uint32_t whenMs;
};

TestRecord board[T_COUNT];

static const char *testName(uint8_t id) {
  switch (id) {
    case T_I2C:        return "N1 bus I2C";
    case T_OLED:       return "N1 OLED SSD1306";
    case T_ICM:        return "N1 ICM-20948";
    case T_MPU:        return "N1 MPU6500";
    case T_SD:         return "N1 microSD";
    case T_GNSS:       return "N1 ZED-F9P";
    case T_MODEM:      return "N1 SIM7600G";
    case T_LED:        return "N1 indicadores";
    case T_IMU_STREAM: return "N2 IMU en regimen";
    case T_SD_RATE:    return "N2 SD 20 Hz sostenido";
    case T_HTTP:       return "N2 LTE + HTTP";
    case T_CHAIN:      return "N3 cadena completa";
  }
  return "?";
}

static const char *verdictTag(Verdict v) {
  switch (v) {
    case V_PASS: return "[ OK ]";
    case V_WARN: return "[WARN]";
    case V_FAIL: return "[FALLA]";
    default:     return "[ -- ]";
  }
}

void setResult(uint8_t id, Verdict v, const char *detail) {
  if (id >= T_COUNT) return;
  board[id].verdict = v;
  board[id].whenMs = millis();
  snprintf(board[id].detail, sizeof(board[id].detail), "%s", detail ? detail : "");
  Serial.print("  -> ");
  Serial.print(verdictTag(v));
  Serial.print(' ');
  Serial.print(testName(id));
  if (detail && detail[0]) { Serial.print(" : "); Serial.print(detail); }
  Serial.println();
}

void banner(const char *title) {
  Serial.println();
  Serial.println("============================================================");
  Serial.print("  "); Serial.println(title);
  Serial.println("============================================================");
}

// Estado de subsistemas ya inicializados en esta sesion de diagnostico.
bool oledReady = false;
bool sdReady   = false;
bool icmReady  = false;
bool mpuReady  = false;
bool gnssReady = false;

// ============================================================================
// UTILIDADES I2C DIRECTAS
// ============================================================================
bool i2cWrite8(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cReadN(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom((int)address, (int)count, (int)true);
  if (received != count) return false;
  for (uint8_t i = 0; i < count; ++i) buffer[i] = Wire.read();
  return true;
}

bool i2cPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// ============================================================================
// NIVEL 1 - BUS I2C
// ============================================================================
void testI2cScan() {
  banner("NIVEL 1 - Exploracion del bus I2C (SDA=GP4 SCL=GP5, 400 kHz)");

  uint8_t found = 0;
  bool oled = false, gnss = false, mpu = false, icm = false;

  for (uint8_t addr = 1; addr < 127; ++addr) {
    if (!i2cPresent(addr)) continue;
    found++;
    Serial.print("  0x");
    if (addr < 16) Serial.print('0');
    Serial.print(addr, HEX);
    Serial.print("  ");
    if (addr == ADDR_OLED)      { Serial.print("OLED SSD1306"); oled = true; }
    else if (addr == ADDR_GNSS) { Serial.print("ZED-F9P");      gnss = true; }
    else if (addr == ADDR_MPU)  { Serial.print("MPU6500");      mpu  = true; }
    else if (addr == ADDR_ICM)  { Serial.print("ICM-20948");    icm  = true; }
    else                          Serial.print("desconocido");
    Serial.println();
  }

  if (found == 0) {
    Serial.println("  No respondio ningun dispositivo.");
    Serial.println("  Revisar: alimentacion 3V3, GND comun, pull-ups en SDA/SCL.");
  }

  char detail[56];
  snprintf(detail, sizeof(detail), "%u disp. OLED%c GNSS%c MPU%c ICM%c",
           found, oled ? '+' : '-', gnss ? '+' : '-',
           mpu ? '+' : '-', icm ? '+' : '-');

  uint8_t missing = (oled ? 0 : 1) + (gnss ? 0 : 1) + (mpu ? 0 : 1) + (icm ? 0 : 1);
  setResult(T_I2C, missing == 0 ? V_PASS : (missing >= 3 ? V_FAIL : V_WARN), detail);
}

// ============================================================================
// NIVEL 1 - OLED
// ============================================================================
void testOled() {
  banner("NIVEL 1 - OLED SSD1306 en 0x3C");

  oledReady = display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED);
  if (!oledReady) {
    Serial.println("  display.begin fallo: no hay respuesta en 0x3C.");
    setResult(T_OLED, V_FAIL, "sin respuesta en 0x3C");
    return;
  }

  Serial.println("  Secuencia visual: patron, texto y limpieza (aprox. 4 s).");
  Serial.println("  Confirmar a la vista que los 128x64 pixeles responden.");

  display.clearDisplay();
  for (int16_t y = 0; y < 64; y += 8)
    for (int16_t x = 0; x < 128; x += 8)
      if (((x / 8) + (y / 8)) % 2 == 0) display.fillRect(x, y, 8, 8, SSD1306_WHITE);
  display.display();
  delay(1500);

  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(6, 10);  display.println(DIAG_VERSION);
  display.setCursor(6, 24);  display.println("Prueba OLED");
  display.setCursor(6, 38);  display.println("128x64 SSD1306");
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("R11-DIAG listo");
  display.display();

  setResult(T_OLED, V_PASS, "begin OK, patron mostrado");
}

// ============================================================================
// NIVEL 1 - ICM-20948
// ============================================================================
bool configureIcm() {
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                      ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok) return false;

  ICM_20948_fss_t fss;
  fss.a = gpm2;
  fss.g = dps250;
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  if (myICM.status != ICM_20948_Stat_Ok) return false;

  ICM_20948_dlpcfg_t dlpf;
  dlpf.a = acc_d23bw9_n34bw4;
  dlpf.g = gyr_d23bw9_n35bw9;
  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpf);
  if (myICM.status != ICM_20948_Stat_Ok) return false;
  myICM.enableDLPF((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), true);
  if (myICM.status != ICM_20948_Stat_Ok) return false;

  ICM_20948_smplrt_t rate;
  rate.a = 10;
  rate.g = 10;
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), rate);
  return myICM.status == ICM_20948_Stat_Ok;
}

void testIcm20948() {
  banner("NIVEL 1 - ICM-20948 en 0x69 (acel +/-2 g, gyro +/-250 dps)");

  ICM_20948_Status_e st = myICM.begin(Wire, ICM_ADDR);
  icmReady = (st == ICM_20948_Stat_Ok);
  if (!icmReady) {
    Serial.print("  begin fallo, estado="); Serial.println((int)st);
    setResult(T_ICM, V_FAIL, "begin fallo");
    return;
  }
  Serial.println("  begin OK (WHO_AM_I validado por la libreria).");

  if (!configureIcm()) {
    Serial.println("  Configuracion de escalas/DLPF fallo.");
    setResult(T_ICM, V_FAIL, "configuracion fallo");
    icmReady = false;
    return;
  }
  Serial.println("  Configurada igual que R11. Tomando 200 muestras en reposo...");
  Serial.println("  NO MOVER EL NODO durante esta prueba.");

  double sa[3] = {0, 0, 0}, sa2[3] = {0, 0, 0};
  double sg[3] = {0, 0, 0};
  double smag = 0;
  int n = 0, nmag = 0;
  uint32_t start = millis();

  while (n < 200 && (millis() - start) < 6000) {
    if (!myICM.dataReady()) { delay(2); continue; }
    myICM.getAGMT();
    float a[3] = { myICM.accX() / 1000.0f, myICM.accY() / 1000.0f, myICM.accZ() / 1000.0f };
    float g[3] = { myICM.gyrX(), myICM.gyrY(), myICM.gyrZ() };
    for (int i = 0; i < 3; ++i) { sa[i] += a[i]; sa2[i] += (double)a[i] * a[i]; sg[i] += g[i]; }
    float mx = myICM.magX(), my = myICM.magY(), mz = myICM.magZ();
    float mm = sqrtf(mx * mx + my * my + mz * mz);
    if (mm > 1.0f && mm < 200.0f) { smag += mm; nmag++; }
    n++;
  }

  if (n < 50) {
    Serial.print("  Solo se obtuvieron "); Serial.print(n); Serial.println(" muestras.");
    setResult(T_ICM, V_FAIL, "flujo de datos insuficiente");
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
  float gyroBias = sqrtf(mg[0] * mg[0] + mg[1] * mg[1] + mg[2] * mg[2]);
  float noise = (sd[0] + sd[1] + sd[2]) / 3.0f;
  float magField = nmag > 0 ? (float)(smag / nmag) : 0.0f;

  Serial.print("  muestras="); Serial.println(n);
  Serial.print("  acel media [g]  X="); Serial.print(ma[0], 4);
  Serial.print(" Y="); Serial.print(ma[1], 4);
  Serial.print(" Z="); Serial.println(ma[2], 4);
  Serial.print("  |a| = "); Serial.print(magG, 4);
  Serial.println(" g  (esperado 0.95 a 1.05 en reposo)");
  Serial.print("  ruido acel [g] = "); Serial.println(noise, 5);
  Serial.print("  sesgo gyro [dps] X="); Serial.print(mg[0], 3);
  Serial.print(" Y="); Serial.print(mg[1], 3);
  Serial.print(" Z="); Serial.print(mg[2], 3);
  Serial.print("  |bias|="); Serial.println(gyroBias, 3);
  Serial.print("  campo magnetico = "); Serial.print(magField, 1);
  Serial.println(" uT  (esperado 25 a 65 sin iman cerca)");

  // Eje vertical dominante: asi determina R11 el piso al arrancar.
  int upAxis = 0;
  for (int i = 1; i < 3; ++i) if (fabsf(ma[i]) > fabsf(ma[upAxis])) upAxis = i;
  Serial.print("  gravedad dominante en el eje ");
  Serial.print((char)('X' + upAxis));
  Serial.print(ma[upAxis] > 0 ? " positivo" : " negativo");
  Serial.println("  (equivale al piso que aprende R11)");

  Verdict v = V_PASS;
  char detail[56];
  if (magG < 0.90f || magG > 1.10f) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "|a|=%.3f g fuera de rango", magG);
  } else if (noise < 0.0002f) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "lectura congelada, ruido nulo");
  } else if (gyroBias > 5.0f) {
    v = V_WARN; snprintf(detail, sizeof(detail), "sesgo gyro alto %.1f dps", gyroBias);
  } else if (nmag == 0) {
    v = V_WARN; snprintf(detail, sizeof(detail), "|a|=%.3f g, magnetometro sin dato", magG);
  } else {
    snprintf(detail, sizeof(detail), "|a|=%.3f g, bias %.2f dps, %.0f uT", magG, gyroBias, magField);
  }
  setResult(T_ICM, v, detail);
}

// ============================================================================
// NIVEL 1 - MPU6500
// ============================================================================
bool initMpu() {
  uint8_t who = 0;
  if (!i2cReadN(ADDR_MPU, MPU_RA_WHO_AM_I, &who, 1)) return false;
  Serial.print("  WHO_AM_I = 0x"); Serial.print(who, HEX);
  if (who == 0x70)      Serial.println("  (MPU6500)");
  else if (who == 0x71) Serial.println("  (MPU9250)");
  else if (who == 0x68) Serial.println("  (MPU6050)");
  else                  Serial.println("  (no reconocido)");

  if (!i2cWrite8(ADDR_MPU, MPU_RA_PWR_MGMT_1, 0x00)) return false;
  delay(10);
  if (!i2cWrite8(ADDR_MPU, MPU_RA_SMPLRT_DIV, 9)) return false;
  if (!i2cWrite8(ADDR_MPU, MPU_RA_CONFIG, 0x03)) return false;
  if (!i2cWrite8(ADDR_MPU, MPU_RA_ACCEL_CONFIG, 0x08)) return false;
  return true;
}

bool readMpuG(float &ax, float &ay, float &az) {
  uint8_t b[6];
  if (!i2cReadN(ADDR_MPU, MPU_RA_ACCEL_XOUT_H, b, 6)) return false;
  ax = (float)(int16_t)((b[0] << 8) | b[1]) / MPU_LSB_PER_G;
  ay = (float)(int16_t)((b[2] << 8) | b[3]) / MPU_LSB_PER_G;
  az = (float)(int16_t)((b[4] << 8) | b[5]) / MPU_LSB_PER_G;
  return true;
}

void testMpu6500() {
  banner("NIVEL 1 - MPU6500 en 0x68 (acel +/-4 g, fuente de vibracion)");

  mpuReady = initMpu();
  if (!mpuReady) {
    Serial.println("  No respondio o fallo la configuracion.");
    setResult(T_MPU, V_FAIL, "sin respuesta en 0x68");
    return;
  }
  Serial.println("  Configurado igual que R11. 200 muestras en reposo, NO MOVER...");

  double sa[3] = {0, 0, 0}, sa2[3] = {0, 0, 0};
  int n = 0;
  uint32_t start = millis();
  while (n < 200 && (millis() - start) < 5000) {
    float x, y, z;
    if (readMpuG(x, y, z)) {
      float a[3] = {x, y, z};
      for (int i = 0; i < 3; ++i) { sa[i] += a[i]; sa2[i] += (double)a[i] * a[i]; }
      n++;
    }
    delay(5);
  }

  if (n < 50) {
    setResult(T_MPU, V_FAIL, "flujo de datos insuficiente");
    return;
  }

  float ma[3], sd[3];
  for (int i = 0; i < 3; ++i) {
    ma[i] = sa[i] / n;
    float var = (sa2[i] / n) - (double)ma[i] * ma[i];
    sd[i] = var > 0 ? sqrtf(var) : 0.0f;
  }
  float magG = sqrtf(ma[0] * ma[0] + ma[1] * ma[1] + ma[2] * ma[2]);
  float noise = (sd[0] + sd[1] + sd[2]) / 3.0f;

  Serial.print("  acel media [g] X="); Serial.print(ma[0], 4);
  Serial.print(" Y="); Serial.print(ma[1], 4);
  Serial.print(" Z="); Serial.println(ma[2], 4);
  Serial.print("  |a| = "); Serial.print(magG, 4); Serial.println(" g");
  Serial.print("  ruido (piso de vibracion) = "); Serial.print(noise, 5);
  Serial.println(" g  (umbral de reposo en R11: 0.035 g)");

  Verdict v = V_PASS;
  char detail[56];
  if (magG < 0.90f || magG > 1.10f) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "|a|=%.3f g fuera de rango", magG);
  } else if (noise < 0.0002f) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "lectura congelada");
  } else if (noise > 0.035f) {
    v = V_WARN; snprintf(detail, sizeof(detail), "ruido %.4f g supera umbral ZUPT", noise);
  } else {
    snprintf(detail, sizeof(detail), "|a|=%.3f g, ruido %.4f g", magG, noise);
  }
  setResult(T_MPU, v, detail);
}

// ============================================================================
// NIVEL 1 - microSD
// ============================================================================
void listSdFiles() {
  File root = SD.open("/");
  if (!root || !root.isDirectory()) { if (root) root.close(); return; }

  Serial.println("  Archivos en la tarjeta:");
  int count = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    String name = entry.name();
    int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(slash + 1);
    if (!entry.isDirectory()) {
      Serial.print("    ");
      Serial.print(name);
      for (int i = name.length(); i < 20; ++i) Serial.print(' ');
      Serial.print(entry.size());
      Serial.println(" bytes");
      count++;
    }
    entry.close();
    if (count >= 30) { Serial.println("    ..."); break; }
  }
  root.close();
  if (count == 0) Serial.println("    (vacia)");
}

void testSdCard() {
  banner("NIVEL 1 - microSD por SPI0 (CS=GP17, MISO=16 SCK=18 MOSI=19)");

  SD.end(false);
  sdReady = SD.begin(SD_CS_PIN);
  if (!sdReady) {
    Serial.println("  SD.begin fallo.");
    Serial.println("  Revisar: tarjeta insertada, formato FAT32, CS en GP17,");
    Serial.println("  soldadura del socket y 3V3 estable durante el arranque.");
    setResult(T_SD, V_FAIL, "SD.begin fallo");
    return;
  }
  Serial.println("  SD.begin OK.");

  FSInfo info;
  if (SDFS.info(info) && info.totalBytes > 0) {
    uint32_t totalMB = (uint32_t)(info.totalBytes / 1048576ULL);
    uint32_t usedMB  = (uint32_t)(info.usedBytes / 1048576ULL);
    uint8_t percent  = (uint8_t)((info.usedBytes * 100ULL) / info.totalBytes);
    Serial.print("  Capacidad: "); Serial.print(usedMB); Serial.print(" MB usados de ");
    Serial.print(totalMB); Serial.print(" MB ("); Serial.print(percent); Serial.println(" %)");
    Serial.println("  (R11 borra el VIB mas antiguo al superar 80 %)");
  } else {
    Serial.println("  ADVERTENCIA: SDFS.info fallo; R11 declararia la tarjeta fuera de servicio.");
  }

  // Escritura/lectura/borrado, igual que probeSdReadWrite() de R11.
  const char *probe = "DIAGPRB.TXT";
  if (SD.exists(probe)) SD.remove(probe);
  File f = SD.open(probe, FILE_WRITE);
  bool ok = false;
  if (f) {
    f.clearWriteError();
    f.println("SD_OK");
    f.flush();
    ok = (f.getWriteError() == 0);
    f.close();
  }
  if (ok) {
    f = SD.open(probe, FILE_READ);
    if (f) {
      String v = f.readStringUntil('\n');
      v.trim();
      f.close();
      ok = (v == "SD_OK");
    } else ok = false;
  }
  SD.remove(probe);
  Serial.print("  Escritura + lectura + borrado: ");
  Serial.println(ok ? "correcto" : "FALLO");

  listSdFiles();

  setResult(T_SD, ok ? V_PASS : V_FAIL, ok ? "montada, E/S verificada" : "E/S fallo");
}

// Encabezado y ultimas lineas del VIB mas reciente (evidencia P-09 / P-10).
void dumpNewestVib(int tailLines) {
  banner("Evidencia microSD - archivo VIB mas reciente");
  if (!sdReady) { Serial.println("  Ejecutar antes el comando 5 (microSD)."); return; }

  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    Serial.println("  No se pudo abrir la raiz.");
    return;
  }

  String newest = "";
  uint32_t newestSize = 0;
  while (true) {
    File e = root.openNextFile();
    if (!e) break;
    String name = e.name();
    int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(slash + 1);
    if (!e.isDirectory() && name.startsWith("VIB_") && name.endsWith(".CSV")) {
      // El nombre lleva la fecha, asi que el mayor alfabeticamente es el mas nuevo.
      if (name > newest) { newest = name; newestSize = e.size(); }
    }
    e.close();
  }
  root.close();

  if (newest.length() == 0) {
    Serial.println("  No hay archivos VIB_YYYYMMDD.CSV en la tarjeta.");
    Serial.println("  R11 solo los crea si la SD monta Y el MPU6500 responde.");
    return;
  }

  Serial.print("  Archivo: "); Serial.print(newest);
  Serial.print("  ("); Serial.print(newestSize); Serial.println(" bytes)");

  File f = SD.open(newest.c_str(), FILE_READ);
  if (!f) { Serial.println("  No se pudo abrir."); return; }

  Serial.println("  --- encabezado ---");
  for (int i = 0; i < 2 && f.available(); ++i) {
    String line = f.readStringUntil('\n');
    line.trim();
    Serial.print("  "); Serial.println(line);
  }

  uint32_t lines = 2;
  while (f.available()) { if (f.read() == '\n') lines++; }
  Serial.print("  lineas totales ~ "); Serial.println(lines);

  Serial.print("  --- ultimas "); Serial.print(tailLines); Serial.println(" lineas ---");
  uint32_t skip = (lines > (uint32_t)tailLines) ? lines - tailLines : 0;
  f.seek(0);
  uint32_t idx = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (idx++ < skip) continue;
    line.trim();
    if (line.length()) { Serial.print("  "); Serial.println(line); }
  }
  f.close();
}

// Clasifica el encabezado de un VIB para saber que firmware lo escribio.
const char *vibHeaderFormat(const String &header) {
  if (header.indexOf("icmCalQ") >= 0)      return "R11 (33 col)";
  if (header.indexOf("headingTrue") >= 0)  return "R11 (33 col)";
  if (header.indexOf("millis") >= 0)       return "R10 (con millis)";
  if (header.indexOf("dirNS") >= 0)        return "anterior a R10 (10 col)";
  return "desconocido";
}

// Catalogo de todos los VIB: quien los escribio y cuantas muestras tienen.
void catalogVibFiles() {
  banner("Catalogo de archivos VIB en la tarjeta");
  if (!sdReady) { Serial.println("  Ejecutar antes el comando 5 (microSD)."); return; }

  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    Serial.println("  No se pudo abrir la raiz.");
    return;
  }

  Serial.println("  archivo             bytes   muestras  formato del encabezado");
  Serial.println("  ------------------------------------------------------------");

  uint32_t withData = 0, headerOnly = 0;
  while (true) {
    File e = root.openNextFile();
    if (!e) break;
    String name = e.name();
    int slash = name.lastIndexOf('/');
    if (slash >= 0) name = name.substring(slash + 1);
    if (e.isDirectory() || !name.startsWith("VIB_") || !name.endsWith(".CSV")) { e.close(); continue; }

    uint32_t size = e.size();
    e.close();

    File f = SD.open(name.c_str(), FILE_READ);
    if (!f) continue;

    // Se salta la linea "# firmware=" si existe y se toma la de columnas.
    String header = f.readStringUntil('\n');
    header.trim();
    if (header.startsWith("#")) { header = f.readStringUntil('\n'); header.trim(); }

    uint32_t lines = 1;
    while (f.available()) { if (f.read() == '\n') lines++; }
    f.close();

    uint32_t samples = (lines > 1) ? lines - 1 : 0;
    if (header.startsWith("#")) samples = 0;
    if (samples > 0) withData++; else headerOnly++;

    Serial.print("  ");
    Serial.print(name);
    for (int i = name.length(); i < 20; ++i) Serial.print(' ');
    Serial.print(size);
    for (int i = String(size).length(); i < 8; ++i) Serial.print(' ');
    Serial.print(samples);
    for (int i = String(samples).length(); i < 10; ++i) Serial.print(' ');
    Serial.println(vibHeaderFormat(header));
  }
  root.close();

  Serial.println("  ------------------------------------------------------------");
  Serial.print("  con muestras: "); Serial.print(withData);
  Serial.print("   solo encabezado: "); Serial.println(headerOnly);
  Serial.println("  Un archivo con 0 muestras significa que se creo el encabezado");
  Serial.println("  pero nunca se escribio una linea de adquisicion.");
}

// Volcado de las ultimas lineas de un archivo indicado por nombre.
void dumpFileByName(const String &name, int tailLines) {
  banner("Volcado de archivo de la microSD");
  if (!sdReady) { Serial.println("  Ejecutar antes el comando 5 (microSD)."); return; }

  if (!SD.exists(name.c_str())) {
    Serial.print("  No existe: "); Serial.println(name);
    return;
  }

  File f = SD.open(name.c_str(), FILE_READ);
  if (!f) { Serial.println("  No se pudo abrir."); return; }

  Serial.print("  Archivo: "); Serial.print(name);
  Serial.print("  ("); Serial.print(f.size()); Serial.println(" bytes)");

  uint32_t lines = 0;
  while (f.available()) { if (f.read() == '\n') lines++; }
  Serial.print("  lineas totales ~ "); Serial.println(lines);

  f.seek(0);
  Serial.println("  --- primeras 2 lineas ---");
  for (int i = 0; i < 2 && f.available(); ++i) {
    String line = f.readStringUntil('\n');
    line.trim();
    Serial.print("  "); Serial.println(line);
  }

  Serial.print("  --- ultimas "); Serial.print(tailLines); Serial.println(" lineas ---");
  uint32_t skip = (lines > (uint32_t)tailLines) ? lines - tailLines : 0;
  f.seek(0);
  uint32_t idx = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (idx++ < skip) continue;
    line.trim();
    if (line.length()) { Serial.print("  "); Serial.println(line); }
  }
  f.close();
}

// ============================================================================
// NIVEL 1 - ZED-F9P
// ============================================================================
void testGnss(uint32_t seconds) {
  banner("NIVEL 1 - ZED-F9P en 0x42 (UBX-NAV-PVT por I2C)");

  if (!gnssReady) {
    gnssReady = myGPS.begin(Wire);
    if (!gnssReady) {
      Serial.println("  begin fallo: no responde en 0x42.");
      setResult(T_GNSS, V_FAIL, "sin respuesta en 0x42");
      return;
    }
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(5);
    myGPS.setAutoPVT(true);
    Serial.println("  begin OK, configurado igual que R11 (5 Hz, autoPVT).");
    delay(400);
  }

  Serial.print("  Observando "); Serial.print(seconds);
  Serial.println(" s. Bajo techo es normal no obtener fijado.");
  Serial.println("  fix sats hAcc[m] pdop sAcc[m/s]  lat          lon          vel[km/h] rumbo");

  uint32_t start = millis();
  uint32_t pvtCount = 0;
  uint8_t bestFix = 0, bestSats = 0;
  uint32_t bestHacc = 0xFFFFFFFF;
  bool usableSeen = false;
  double lastLat = 0, lastLon = 0;
  uint32_t lastPrint = 0;

  while ((millis() - start) < seconds * 1000UL) {
    if (myGPS.getPVT(50)) {
      pvtCount++;
      uint8_t fix = myGPS.getFixType();
      uint8_t sats = myGPS.getSIV();
      uint32_t hAcc = myGPS.getHorizontalAccEst();
      uint16_t pdop = myGPS.getPDOP();
      uint32_t sAcc = myGPS.getSpeedAccEst();
      double lat = myGPS.getLatitude() / 1e7;
      double lon = myGPS.getLongitude() / 1e7;
      float kmh = myGPS.getGroundSpeed() / 1000.0f * 3.6f;
      float head = myGPS.getHeading() / 100000.0f;

      if (fix > bestFix) bestFix = fix;
      if (sats > bestSats) bestSats = sats;
      if (hAcc > 0 && hAcc < bestHacc) bestHacc = hAcc;
      lastLat = lat; lastLon = lon;

      bool usable = (fix >= 3) && (sats >= MIN_GNSS_SATS) &&
                    (hAcc > 0 && hAcc <= HACC_LIMIT_MM) &&
                    (pdop > 0 && pdop <= PDOP_LIMIT_CENTI);
      if (usable) usableSeen = true;

      if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        Serial.print("   "); Serial.print(fix);
        Serial.print("    "); Serial.print(sats);
        Serial.print("    "); Serial.print(hAcc / 1000.0f, 2);
        Serial.print("   "); Serial.print(pdop / 100.0f, 2);
        Serial.print("    "); Serial.print(sAcc / 1000.0f, 2);
        Serial.print("     "); Serial.print(lat, 6);
        Serial.print("   "); Serial.print(lon, 6);
        Serial.print("   "); Serial.print(kmh, 1);
        Serial.print("     "); Serial.print(head, 1);
        Serial.print(usable ? "   <- utilizable por R11" : "");
        Serial.println();
      }
    }
    delay(5);
  }

  Serial.print("  tramas PVT recibidas: "); Serial.println(pvtCount);
  Serial.print("  mejor fix="); Serial.print(bestFix);
  Serial.print("  mejor sats="); Serial.print(bestSats);
  Serial.print("  mejor hAcc=");
  if (bestHacc == 0xFFFFFFFF) Serial.println("n/d");
  else { Serial.print(bestHacc / 1000.0f, 2); Serial.println(" m"); }
  Serial.print("  ultima posicion: "); Serial.print(lastLat, 7);
  Serial.print(", "); Serial.println(lastLon, 7);
  Serial.println("  Criterio R11: fix>=3, sats>=6, hAcc<=10 m, pdop<=6.00");

  char detail[56];
  Verdict v;
  if (pvtCount == 0) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "sin tramas PVT");
  } else if (usableSeen) {
    v = V_PASS; snprintf(detail, sizeof(detail), "fix %u, %u sats, hAcc %.1f m",
                         bestFix, bestSats, bestHacc / 1000.0f);
  } else {
    v = V_WARN; snprintf(detail, sizeof(detail), "responde, fix %u %u sats: sin cielo",
                         bestFix, bestSats);
  }
  setResult(T_GNSS, v, detail);
}

// ============================================================================
// NIVEL 1 - SIM7600G
// ============================================================================
// La FIFO de recepcion por omision es de 32 bytes. Una respuesta como
// AT+HTTPHEAD llega en una rafaga de 160 bytes y se pierden caracteres.
// setFIFOSize debe llamarse antes de begin para que tenga efecto.
void modemBegin() {
  static bool sized = false;
  if (!sized) {
    Serial1.setFIFOSize(1024);  // setFIFOSize vive en SerialUART, no en HardwareSerial
    sized = true;
  }
  modem->begin(115200);
}

void modemPowerPulse() {
  Serial.println("  Pulso de encendido en PWRKEY (GP6)...");
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(100);
  digitalWrite(PWRKEY, HIGH);
  delay(900);
  digitalWrite(PWRKEY, LOW);
  delay(4500);
}

// El SIM7600 emite lineas no solicitadas con retraso (por ejemplo el resto de
// "+IP ERROR: Network is already opened"). Si no se dejan llegar antes de
// limpiar, aparecen como respuesta del comando siguiente y lo desalinean todo.
void modemFlush() {
  uint32_t start = millis();
  while ((millis() - start) < 150) {
    while (modem->available()) modem->read();
    delay(10);
  }
}

// Envia un comando y devuelve la respuesta hasta OK/ERROR o hasta el timeout.
String modemCommand(const char *cmd, uint32_t timeoutMs, bool echo = true) {
  modemFlush();
  if (echo) { Serial.print("  MDM> "); Serial.println(cmd); }
  modem->println(cmd);

  String response = "";
  response.reserve(1024);
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    while (modem->available()) {
      char c = (char)modem->read();
      if (c == '\r') continue;
      response += c;
      if (response.length() > 900) response.remove(0, 400);
    }
    if (response.indexOf("\nOK") >= 0 || response.startsWith("OK") ||
        response.indexOf("ERROR") >= 0 || response.indexOf("DOWNLOAD") >= 0) break;
    delay(5);
  }

  if (echo) {
    String pretty = response;
    pretty.trim();
    pretty.replace("\n", " | ");
    Serial.print("  MDM< ");
    Serial.println(pretty.length() ? pretty : "(sin respuesta)");
  }
  return response;
}

bool modemSaysOk(const String &r) { return r.indexOf("OK") >= 0; }

int parseCsq(const String &r) {
  int p = r.indexOf("+CSQ:");
  if (p < 0) return -1;
  return r.substring(p + 5, r.indexOf(',', p)).toInt();
}

void testModem(bool doPowerPulse) {
  banner("NIVEL 1 - SIM7600G-H por Serial1 (GP0=TX GP1=RX, 115200)");

  modemBegin();
  delay(200);

  String r = modemCommand("AT", 2000);
  if (!modemSaysOk(r)) {
    if (doPowerPulse) {
      Serial.println("  Sin respuesta. Aplicando pulso de encendido y reintentando.");
      modemPowerPulse();
      r = modemCommand("AT", 3000);
    }
    if (!modemSaysOk(r)) {
      Serial.println("  El modem no responde a AT.");
      Serial.println("  Revisar: alimentacion propia del SIM7600 (picos cercanos a 2 A),");
      Serial.println("  cruce TX/RX (GP0 -> RX del modem, GP1 <- TX del modem) y GND comun.");
      setResult(T_MODEM, V_FAIL, "sin respuesta a AT");
      return;
    }
  }
  Serial.println("  Responde a AT.");

  modemCommand("ATE0", 2000);
  modemCommand("ATI", 3000);

  String cpin = modemCommand("AT+CPIN?", 5000);
  bool simOk = cpin.indexOf("READY") >= 0;
  Serial.println(simOk ? "  SIM lista." : "  SIM NO lista (ausente, mal insertada o con PIN).");

  String csq = modemCommand("AT+CSQ", 3000);
  int rssi = parseCsq(csq);
  if (rssi >= 0 && rssi < 99) {
    int dbm = -113 + 2 * rssi;
    Serial.print("  Senal: CSQ="); Serial.print(rssi);
    Serial.print("  ~"); Serial.print(dbm); Serial.println(" dBm");
    if (rssi < 8) Serial.println("  Senal debil: por debajo de CSQ 8 el HTTP suele fallar.");
  } else {
    Serial.println("  Senal desconocida (CSQ=99): sin cobertura o antena desconectada.");
  }

  String creg  = modemCommand("AT+CREG?", 3000);
  String cgreg = modemCommand("AT+CGREG?", 3000);
  bool registered = (creg.indexOf(",1") >= 0 || creg.indexOf(",5") >= 0 ||
                     cgreg.indexOf(",1") >= 0 || cgreg.indexOf(",5") >= 0);
  Serial.println(registered ? "  Registrado en la red." : "  NO registrado en la red.");

  modemCommand("AT+COPS?", 8000);
  String cgatt = modemCommand("AT+CGATT?", 5000);
  bool attached = cgatt.indexOf("+CGATT: 1") >= 0;
  Serial.println(attached ? "  Adjunto al servicio de datos." : "  Sin adjuntar a datos.");

  Verdict v;
  char detail[56];
  if (!simOk) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "AT OK pero SIM no lista");
  } else if (!registered) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "SIM lista, sin registro de red");
  } else if (rssi >= 0 && rssi < 8) {
    v = V_WARN; snprintf(detail, sizeof(detail), "registrado, senal baja CSQ=%d", rssi);
  } else if (!attached) {
    v = V_WARN; snprintf(detail, sizeof(detail), "registrado, sin adjuntar datos");
  } else {
    v = V_PASS; snprintf(detail, sizeof(detail), "registrado, CSQ=%d, datos activos", rssi);
  }
  setResult(T_MODEM, v, detail);
}

// ============================================================================
// NIVEL 1 - INDICADORES
// ============================================================================
void testLeds() {
  banner("NIVEL 1 - LED indicadores");
  struct { uint8_t pin; const char *name; } leds[] = {
    { LED_ONBOARD,   "GP25 interno (latido)" },
    { LED_HEART_EXT, "GP15 latido externo" },
    { LED_SD,        "GP14 actividad SD" },
    { LED_GPSFAIL,   "GP3  falla GNSS" },
    { LED_TS_OK,     "GP2  envio ThingSpeak" },
  };

  for (uint8_t i = 0; i < 5; ++i) digitalWrite(leds[i].pin, LOW);

  for (uint8_t i = 0; i < 5; ++i) {
    Serial.print("  Encendiendo "); Serial.println(leds[i].name);
    for (uint8_t k = 0; k < 6; ++k) {
      digitalWrite(leds[i].pin, HIGH); delay(120);
      digitalWrite(leds[i].pin, LOW);  delay(120);
    }
  }
  Serial.println("  Confirmar a la vista que los cinco parpadearon en orden.");
  setResult(T_LED, V_PASS, "secuencia ejecutada, verificar a la vista");
}

// ============================================================================
// NIVEL 2 - IMU EN REGIMEN DE TRABAJO
// ============================================================================
void streamImu(uint32_t seconds) {
  banner("NIVEL 2 - IMU en regimen: mover el nodo para verificar signos");

  if (!icmReady && !mpuReady) {
    Serial.println("  Ejecutar antes los comandos 3 y 4.");
    setResult(T_IMU_STREAM, V_FAIL, "IMU no inicializadas");
    return;
  }

  Serial.println("  Sugerencia: inclinar hacia adelante, luego a la derecha,");
  Serial.println("  y girar sobre el eje vertical. Observar que cambian los ejes.");
  Serial.println("  ICM ax ay az [g] | gx gy gz [dps]  ||  MPU ax ay az [g]");

  uint32_t start = millis();
  uint32_t lastPrint = 0;
  uint32_t icmSamples = 0, mpuSamples = 0;
  float icmPeak = 0, mpuPeak = 0;

  while ((millis() - start) < seconds * 1000UL) {
    float ia[3] = {0, 0, 0}, ig[3] = {0, 0, 0};
    bool haveIcm = false;
    if (icmReady && myICM.dataReady()) {
      myICM.getAGMT();
      ia[0] = myICM.accX() / 1000.0f; ia[1] = myICM.accY() / 1000.0f; ia[2] = myICM.accZ() / 1000.0f;
      ig[0] = myICM.gyrX(); ig[1] = myICM.gyrY(); ig[2] = myICM.gyrZ();
      icmSamples++;
      haveIcm = true;
      float gm = sqrtf(ig[0] * ig[0] + ig[1] * ig[1] + ig[2] * ig[2]);
      if (gm > icmPeak) icmPeak = gm;
    }

    float mx = 0, my = 0, mz = 0;
    bool haveMpu = mpuReady && readMpuG(mx, my, mz);
    if (haveMpu) {
      mpuSamples++;
      float am = fabsf(sqrtf(mx * mx + my * my + mz * mz) - 1.0f);
      if (am > mpuPeak) mpuPeak = am;
    }

    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.print("  ");
      if (haveIcm) {
        Serial.print(ia[0], 2); Serial.print(' ');
        Serial.print(ia[1], 2); Serial.print(' ');
        Serial.print(ia[2], 2); Serial.print(" | ");
        Serial.print(ig[0], 1); Serial.print(' ');
        Serial.print(ig[1], 1); Serial.print(' ');
        Serial.print(ig[2], 1);
      } else Serial.print("ICM sin dato        ");
      Serial.print("  ||  ");
      if (haveMpu) {
        Serial.print(mx, 2); Serial.print(' ');
        Serial.print(my, 2); Serial.print(' ');
        Serial.print(mz, 2);
      } else Serial.print("MPU sin dato");
      Serial.println();
    }
    delay(5);
  }

  float icmHz = icmSamples / (float)seconds;
  float mpuHz = mpuSamples / (float)seconds;
  Serial.print("  ICM ~"); Serial.print(icmHz, 1);
  Serial.print(" Hz (R11 espera ~100), pico gyro "); Serial.print(icmPeak, 1); Serial.println(" dps");
  Serial.print("  MPU ~"); Serial.print(mpuHz, 1);
  Serial.print(" Hz (R11 muestrea a 50), pico |a|-1g "); Serial.print(mpuPeak, 3); Serial.println(" g");

  char detail[56];
  Verdict v = V_PASS;
  if (icmReady && icmHz < 40.0f) {
    v = V_WARN; snprintf(detail, sizeof(detail), "ICM lenta %.0f Hz", icmHz);
  } else if (mpuReady && mpuHz < 40.0f) {
    v = V_WARN; snprintf(detail, sizeof(detail), "MPU lenta %.0f Hz", mpuHz);
  } else if (icmPeak < 5.0f && mpuPeak < 0.05f) {
    v = V_WARN; snprintf(detail, sizeof(detail), "no se detecto movimiento");
  } else {
    snprintf(detail, sizeof(detail), "ICM %.0f Hz, MPU %.0f Hz, responde al mover", icmHz, mpuHz);
  }
  setResult(T_IMU_STREAM, v, detail);
}

// ============================================================================
// NIVEL 2 - SD AL RITMO REAL DE REGISTRO
// ============================================================================
void testSdRate(uint32_t seconds) {
  banner("NIVEL 2 - microSD escribiendo a 20 Hz como lo hace R11");

  if (!sdReady) {
    Serial.println("  Ejecutar antes el comando 5 (microSD).");
    setResult(T_SD_RATE, V_FAIL, "SD no montada");
    return;
  }

  const char *name = "DIAGRATE.CSV";
  if (SD.exists(name)) SD.remove(name);
  File f = SD.open(name, FILE_WRITE);
  if (!f) {
    setResult(T_SD_RATE, V_FAIL, "no se pudo crear archivo de prueba");
    return;
  }
  f.println("millis,ax_g,ay_g,az_g,mag_g");

  Serial.print("  Escribiendo "); Serial.print(seconds);
  Serial.println(" s a 20 Hz con flush cada segundo (mismo patron que VIB).");

  uint32_t start = millis();
  uint32_t lastWrite = 0, lastFlush = 0;
  uint32_t writes = 0;
  uint32_t worstWriteUs = 0, worstFlushUs = 0;
  bool writeError = false;

  while ((millis() - start) < seconds * 1000UL) {
    uint32_t now = millis();
    if (now - lastWrite >= 50) {
      lastWrite = now;
      float ax = 0, ay = 0, az = 0;
      if (mpuReady) readMpuG(ax, ay, az);
      float mag = sqrtf(ax * ax + ay * ay + az * az);

      uint32_t t0 = micros();
      f.clearWriteError();
      f.print(now); f.print(',');
      f.print(ax, 4); f.print(',');
      f.print(ay, 4); f.print(',');
      f.print(az, 4); f.print(',');
      f.println(mag, 4);
      uint32_t dt = micros() - t0;
      if (dt > worstWriteUs) worstWriteUs = dt;
      if (f.getWriteError()) writeError = true;
      writes++;
    }
    if (now - lastFlush >= 1000) {
      lastFlush = now;
      uint32_t t0 = micros();
      f.flush();
      uint32_t dt = micros() - t0;
      if (dt > worstFlushUs) worstFlushUs = dt;
    }
    delay(1);
  }

  f.flush();
  uint32_t finalSize = f.size();
  f.close();

  Serial.print("  lineas escritas: "); Serial.print(writes);
  Serial.print("  (esperadas ~"); Serial.print(seconds * 20); Serial.println(")");
  Serial.print("  tamano final: "); Serial.print(finalSize); Serial.println(" bytes");
  Serial.print("  peor escritura: "); Serial.print(worstWriteUs / 1000.0f, 1); Serial.println(" ms");
  Serial.print("  peor flush:     "); Serial.print(worstFlushUs / 1000.0f, 1); Serial.println(" ms");
  Serial.println("  Un flush por encima de ~100 ms le roba tiempo al lazo principal.");

  bool sizeOk = finalSize > 0;
  bool rateOk = writes >= seconds * 18;
  SD.remove(name);

  Verdict v;
  char detail[56];
  if (writeError || !sizeOk) {
    v = V_FAIL; snprintf(detail, sizeof(detail), "error de escritura");
  } else if (!rateOk) {
    v = V_WARN; snprintf(detail, sizeof(detail), "%lu lineas, por debajo de 20 Hz", (unsigned long)writes);
  } else if (worstFlushUs > 150000) {
    v = V_WARN; snprintf(detail, sizeof(detail), "flush lento %.0f ms", worstFlushUs / 1000.0f);
  } else {
    v = V_PASS; snprintf(detail, sizeof(detail), "%lu lineas, flush max %.0f ms",
                         (unsigned long)writes, worstFlushUs / 1000.0f);
  }
  setResult(T_SD_RATE, v, detail);
}

// ============================================================================
// NIVEL 2 - LTE + HTTP  (misma secuencia AT que la FSM de R11)
// ============================================================================
String buildPayload(bool useGnss, double lat, double lon, float kmh, float heading, float vib) {
  String status = "DIAG";
  status += useGnss ? "_GNSS" : "_SIMULADO";
  String post = "api_key=" + String(API_KEY) +
                "&field1=" + String(lat, 6) +
                "&field2=" + String(lon, 6) +
                "&field3=" + String(kmh, 1) +
                "&field4=" + String(heading, 1) +
                "&field5=" + String(useGnss ? 0.0f : -1.0f, 1) +
                "&field6=" + String(ID_TREN) +
                "&field7=" + String(ID_NODO) +
                "&field8=" + String(vib, 4) +
                "&status=" + status;
  return post;
}

// Devuelve el codigo HTTP, o un negativo que indica en que paso se detuvo.
int runHttpPost(const String &payload, const char *url) {
  Serial.print("  Destino: "); Serial.println(url);
  Serial.print("  Cuerpo ("); Serial.print(payload.length()); Serial.println(" bytes):");
  Serial.print("    "); Serial.println(payload);

  if (!modemSaysOk(modemCommand("AT", 3000))) return -1;
  modemCommand("ATE0", 2000);
  if (!modemSaysOk(modemCommand("AT+CFUN=1", 10000))) return -2;
  if (!modemSaysOk(modemCommand((String("AT+CGDCONT=1,\"IP\",\"") + APN + "\"").c_str(), 10000))) return -3;
  if (!modemSaysOk(modemCommand("AT+CGATT=1", 30000))) return -4;
  if (!modemSaysOk(modemCommand("AT+CGACT=1,1", 30000))) return -5;

  String netopen = modemCommand("AT+NETOPEN", 30000);
  bool netOk = netopen.indexOf("+NETOPEN: 0") >= 0 ||
               netopen.indexOf("already opened") >= 0 ||
               modemSaysOk(netopen);
  if (!netOk) {
    // Igual que R11: se continua y HTTPINIT decide si el contexto sirve.
    Serial.println("  NETOPEN no confirmo; se continua para que HTTPINIT decida.");
  }

  modemCommand("AT+HTTPTERM", 3000);  // ERROR aqui es normal si no habia sesion
  if (!modemSaysOk(modemCommand("AT+HTTPINIT", 10000))) {
    // Una sesion HTTP colgada del ciclo anterior deja HTTPINIT en ERROR.
    Serial.println("  HTTPINIT fallo; se cierra la sesion previa y se reintenta.");
    modemCommand("AT+HTTPTERM", 5000);
    delay(500);
    if (!modemSaysOk(modemCommand("AT+HTTPINIT", 10000))) return -6;
  }
  if (!modemSaysOk(modemCommand("AT+HTTPPARA=\"CID\",1", 10000))) return -7;
  if (!modemSaysOk(modemCommand((String("AT+HTTPPARA=\"URL\",\"") + url + "\"").c_str(), 10000))) return -8;
  if (!modemSaysOk(modemCommand("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"", 10000))) return -9;

  String dl = modemCommand((String("AT+HTTPDATA=") + payload.length() + ",10000").c_str(), 10000);
  if (dl.indexOf("DOWNLOAD") < 0) return -10;

  modemFlush();
  Serial.println("  MDM> [payload]");
  modem->print(payload);

  String bodyAck = "";
  uint32_t start = millis();
  while ((millis() - start) < 12000) {
    while (modem->available()) bodyAck += (char)modem->read();
    if (bodyAck.indexOf("OK") >= 0 || bodyAck.indexOf("ERROR") >= 0) break;
    delay(5);
  }
  if (bodyAck.indexOf("OK") < 0) return -11;
  Serial.println("  Cuerpo aceptado por el modem.");

  modemFlush();
  Serial.println("  MDM> AT+HTTPACTION=1");
  modem->println("AT+HTTPACTION=1");

  String action = "";
  int code = -12;
  start = millis();
  while ((millis() - start) < 40000) {
    while (modem->available()) action += (char)modem->read();
    int p = action.indexOf("+HTTPACTION:");
    if (p >= 0 && action.indexOf('\n', p) > p) {
      String lineStr = action.substring(p, action.indexOf('\n', p));
      int c1 = lineStr.indexOf(',');
      int c2 = lineStr.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        code = lineStr.substring(c1 + 1, c2).toInt();
        lineStr.trim();
        Serial.print("  MDM< "); Serial.println(lineStr);
      }
      break;
    }
    delay(10);
  }

  // Los encabezados explican un 3xx: dicen a donde quiere redirigir el servidor.
  if (code > 0) {
    Serial.println("  Encabezados devueltos por el servidor:");
    modemCommand("AT+HTTPHEAD", 10000);
    if (code >= 200 && code < 400) modemCommand("AT+HTTPREAD=0,64", 10000);
  }

  modemCommand("AT+HTTPTERM", 5000);
  return code;
}

const char *httpStageName(int code) {
  switch (code) {
    case -1:  return "el modem no responde a AT";
    case -2:  return "CFUN=1 rechazado";
    case -3:  return "CGDCONT (APN) rechazado";
    case -4:  return "CGATT: no adjunta a datos";
    case -5:  return "CGACT: no activa el contexto PDP";
    case -6:  return "HTTPINIT fallo";
    case -7:  return "HTTPPARA CID fallo";
    case -8:  return "HTTPPARA URL fallo";
    case -9:  return "HTTPPARA CONTENT fallo";
    case -10: return "HTTPDATA no entrego DOWNLOAD";
    case -11: return "el modem no acepto el cuerpo";
    case -12: return "sin respuesta a HTTPACTION";
  }
  return "codigo HTTP";
}

void testHttp(bool useRealGnss, const char *url) {
  banner("NIVEL 2 - Sesion de datos LTE y POST a ThingSpeak");

  double lat = 19.504700, lon = -99.146900;   // Coordenada de prueba
  float kmh = 0.0f, heading = 0.0f, vib = 0.0f;

  if (useRealGnss && gnssReady && myGPS.getPVT(1000) && myGPS.getFixType() >= 2) {
    lat = myGPS.getLatitude() / 1e7;
    lon = myGPS.getLongitude() / 1e7;
    kmh = myGPS.getGroundSpeed() / 1000.0f * 3.6f;
    heading = myGPS.getHeading() / 100000.0f;
    Serial.println("  Usando la posicion GNSS actual.");
  } else if (useRealGnss) {
    Serial.println("  Sin fijado GNSS; se envia una coordenada de prueba marcada como SIMULADO.");
    useRealGnss = false;
  }

  if (mpuReady) {
    float ax, ay, az;
    if (readMpuG(ax, ay, az)) vib = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1.0f);
  }

  String payload = buildPayload(useRealGnss, lat, lon, kmh, heading, vib);
  modemBegin();
  delay(100);

  uint32_t t0 = millis();
  int code = runHttpPost(payload, url);
  uint32_t elapsedMs = millis() - t0;

  Serial.print("  Duracion del ciclo: "); Serial.print(elapsedMs / 1000.0f, 1); Serial.println(" s");

  char detail[56];
  Verdict v;
  if (code >= 200 && code < 300) {
    v = V_PASS;
    snprintf(detail, sizeof(detail), "HTTP %d en %.1f s", code, elapsedMs / 1000.0f);
    Serial.println("  ThingSpeak acepto el dato. Verificar el canal en la plataforma.");
    digitalWrite(LED_TS_OK, HIGH); delay(400); digitalWrite(LED_TS_OK, LOW);
  } else if (code >= 300 && code < 400) {
    v = V_FAIL;
    snprintf(detail, sizeof(detail), "HTTP %d redireccion, R11 lo trata como fallo", code);
    Serial.println("  El servidor redirige. R11 solo acepta 2xx, asi que reintentaria sin avanzar.");
    Serial.println("  Ver el encabezado Location de arriba para saber a donde apunta.");
  } else if (code > 0) {
    v = V_FAIL;
    snprintf(detail, sizeof(detail), "HTTP %d rechazado", code);
    Serial.println("  El servidor respondio con error. Revisar API key y formato del cuerpo.");
  } else {
    v = V_FAIL;
    snprintf(detail, sizeof(detail), "%s", httpStageName(code));
    Serial.print("  La cadena se corto en: "); Serial.println(httpStageName(code));
  }
  setResult(T_HTTP, v, detail);
}

// Puente transparente USB <-> SIM7600 para escribir comandos AT a mano.
void modemBridge() {
  banner("Puente AT directo con el SIM7600  (escribir  +++  para salir)");
  modemBegin();
  String userLine = "";
  while (true) {
    while (modem->available()) Serial.write(modem->read());
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        userLine.trim();
        if (userLine == "+++") { Serial.println("Saliendo del puente."); return; }
        modem->println(userLine);
        userLine = "";
      } else {
        userLine += c;
        if (userLine.length() > 200) userLine = "";
      }
    }
    delay(2);
  }
}

// ============================================================================
// NIVEL 3 - CADENA COMPLETA
// ============================================================================
void testFullChain() {
  banner("NIVEL 3 - Cadena completa: GNSS -> posicion -> SD -> LTE -> ThingSpeak");
  Serial.println("  Cada eslabon se evalua con el mismo criterio que aplica R11.");
  Serial.println("  Para un resultado representativo, el nodo debe estar a cielo abierto.");

  bool okGnss = false, okPos = false, okSd = false, okModem = false, okHttp = false;
  double lat = 0, lon = 0;
  float kmh = 0, heading = 0, vib = 0;
  int httpCode = 0;

  // --- Eslabon 1: GNSS ---
  Serial.println();
  Serial.println("  [1/5] GNSS valido segun criterio R11 (hasta 60 s)");
  if (!gnssReady) {
    gnssReady = myGPS.begin(Wire);
    if (gnssReady) {
      myGPS.setI2COutput(COM_TYPE_UBX);
      myGPS.setNavigationFrequency(5);
      myGPS.setAutoPVT(true);
      delay(400);
    }
  }
  if (gnssReady) {
    uint32_t start = millis();
    while ((millis() - start) < 60000 && !okGnss) {
      if (myGPS.getPVT(100)) {
        uint8_t fix = myGPS.getFixType();
        uint8_t sats = myGPS.getSIV();
        uint32_t hAcc = myGPS.getHorizontalAccEst();
        uint16_t pdop = myGPS.getPDOP();
        if (fix >= 3 && sats >= MIN_GNSS_SATS && hAcc > 0 && hAcc <= HACC_LIMIT_MM &&
            pdop > 0 && pdop <= PDOP_LIMIT_CENTI) {
          okGnss = true;
          lat = myGPS.getLatitude() / 1e7;
          lon = myGPS.getLongitude() / 1e7;
          kmh = myGPS.getGroundSpeed() / 1000.0f * 3.6f;
          heading = myGPS.getHeading() / 100000.0f;
          Serial.print("        fijado en "); Serial.print((millis() - start) / 1000.0f, 1);
          Serial.print(" s  sats="); Serial.print(sats);
          Serial.print("  hAcc="); Serial.print(hAcc / 1000.0f, 2); Serial.println(" m");
        }
      }
      delay(20);
    }
    if (!okGnss) Serial.println("        sin solucion utilizable en 60 s");
  } else Serial.println("        el receptor no responde");

  // --- Eslabon 2: posicion disponible para transmitir ---
  Serial.println("  [2/5] Posicion disponible para el paquete");
  okPos = okGnss && (fabs(lat) > 0.000001 || fabs(lon) > 0.000001) &&
          lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
  if (okPos) {
    Serial.print("        lat="); Serial.print(lat, 7);
    Serial.print("  lon="); Serial.println(lon, 7);
  } else {
    Serial.println("        sin coordenada valida: R11 quedaria en SRC_NONE y no transmitiria");
  }

  // --- Eslabon 3: registro local ---
  Serial.println("  [3/5] Registro en microSD");
  if (!sdReady) sdReady = SD.begin(SD_CS_PIN);
  if (sdReady) {
    if (mpuReady) {
      float ax, ay, az;
      if (readMpuG(ax, ay, az)) vib = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1.0f);
    }
    File f = SD.open("DIAGCHN.CSV", FILE_WRITE);
    if (f) {
      f.clearWriteError();
      f.print(millis()); f.print(',');
      f.print(lat, 7); f.print(',');
      f.print(lon, 7); f.print(',');
      f.print(kmh, 2); f.print(',');
      f.print(heading, 2); f.print(',');
      f.println(vib, 4);
      f.flush();
      okSd = (f.getWriteError() == 0);
      f.close();
    }
    Serial.println(okSd ? "        linea escrita y confirmada en DIAGCHN.CSV"
                        : "        la escritura fallo");
    if (!mpuReady) Serial.println("        nota: sin MPU6500, R11 no escribe ningun VIB_YYYYMMDD.CSV");
  } else Serial.println("        tarjeta no montada");

  // --- Eslabon 4: modem listo ---
  Serial.println("  [4/5] SIM7600 registrado en la red");
  modemBegin();
  delay(100);
  if (modemSaysOk(modemCommand("AT", 3000, false))) {
    modemCommand("ATE0", 2000, false);
    String cpin  = modemCommand("AT+CPIN?", 5000, false);
    String creg  = modemCommand("AT+CREG?", 3000, false);
    String cgreg = modemCommand("AT+CGREG?", 3000, false);
    String csq   = modemCommand("AT+CSQ", 3000, false);
    int rssi = parseCsq(csq);
    bool reg = (creg.indexOf(",1") >= 0 || creg.indexOf(",5") >= 0 ||
                cgreg.indexOf(",1") >= 0 || cgreg.indexOf(",5") >= 0);
    okModem = (cpin.indexOf("READY") >= 0) && reg;
    Serial.print("        SIM="); Serial.print(cpin.indexOf("READY") >= 0 ? "lista" : "no lista");
    Serial.print("  registro="); Serial.print(reg ? "si" : "no");
    Serial.print("  CSQ="); Serial.println(rssi);
  } else Serial.println("        el modem no responde a AT");

  // --- Eslabon 5: HTTP a ThingSpeak ---
  Serial.println("  [5/5] POST a ThingSpeak");
  if (okPos && okModem) {
    String payload = buildPayload(true, lat, lon, kmh, heading, vib);
    httpCode = runHttpPost(payload, SERVER_URL);  // la misma URL que usa R11
    okHttp = (httpCode >= 200 && httpCode < 300);
    if (okHttp) { digitalWrite(LED_TS_OK, HIGH); delay(400); digitalWrite(LED_TS_OK, LOW); }
  } else {
    Serial.println("        no se ejecuta: falta posicion valida o red");
  }

  // --- Veredicto de la cadena ---
  Serial.println();
  Serial.println("  RESUMEN DE LA CADENA");
  Serial.print("    GNSS valido       "); Serial.println(okGnss ? "OK" : "NO");
  Serial.print("    posicion lista    "); Serial.println(okPos ? "OK" : "NO");
  Serial.print("    microSD registra  "); Serial.println(okSd ? "OK" : "NO");
  Serial.print("    LTE registrado    "); Serial.println(okModem ? "OK" : "NO");
  Serial.print("    HTTP ThingSpeak   ");
  if (okHttp) { Serial.print("OK ("); Serial.print(httpCode); Serial.println(")"); }
  else if (httpCode > 0) { Serial.print("NO (HTTP "); Serial.print(httpCode); Serial.println(")"); }
  else if (httpCode < 0) { Serial.print("NO ("); Serial.print(httpStageName(httpCode)); Serial.println(")"); }
  else Serial.println("NO ejecutado");

  const char *firstBreak = nullptr;
  if (!okGnss) firstBreak = "se corta en GNSS";
  else if (!okPos) firstBreak = "se corta en la posicion";
  else if (!okSd) firstBreak = "se corta en microSD";
  else if (!okModem) firstBreak = "se corta en el registro LTE";
  else if (!okHttp) firstBreak = "se corta en HTTP";

  char detail[56];
  Verdict v;
  if (okGnss && okPos && okSd && okModem && okHttp) {
    v = V_PASS;
    snprintf(detail, sizeof(detail), "cadena completa, HTTP %d", httpCode);
    Serial.println("    -> Los dos caminos de evidencia estan disponibles.");
  } else if (okSd && !okHttp) {
    v = V_WARN;
    snprintf(detail, sizeof(detail), "solo evidencia local: %s", firstBreak);
    Serial.println("    -> Queda evidencia en microSD, pero no llega a la plataforma.");
  } else {
    v = V_FAIL;
    snprintf(detail, sizeof(detail), "%s", firstBreak ? firstBreak : "cadena incompleta");
  }
  setResult(T_CHAIN, v, detail);
}

// ============================================================================
// INFORME, ENERGIA Y MENU
// ============================================================================
void printPower() {
  banner("Alimentacion y recursos del RP2040");
  analogReadResolution(12);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; ++i) { sum += analogRead(VSYS_ADC_PIN); delay(2); }
  float counts = sum / 16.0f;
  float vsys = counts * 3.3f / 4095.0f * 3.0f;
  Serial.print("  VSYS aproximado: "); Serial.print(vsys, 2); Serial.println(" V");
  Serial.println("  Por USB deben verse ~4.6 a 5.1 V. Por debajo de 4.4 V el SIM7600");
  Serial.println("  puede reiniciarse justo al transmitir, y arrastrar a la SD con el.");
  Serial.print("  Memoria libre: "); Serial.print(rp2040.getFreeHeap()); Serial.println(" bytes");
  Serial.print("  Tiempo encendido: "); Serial.print(millis() / 1000); Serial.println(" s");
}

void printReport() {
  banner("INFORME DE DIAGNOSTICO");
  Serial.println("  Prueba                      Veredicto  Detalle");
  Serial.println("  ------------------------------------------------------------");
  uint8_t pass = 0, warn = 0, fail = 0, notrun = 0;
  for (uint8_t i = 0; i < T_COUNT; ++i) {
    Serial.print("  ");
    const char *name = testName(i);
    Serial.print(name);
    for (int k = strlen(name); k < 28; ++k) Serial.print(' ');
    const char *tag = verdictTag(board[i].verdict);
    Serial.print(tag);
    for (int k = strlen(tag); k < 9; ++k) Serial.print(' ');
    Serial.print(' ');
    Serial.println(board[i].detail);
    switch (board[i].verdict) {
      case V_PASS: pass++; break;
      case V_WARN: warn++; break;
      case V_FAIL: fail++; break;
      default: notrun++; break;
    }
  }
  Serial.println("  ------------------------------------------------------------");
  Serial.print("  OK="); Serial.print(pass);
  Serial.print("  ADVERTENCIAS="); Serial.print(warn);
  Serial.print("  FALLAS="); Serial.print(fail);
  Serial.print("  SIN EJECUTAR="); Serial.println(notrun);
  Serial.println("  Copiar este bloque tal cual como evidencia de la prueba.");
}

void printHelp() {
  Serial.println();
  Serial.println("============================================================");
  Serial.print("  "); Serial.println(DIAG_VERSION);
  Serial.println("  Banco de pruebas del nodo. Escribir la letra y ENTER.");
  Serial.println("============================================================");
  Serial.println("  NIVEL 1 - COMPONENTE");
  Serial.println("    1  explorar el bus I2C");
  Serial.println("    2  OLED SSD1306");
  Serial.println("    3  ICM-20948   (dejar el nodo quieto)");
  Serial.println("    4  MPU6500     (dejar el nodo quieto)");
  Serial.println("    5  microSD     (monta, E/S y listado)");
  Serial.println("    6  ZED-F9P     (30 s de observacion)");
  Serial.println("    7  SIM7600G    (AT, SIM, senal, registro)");
  Serial.println("    8  LED indicadores");
  Serial.println("    a  ejecutar todo el nivel 1 en orden");
  Serial.println();
  Serial.println("  NIVEL 2 - SUBSISTEMA");
  Serial.println("    s  IMU en regimen, 15 s   (mover el nodo)");
  Serial.println("    w  microSD a 20 Hz, 15 s");
  Serial.println("    d  sesion LTE + POST a ThingSpeak por HTTP   (como R11)");
  Serial.println("    e  el mismo POST pero por HTTPS   (contraste con el 302)");
  Serial.println();
  Serial.println("  NIVEL 3 - NODO COMPLETO");
  Serial.println("    n  cadena GNSS -> SD -> LTE -> ThingSpeak");
  Serial.println();
  Serial.println("  APOYO");
  Serial.println("    f  ver el VIB_YYYYMMDD.CSV mas reciente de la tarjeta");
  Serial.println("    c  catalogo de todos los VIB: muestras y firmware que los escribio");
  Serial.println("    t  volcar un archivo por nombre, ej:  t VIB_20260822.CSV");
  Serial.println("    g  observar GNSS 120 s (busqueda de cielo)");
  Serial.println("    v  alimentacion VSYS y memoria libre");
  Serial.println("    p  puente AT directo con el modem (salir con +++)");
  Serial.println("    r  informe consolidado");
  Serial.println("    h  este menu");
  Serial.println("============================================================");
}

void runLevel1() {
  banner("NIVEL 1 COMPLETO - todos los componentes en orden");
  testI2cScan();
  testOled();
  testIcm20948();
  testMpu6500();
  testSdCard();
  testGnss(30);
  testModem(true);
  testLeds();
  printReport();
}

// ============================================================================
// SETUP Y LAZO DE COMANDOS
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_ONBOARD, OUTPUT);
  pinMode(LED_HEART_EXT, OUTPUT);
  pinMode(LED_SD, OUTPUT);
  pinMode(LED_GPSFAIL, OUTPUT);
  pinMode(LED_TS_OK, OUTPUT);
  digitalWrite(LED_ONBOARD, LOW);
  digitalWrite(LED_HEART_EXT, LOW);
  digitalWrite(LED_SD, LOW);
  digitalWrite(LED_GPSFAIL, LOW);
  digitalWrite(LED_TS_OK, LOW);

  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeout(50);

  for (uint8_t i = 0; i < T_COUNT; ++i) {
    board[i].verdict = V_NOTRUN;
    board[i].detail[0] = '\0';
    board[i].whenMs = 0;
  }

  uint32_t start = millis();
  while (!Serial && (millis() - start) < 4000) delay(50);
  delay(300);

  Serial.println();
  Serial.print("Arranque de "); Serial.println(DIAG_VERSION);
  Serial.println("Este firmware NO opera el nodo: solo lo mide.");
  Serial.println("Para volver a la operacion normal, cargar ProgramaFinal_R11_General_ENU.ino");
  printHelp();
  Serial.print("> ");
}

void handleCommand(const String &cmdIn) {
  String cmd = cmdIn;
  cmd.trim();
  if (cmd.length() == 0) { Serial.print("> "); return; }

  // El argumento conserva mayusculas: los nombres FAT de la tarjeta las usan.
  String argument = "";
  int space = cmd.indexOf(' ');
  if (space > 0) {
    argument = cmd.substring(space + 1);
    argument.trim();
  }

  char selector = (char)tolower(cmd.charAt(0));
  switch (selector) {
    case '1': testI2cScan(); break;
    case '2': testOled(); break;
    case '3': testIcm20948(); break;
    case '4': testMpu6500(); break;
    case '5': testSdCard(); break;
    case '6': testGnss(30); break;
    case '7': testModem(true); break;
    case '8': testLeds(); break;
    case 'a': runLevel1(); break;
    case 's': streamImu(15); break;
    case 'w': testSdRate(15); break;
    case 'd': testHttp(true, SERVER_URL); break;
    case 'e': testHttp(true, SERVER_URL_HTTPS); break;
    case 'n': testFullChain(); break;
    case 'f': dumpNewestVib(10); break;
    case 'c': catalogVibFiles(); break;
    case 't': {
      if (argument.length() == 0) { Serial.println("Uso:  t VIB_20260822.CSV [lineas]"); break; }
      // Segundo argumento opcional: cuantas lineas finales mostrar.
      int sp2 = argument.indexOf(' ');
      int lineas = 10;
      String nombre = argument;
      if (sp2 > 0) {
        nombre = argument.substring(0, sp2);
        lineas = argument.substring(sp2 + 1).toInt();
        if (lineas <= 0) lineas = 10;
      }
      dumpFileByName(nombre, lineas);
      break;
    }
    case 'g': testGnss(120); break;
    case 'v': printPower(); break;
    case 'p': modemBridge(); break;
    case 'r': printReport(); break;
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

  // Latido lento: confirma que el firmware de diagnostico sigue vivo.
  if (millis() - lastBlink >= 1000) {
    lastBlink = millis();
    digitalWrite(LED_ONBOARD, !digitalRead(LED_ONBOARD));
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
