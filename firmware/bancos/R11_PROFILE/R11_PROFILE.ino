// ============================================================================
//  PROGRAMA FINAL R11 - NAVEGACION GENERAL ENU + AUTOCALIBRACION IMU
//  Raspberry Pi Pico (RP2040) + ZED-F9P + ICM-20948 + MPU6500 + microSD
//  + OLED SSD1306 + SIM7600G-H
//
//  OBJETIVO DE ESTA VERSION:
//    - Funcionar en cualquier ubicacion geografica, sin depender de una ruta,
//      estaciones o polilinea precargada.
//    - Determinar automaticamente el piso mediante el vector de gravedad.
//    - Aprender el eje longitudinal de cada tarjeta mediante aceleraciones y
//      frenados observados simultaneamente por la IMU y el GNSS.
//    - Mantener una referencia respecto al norte verdadero usando el rumbo de
//      movimiento del ZED-F9P y el giroscopio de la ICM-20948.
//    - Realizar fusion bidimensional Este-Norte. GNSS corrige posicion y
//      velocidad; la ICM mantiene una estimacion breve cuando el GNSS se pierde.
//
//  CONVENCIONES:
//    - Marco local ENU: X=Este, Y=Norte, Z=Arriba.
//    - Marco del nodo: X=sentido de la primera marcha usada para calibrar,
//      Y=derecha respecto a X y Z=arriba.
//    - El norte absoluto solo puede adquirirse con movimiento GNSS valido.
//      En reposo se conserva la ultima referencia aprendida y se propaga con el
//      giroscopio, por lo que debe volver a corregirse al reanudar la marcha.
//
//  LIMITACION IMPORTANTE:
//    - El dead reckoning supone que el nodo permanece rigidamente instalado.
//      Si se gira o se cambia de vehiculo durante una perdida de GNSS, la
//      estimacion inercial puede perder validez hasta recibir una nueva
//      correccion de rumbo GNSS.
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

// ============================================================================
// IDENTIFICACION Y VERSION
// ============================================================================
static const char FW_VERSION[] = "TT-NODO-R11-GENERAL";
static const int ID_TREN = 0;
static const int ID_NODO = 0;

// ============================================================================
// PINES
// ============================================================================
#define LED_ONBOARD    25
#define LED_HEART_EXT  15  // antes GP2
#define LED_SD         14  // antes GP3
#define LED_GPSFAIL     3  // antes GP14
#define LED_TS_OK       2  // antes GP15

#define SD_CS_PIN      17

#define MODEM_TX        0   // Pico GP0 TX -> RX del SIM7600
#define MODEM_RX        1   // Pico GP1 RX <- TX del SIM7600
#define PWRKEY           6

#define I2C_SDA_PIN      4
#define I2C_SCL_PIN      5

HardwareSerial *modem = &Serial1;

// ============================================================================
// THINGSPEAK / RED CELULAR
// ============================================================================
static const char APN[]        = "internet.itelcel.com";
static const char API_KEY[]    = "AQUI_TU_CLAVE_DE_ESCRITURA"   // sustituida al publicar; ver README;
static const char SERVER_URL[] = "http://api.thingspeak.com/update";

// Mapa del canal ThingSpeak:
// field1=latitud, field2=longitud, field3=velocidad_kmh,
// field4=rumbo verdadero en grados (0=N, 90=E, 180=S, 270=O),
// field5=segundos_sin_GNSS, field6=id_tren, field7=id_nodo,
// field8=vibracion_lineal_mas_reciente_g.
// El campo status incluye fuente, salud IMU, referencia norte y estado RTK.
//
// Para conservar temporalmente una plataforma antigua que espera 0/1 en
// field4, cambie FIELD4_MODE a FIELD4_LEGACY_NS mas abajo.
//
// Se envia una muestra resumida cada 15 s. El temporizador se reinicia
// al confirmarse el HTTP 2xx, para mantener al menos 15 s entre exitos.
static const uint32_t SEND_PERIOD_MS = 15000;
static const uint32_t RETRY_AFTER_MS = 2000;

// ============================================================================
// OBJETOS
// ============================================================================
SFE_UBLOX_GNSS myGPS;
ICM_20948_I2C  myICM;
#define ICM_ADDR ICM_20948_I2C_ADDR_AD1  // 0x69

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================================
// MPU6500
// ============================================================================
static const uint8_t MPU_ADDR            = 0x68;
static const uint8_t MPU_RA_WHO_AM_I     = 0x75;
static const uint8_t MPU_RA_PWR_MGMT_1   = 0x6B;
static const uint8_t MPU_RA_SMPLRT_DIV   = 0x19;
static const uint8_t MPU_RA_CONFIG       = 0x1A;
static const uint8_t MPU_RA_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_RA_ACCEL_XOUT_H = 0x3B;
static const float   MPU_LSB_PER_G        = 8192.0f; // +/-4 g

// ============================================================================
// ORIENTACION GENERAL DE SENSORES
// ============================================================================
// Orientacion provisional de respaldo. Solo se usa antes de completar la
// calibracion automatica. 0=X del sensor, 1=Y, 2=Z.
static const uint8_t ICM_BODY_X_SENSOR_AXIS = 0;
static const uint8_t ICM_BODY_Y_SENSOR_AXIS = 1;
static const uint8_t ICM_BODY_Z_SENSOR_AXIS = 2;
static const float ICM_BODY_X_SIGN = 1.0f;
static const float ICM_BODY_Y_SIGN = 1.0f;
static const float ICM_BODY_Z_SIGN = 1.0f;

static const uint8_t MPU_BODY_X_SENSOR_AXIS = 0;
static const uint8_t MPU_BODY_Y_SENSOR_AXIS = 1;
static const uint8_t MPU_BODY_Z_SENSOR_AXIS = 2;
static const float MPU_BODY_X_SIGN = 1.0f;
static const float MPU_BODY_Y_SIGN = 1.0f;
static const float MPU_BODY_Z_SIGN = 1.0f;

// Signo para convertir el giro positivo alrededor de Z del marco
// X-adelante, Y-derecha, Z-arriba a un rumbo geografico que aumenta en sentido
// horario. Verificar durante la primera prueba: al girar a la derecha el rumbo
// debe aumentar. Si ocurre lo contrario, cambie a +1.0f.
static const float GYRO_TO_HEADING_SIGN = -1.0f;

struct Vec3f {
  float x;
  float y;
  float z;
};

struct SensorAlignment {
  Vec3f upSensor;          // vertical positiva expresada en ejes del sensor
  Vec3f forwardSensor;     // eje X fijo del nodo, aprendido con GNSS
  Vec3f rightSensor;       // lateral derecha respecto a forwardSensor
  Vec3f headingCorrelation;
  bool floorValid;
  bool headingValid;
  uint16_t headingSamples;
  float headingExcitation;
  float headingWeight;
  float headingQuality;
};

SensorAlignment icmAlignment = {};
SensorAlignment mpuAlignment = {};

static const char IMU_CAL_FILE[] = "IMUCAL.BIN";
static const char IMU_CAL_TEMP_FILE[] = "IMUCAL.TMP";
static const uint32_t IMU_CAL_MAGIC = 0x5443414CUL; // "TCAL"
static const uint16_t IMU_CAL_VERSION = 2;

struct ImuCalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t sequence;
  uint8_t icmFloorValid;
  uint8_t icmHeadingValid;
  uint8_t mpuFloorValid;
  uint8_t mpuHeadingValid;
  uint8_t northReferenceValid;
  int8_t motionDirectionSign;
  uint16_t reserved;
  float icmUp[3];
  float icmForward[3];
  float mpuUp[3];
  float mpuForward[3];
  float icmHeadingQuality;
  float mpuHeadingQuality;
  float bodyHeadingTrue_deg;
  uint32_t crc;
};

uint32_t imuCalibrationSequence = 0;
bool imuCalibrationLoaded = false;
bool imuCalibrationDirty = false;
uint32_t lastImuCalibrationSaveMs = 0;

// field4 puede enviar el rumbo general o el valor legado norte/sur.
enum Field4OutputMode : uint8_t {
  FIELD4_LEGACY_NS = 0,
  FIELD4_HEADING_DEG = 1
};
static const Field4OutputMode FIELD4_MODE = FIELD4_HEADING_DEG;

// ============================================================================
// INTERVALOS
// ============================================================================
static const uint32_t I2C_CLOCK_HZ         = 400000;
static const uint32_t OLED_INTERVAL_MS     = 250;
static const uint32_t MPU_SAMPLE_MS        = 20;   // lectura efectiva 50 Hz
static const uint32_t VIB_LOG_INTERVAL_MS  = 50;   // almacenamiento 20 Hz
static const uint32_t FILE_FLUSH_MS         = 1000;
static const uint32_t LASTPOS_SAVE_INTERVAL_MS = 15000;
// SDFS.info() recorre la FAT: ~6.15 s en la tarjeta de 15 GB del prototipo.
// Es la unica forma de conocer la ocupacion real, asi que se conserva, pero
// se espacia para que ese bloqueo no afecte el registro a 20 Hz. A 20 Hz el
// nodo escribe ~18 MB por hora, de modo que una revision cada 30 min sigue
// siendo muy holgada frente al umbral de limpieza del 80 %.
static const uint32_t SD_USAGE_CHECK_MS        = 1800000;
static const uint32_t SD_HEALTH_CHECK_MS       = 5000;
static const uint32_t SD_RETRY_INTERVAL_MS     = 5000;
static const uint32_t SD_FAULT_LED_BLINK_MS    = 250;
static const uint8_t  SD_MAX_CONSECUTIVE_ERRORS = 3;
static const uint8_t  SD_CLEAN_TRIGGER_PERCENT = 80;
static const uint8_t  SD_CLEAN_TARGET_PERCENT  = 75;

static const uint32_t GPS_RECENT_MS          = 2000;
static const uint32_t LAST_VALID_MAX_AGE_MS  = 120000; // validar experimentalmente
static const uint32_t DIRECTION_REF_MS       = 2000;

// ============================================================================
// CRITERIOS GNSS GENERALES
// ============================================================================
static const uint8_t  MIN_GNSS_SATS         = 6;
static const uint32_t HACC_LIMIT_MM         = 10000; // 10 m
static const uint32_t SACC_LIMIT_MMPS       = 1500;  // 1.5 m/s
static const uint16_t PDOP_LIMIT_CENTI      = 600;   // 6.00
static const float    HEADING_OUTPUT_MIN_SPEED_MPS = 0.50f;
static const float    LEGACY_NS_MIN_VN_MPS = 0.30f;
static const float    LOCAL_REBASE_DISTANCE_M = 50000.0f;
static const bool     REQUIRE_RTK_FOR_POSITION = false;

// ============================================================================
// CRITERIOS IMU, FUSION Y SUPERVISION
// ============================================================================
static const float GRAVITY_MPS2               = 9.80665f;
static const float ACC_DEADBAND_MPS2           = 0.035f;
static const float NAV_ACCEL_LPF_HZ            = 3.0f;
static const float ATTITUDE_TAU_S              = 0.50f;
static const float GPS_STILL_MPS               = 0.20f;
static const float STILL_MAG_MIN_G             = 0.92f;
static const float STILL_MAG_MAX_G             = 1.08f;
// Banda de reposo alrededor de la gravedad que mide cada sensor.
// Un acelerometro con sesgo de fabrica puede leer 1.11 g en reposo y aun asi
// estar perfectamente quieto. Comparar contra 1.000 g absoluto descartaba esas
// unidades y dejaba el ZUPT permanentemente inhabilitado. La referencia se
// aprende en el arranque, cuando el nodo esta detenido por procedimiento.
static const float STILL_MAG_TOL_G             = 0.08f;
static const float STILL_VIB_MAX_G             = 0.035f;
static const float STILL_GYRO_MAX_DPS          = 1.5f;
static const uint32_t STILL_HOLD_MS             = 3000;
static const float BIAS_ADAPT_ALPHA             = 0.001f;
static const float IMU_SPEED_INNOV_LIMIT_MPS    = 1.50f;
static const float IMU_STILL_ACCEL_LIMIT_MPS2   = 0.30f;
static const uint32_t IMU_ERROR_HOLD_MS          = 3000;
static const float FUSION_MAX_SPEED_MPS          = 35.0f;
static const float FUSION_ACCEL_NOISE_MPS2       = 0.35f;
static const float FUSION_BIAS_RW_MPS2           = 0.003f;

// Filtro de gravedad del MPU6500 para vibracion.
static const float GRAV_CUTOFF_HZ = 0.35f;

// Calibracion automatica de orientacion.
static const uint16_t ICM_FLOOR_CAL_SAMPLES = 300;
static const uint16_t MPU_FLOOR_CAL_SAMPLES = 150;
static const float FLOOR_CAL_MIN_MAG_G = 0.85f;
static const float FLOOR_CAL_MAX_MAG_G = 1.15f;
static const float HEADING_CAL_MIN_GPS_ACCEL_MPS2 = 0.12f;
static const float HEADING_CAL_MAX_GPS_ACCEL_MPS2 = 2.50f;
static const float HEADING_CAL_MAX_SACC_MPS = 0.50f;
static const float HEADING_CAL_MIN_HORIZONTAL_G = 0.004f;
static const uint16_t HEADING_CAL_MIN_SAMPLES = 18;
static const float HEADING_CAL_MIN_EXCITATION_MPS = 1.20f;
static const float HEADING_CAL_MIN_QUALITY = 0.62f;
static const float HEADING_ACCEL_LPF_HZ = 1.0f;
static const float HEADING_CAL_MIN_COURSE_SPEED_MPS = 1.0f;
static const float HEADING_CAL_MAX_COURSE_ERROR_DEG = 35.0f;
static const float HEADING_CAL_MAX_HEADACC_DEG = 20.0f;
static const float NORTH_CORRECTION_GAIN = 0.18f;
static const float NORTH_REACQUIRE_GAIN = 0.45f;
static const uint32_t MOTION_SIGN_VALID_MS = 8000;
static const uint32_t IMU_CAL_SAVE_MIN_MS = 30000;

// ============================================================================
// ODOMETRIA DE RUEDA OPCIONAL
// ============================================================================
#define ENABLE_WHEEL_ODOMETRY 0
#define WHEEL_PULSE_PIN 7
static const float WHEEL_METERS_PER_PULSE = 0.050f; // CALIBRAR antes de habilitar
static const uint32_t WHEEL_UPDATE_MS = 200;
static const float WHEEL_SPEED_SIGMA_MPS = 0.15f;

// ============================================================================
// ESTADO GENERAL DE HARDWARE
// ============================================================================
bool oledOK = false;
bool sdOK   = false;
bool gpsPresent = false;
bool icmPresent = false;
bool mpuPresent = false;

// ============================================================================
// TIEMPO, GNSS Y POSICION GENERAL ENU
// ============================================================================
bool gpsTimeValid = false;
uint16_t gpsYear = 0;
uint8_t gpsMonth = 0, gpsDay = 0;
uint8_t gpsHour = 0, gpsMinute = 0, gpsSecond = 0;

double gpsLat = 0.0, gpsLon = 0.0; // posicion cruda UBX-NAV-PVT
float gpsGroundSpeed_mps = 0.0f;
float gpsSpeed_mps = 0.0f;
float gpsVelE_mps = 0.0f, gpsVelN_mps = 0.0f;
float gpsHeading_deg = 0.0f;
float gpsHeadingAcc_deg = 0.0f;
int gpsSats = 0;
uint8_t gpsFixType = 0;
uint8_t gpsCarrierSolution = 0; // 0=sin RTK, 1=float, 2=fixed
uint32_t gpsHAcc_mm = 0;
uint32_t gpsSAcc_mmps = 0;
uint16_t gpsPDOP_centi = 9999;
bool gpsGnssFixOk = false;
bool gpsInvalidLlh = true;
bool gpsFixRaw = false;
bool gpsQualityCurrent = false;
bool gpsUsable = false;
bool previousGpsUsable = false;
uint32_t lastPvtMs = 0;
uint32_t lastGpsValidMs = 0;

// Origen local ENU. Se fija con la primera posicion valida o con LASTPOS.TXT.
bool localOriginValid = false;
double localOriginLat = 0.0, localOriginLon = 0.0;
float gpsLocalE_m = 0.0f, gpsLocalN_m = 0.0f;

bool hasPositionReference = false;
double refLat = 0.0, refLon = 0.0;
bool referenceRecoveredFromSd = false;
uint32_t recoveredReferenceMs = 0;

// Compatibilidad con la plataforma anterior:
// 0 = componente de movimiento hacia el norte; 1 = hacia el sur.
// No representa una direccion completa cuando el movimiento es este-oeste.
int dirNS = 0;
bool directionKnown = false;

// Referencia de norte del marco fijo del nodo.
bool northReferenceValid = false;
float bodyHeadingTrue_deg = 0.0f;  // rumbo del +X del nodo, horario desde norte
float northReferenceQuality = 0.0f;
int8_t motionDirectionSign = 1;    // +1: movimiento sobre +X; -1: sobre -X
uint32_t lastMotionSignMs = 0;
uint32_t lastNorthCorrectionMs = 0;

// ============================================================================
// ESTADO ICM-20948: ACTITUD Y ACELERACION 2D
// ============================================================================
float icmRawX_mg = 0.0f, icmRawY_mg = 0.0f, icmRawZ_mg = 0.0f;
float icmRawGyrX_dps = 0.0f, icmRawGyrY_dps = 0.0f, icmRawGyrZ_dps = 0.0f;
float icmBodyAx_g = 0.0f, icmBodyAy_g = 0.0f, icmBodyAz_g = 0.0f;
float icmBodyGx_dps = 0.0f, icmBodyGy_dps = 0.0f, icmBodyGz_dps = 0.0f;
float icmGyroBiasSensorX_dps = 0.0f, icmGyroBiasSensorY_dps = 0.0f, icmGyroBiasSensorZ_dps = 0.0f;
float icmBodyZeroX_mps2 = 0.0f, icmBodyZeroY_mps2 = 0.0f;
float icmMag_g = 0.0f;
float icmRoll_deg = 0.0f, icmPitch_deg = 0.0f;
float icmBodyAccelXRaw_mps2 = 0.0f, icmBodyAccelYRaw_mps2 = 0.0f;
float icmBodyAccelX_mps2 = 0.0f, icmBodyAccelY_mps2 = 0.0f;
float icmAccelE_mps2 = 0.0f, icmAccelN_mps2 = 0.0f;
bool attitudeInitialized = false;
uint32_t lastIcmSampleUs = 0;

// ============================================================================
// ESTADO MPU6500: VIBRACION Y CONFIRMACION DE REPOSO
// ============================================================================
float mpuRawX_g = 0.0f, mpuRawY_g = 0.0f, mpuRawZ_g = 0.0f;
float mpuBodyAx_g = 0.0f, mpuBodyAy_g = 0.0f, mpuBodyAz_g = 0.0f;
float mpuMag_g = 0.0f;
uint32_t lastMpuSampleMs = 0;

// Derivada de la rapidez GNSS usada para aprender el eje longitudinal.
bool headingCalPrevGpsValid = false;
float headingCalPrevSpeed_mps = 0.0f;
uint32_t headingCalPrevGpsMs = 0;
float headingCalGpsAccel_mps2 = 0.0f;
float currentMotionHeadingTrue_deg = 0.0f;
float gEstX = 0.0f, gEstY = 0.0f, gEstZ = 0.0f;
bool gEstInit = false;

// Gravedad de referencia propia de cada sensor, medida en reposo al arrancar.
// Absorbe el sesgo de fabrica de cada unidad. Si la calibracion de piso no se
// completa, se conserva la banda absoluta como respaldo.
float icmGravityRef_g = 1.0f;
float mpuGravityRef_g = 1.0f;
bool icmGravityRefValid = false;
bool mpuGravityRefValid = false;

// Estado de reposo y de aplicacion del ZUPT, expuesto para el registro.
bool stillDetected = false;
bool zuptApplied = false;
float vibAxLin_g = 0.0f, vibAyLin_g = 0.0f, vibAzLin_g = 0.0f;
float vibMagLin_g = 0.0f;

// ============================================================================
// FILTRO DE KALMAN 2D: un eje [posicion, velocidad, bias] para E y N
// ============================================================================
struct AxisKalman {
  float p;
  float v;
  float bias;
  float P[3][3];
};

bool fusionInitialized = false;
AxisKalman fusionEast = {};
AxisKalman fusionNorth = {};
double fusedLat = 0.0, fusedLon = 0.0;
float lastSpeedInnovation_mps = 0.0f;
float lastPositionInnovation_m = 0.0f;

#if ENABLE_WHEEL_ODOMETRY
volatile uint32_t wheelPulseCounter = 0;
uint32_t lastWheelPulseSnapshot = 0;
uint32_t lastWheelUpdateMs = 0;
float wheelSpeed_mps = 0.0f;
#endif

// ============================================================================
// SUPERVISION DE IMU
// ============================================================================
enum ImuHealth : uint8_t {
  IMU_UNKNOWN = 0,
  IMU_OK = 1,
  IMU_ICM_DRIFT = 2,
  IMU_DISAGREE = 3,
  IMU_SENSOR_FAIL = 4
};

ImuHealth imuHealth = IMU_UNKNOWN;
uint32_t icmDriftStartMs = 0;
uint32_t imuDisagreeStartMs = 0;
uint32_t stillStartMs = 0;

// ============================================================================
// FUENTE FINAL / PAYLOAD
// ============================================================================
enum PositionSource : uint8_t {
  SRC_NONE = 0,
  SRC_GNSS_FUSED = 1,
  SRC_GENERAL_DR = 2
};

PositionSource txSource = SRC_NONE;
double txLat = 0.0, txLon = 0.0;
float txSpeedKmh = 0.0f;
float txHeading_deg = -1.0f;
float txVelE_mps = 0.0f, txVelN_mps = 0.0f;

// ============================================================================
// SD Y ARCHIVOS
// ============================================================================
// La SD conserva de forma permanente los archivos diarios de vibracion.
// LASTPOS.TXT conserva solo la ultima posicion GNSS valida y se sobrescribe
// de forma controlada. PENDING.TXT conserva un unico paquete no confirmado,
// pero durante una interrupcion se reemplaza cada 15 s con el estado mas reciente.
File vibFile;
char vibFilename[20] = {0};
uint16_t lastVibYear = 65535;
uint8_t lastVibMonth = 255, lastVibDay = 255;
uint32_t lastVibFlushMs = 0;

static const char PENDING_FILE[] = "PENDING.TXT";
static const char LASTPOS_FILE[] = "LASTPOS.TXT";
static const char LASTPOS_TEMP_FILE[] = "LASTPOS.TMP";
bool havePending = false;
String pendingLine = "";
// Momento en que el contenido pendiente fue generado a partir de los sensores.
// No se actualiza por un simple reintento del mismo paquete.
uint32_t lastPendingRefreshMs = 0;
// Un paquete recuperado de SD se refresca tan pronto exista una posicion actual
// utilizable, antes de volver a transmitirlo.
bool pendingNeedsRefresh = false;
bool lastPosSavedFromCurrentFix = false;
uint32_t lastPosSaveMs = 0;
uint32_t lastSdUsageCheckMs = 0;

// Administrador de fallas y recuperacion de la microSD.
uint8_t sdConsecutiveErrors = 0;
uint32_t lastSdHealthCheckMs = 0;
uint32_t lastSdRetryMs = 0;
uint32_t lastSdFaultBlinkMs = 0;
bool sdFaultLedState = false;
bool pendingRemovalRequired = false;
String sdLastError = "";
static const char SD_PROBE_FILE[] = "SD_PROBE.TMP";

// ============================================================================
// TEMPORIZADORES GENERALES Y LEDS
// ============================================================================
uint32_t lastHeartbeatMs = 0;
bool heartbeatState = false;
uint32_t ledSdOffMs = 0;
uint32_t ledTsOffMs = 0;
uint32_t lastVibLogMs = 0;
uint32_t lastOledMs = 0;

// ============================================================================
// LTE FSM
// ============================================================================
enum LteState : uint8_t {
  LTE_IDLE = 0,
  LTE_SEND_AT, LTE_WAIT_AT,
  LTE_SEND_ATE0, LTE_WAIT_ATE0,
  LTE_SEND_CFUN, LTE_WAIT_CFUN,
  LTE_SEND_CGDCONT, LTE_WAIT_CGDCONT,
  LTE_SEND_CGATT, LTE_WAIT_CGATT,
  LTE_SEND_CGACT, LTE_WAIT_CGACT,
  LTE_SEND_NETOPEN, LTE_WAIT_NETOPEN,
  LTE_SEND_HTTPTERM_PRE, LTE_WAIT_HTTPTERM_PRE,
  LTE_SEND_HTTPINIT, LTE_WAIT_HTTPINIT,
  LTE_SEND_CID, LTE_WAIT_CID,
  LTE_SEND_URL, LTE_WAIT_URL,
  LTE_SEND_CONTENT, LTE_WAIT_CONTENT,
  LTE_SEND_HTTPDATA, LTE_WAIT_DOWNLOAD,
  LTE_SEND_BODY, LTE_WAIT_BODY_OK,
  LTE_SEND_HTTPACTION, LTE_WAIT_HTTPACTION,
  LTE_SEND_HTTPTERM_POST, LTE_WAIT_HTTPTERM_POST,
  LTE_DONE,
  LTE_FAIL
};

LteState lteState = LTE_IDLE;
uint32_t lteStateStartMs = 0;
uint32_t lastSendMs = 0;
uint32_t lastRetryMs = 0;
String modemRxBuffer = "";

bool rxOK = false;
bool rxERROR = false;
bool rxDOWNLOAD = false;
bool rxNetOpenSuccess = false;
bool rxNetAlreadyOpen = false;
bool rxHttpAction = false;
int httpCode = -1;
int httpResponseLength = 0;
bool cycleHttpSuccess = false;

// ============================================================================
// UTILIDADES GENERALES
// ============================================================================
static inline bool elapsed(uint32_t now, uint32_t since, uint32_t period) {
  return (uint32_t)(now - since) >= period;
}

static inline bool lteTimedOut(uint32_t timeoutMs) {
  return elapsed(millis(), lteStateStartMs, timeoutMs);
}

Vec3f makeVec3(float x, float y, float z) {
  Vec3f v = {x, y, z};
  return v;
}

Vec3f addVec3(const Vec3f &a, const Vec3f &b) {
  return makeVec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vec3f scaleVec3(const Vec3f &v, float k) {
  return makeVec3(v.x * k, v.y * k, v.z * k);
}

float dotVec3(const Vec3f &a, const Vec3f &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3f crossVec3(const Vec3f &a, const Vec3f &b) {
  return makeVec3(a.y * b.z - a.z * b.y,
                  a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x);
}

float normVec3(const Vec3f &v) {
  return sqrtf(dotVec3(v, v));
}

float angularDifferenceDeg(float a, float b) {
  float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return fabsf(d);
}

bool normalizeVec3(Vec3f &v) {
  float n = normVec3(v);
  if (!isfinite(n) || n < 1.0e-6f) return false;
  v = scaleVec3(v, 1.0f / n);
  return true;
}

Vec3f projectToPlane(const Vec3f &v, const Vec3f &normalUnit) {
  return addVec3(v, scaleVec3(normalUnit, -dotVec3(v, normalUnit)));
}

Vec3f manualAxisVector(uint8_t axis, float sign) {
  Vec3f v = {0.0f, 0.0f, 0.0f};
  if (axis == 0) v.x = sign;
  else if (axis == 1) v.y = sign;
  else v.z = sign;
  return v;
}

bool rebuildAlignmentBasis(SensorAlignment &alignment,
                           uint8_t fallbackForwardAxis,
                           float fallbackForwardSign) {
  if (!alignment.floorValid) return false;

  Vec3f up = alignment.upSensor;
  if (!normalizeVec3(up)) return false;

  Vec3f forward = alignment.headingValid
      ? alignment.forwardSensor
      : manualAxisVector(fallbackForwardAxis, fallbackForwardSign);
  forward = projectToPlane(forward, up);

  // Si el eje provisional elegido resulta casi vertical, se selecciona
  // automaticamente el eje cartesiano del sensor menos paralelo a la gravedad.
  if (!normalizeVec3(forward)) {
    alignment.headingValid = false;
    Vec3f candidate[3] = {
      makeVec3(1.0f, 0.0f, 0.0f),
      makeVec3(0.0f, 1.0f, 0.0f),
      makeVec3(0.0f, 0.0f, 1.0f)
    };
    uint8_t best = 0;
    float bestParallel = fabsf(dotVec3(candidate[0], up));
    for (uint8_t i = 1; i < 3; ++i) {
      float parallel = fabsf(dotVec3(candidate[i], up));
      if (parallel < bestParallel) {
        best = i;
        bestParallel = parallel;
      }
    }
    forward = projectToPlane(candidate[best], up);
    if (!normalizeVec3(forward)) return false;
  }

  // Y derecha = X adelante x Z arriba. Se conserva esta convencion porque
  // el filtro de pitch existente interpreta gyro Y positivo como nariz arriba.
  Vec3f right = crossVec3(forward, up);
  if (!normalizeVec3(right)) return false;
  forward = crossVec3(up, right);
  if (!normalizeVec3(forward)) return false;

  alignment.upSensor = up;
  alignment.forwardSensor = forward;
  alignment.rightSensor = right;
  return true;
}

void manualAxesToBody(float sx, float sy, float sz,
                      uint8_t xAxis, uint8_t yAxis, uint8_t zAxis,
                      float xSign, float ySign, float zSign,
                      float &bx, float &by, float &bz) {
  const float value[3] = {sx, sy, sz};
  bx = xSign * value[xAxis];
  by = ySign * value[yAxis];
  bz = zSign * value[zAxis];
}

void transformSensorToTrain(const SensorAlignment &alignment,
                            float sx, float sy, float sz,
                            uint8_t xAxis, uint8_t yAxis, uint8_t zAxis,
                            float xSign, float ySign, float zSign,
                            float &bx, float &by, float &bz) {
  if (alignment.floorValid) {
    Vec3f v = makeVec3(sx, sy, sz);
    bx = dotVec3(v, alignment.forwardSensor);
    by = dotVec3(v, alignment.rightSensor);
    bz = dotVec3(v, alignment.upSensor);
  } else {
    manualAxesToBody(sx, sy, sz, xAxis, yAxis, zAxis,
                     xSign, ySign, zSign, bx, by, bz);
  }
}

void icmSensorToTrain(float sx, float sy, float sz,
                      float &bx, float &by, float &bz) {
  transformSensorToTrain(icmAlignment, sx, sy, sz,
                         ICM_BODY_X_SENSOR_AXIS,
                         ICM_BODY_Y_SENSOR_AXIS,
                         ICM_BODY_Z_SENSOR_AXIS,
                         ICM_BODY_X_SIGN,
                         ICM_BODY_Y_SIGN,
                         ICM_BODY_Z_SIGN,
                         bx, by, bz);
}

void mpuSensorToTrain(float sx, float sy, float sz,
                      float &bx, float &by, float &bz) {
  transformSensorToTrain(mpuAlignment, sx, sy, sz,
                         MPU_BODY_X_SENSOR_AXIS,
                         MPU_BODY_Y_SENSOR_AXIS,
                         MPU_BODY_Z_SENSOR_AXIS,
                         MPU_BODY_X_SIGN,
                         MPU_BODY_Y_SIGN,
                         MPU_BODY_Z_SIGN,
                         bx, by, bz);
}

bool coordinatesAreValid(double lat, double lon) {
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
  if (fabs(lat) < 0.000001 && fabs(lon) < 0.000001) return false;
  return true;
}

float normalizeHeadingDeg(float value) {
  while (value < 0.0f) value += 360.0f;
  while (value >= 360.0f) value -= 360.0f;
  return value;
}

float signedHeadingErrorDeg(float target, float current) {
  return fmodf(target - current + 540.0f, 360.0f) - 180.0f;
}

float blendHeadingDeg(float current, float target, float gain) {
  if (gain < 0.0f) gain = 0.0f;
  if (gain > 1.0f) gain = 1.0f;
  return normalizeHeadingDeg(current + gain * signedHeadingErrorDeg(target, current));
}

void setLocalOrigin(double lat, double lon) {
  if (!coordinatesAreValid(lat, lon)) return;
  localOriginLat = lat;
  localOriginLon = lon;
  localOriginValid = true;
}

void latLonToLocal(double lat, double lon, float &east, float &north) {
  east = north = 0.0f;
  if (!localOriginValid) return;
  const double earthRadius = 6378137.0;
  const double lat0Rad = localOriginLat * PI / 180.0;
  east = (float)((lon - localOriginLon) * PI / 180.0 *
                 earthRadius * cos(lat0Rad));
  north = (float)((lat - localOriginLat) * PI / 180.0 * earthRadius);
}

void localToLatLon(float east, float north, double &lat, double &lon) {
  if (!localOriginValid) {
    lat = lon = 0.0;
    return;
  }
  const double earthRadius = 6378137.0;
  const double lat0Rad = localOriginLat * PI / 180.0;
  lat = localOriginLat + (north / earthRadius) * 180.0 / PI;
  double denom = earthRadius * cos(lat0Rad);
  if (fabs(denom) < 1.0) denom = 1.0;
  lon = localOriginLon + (east / denom) * 180.0 / PI;
}

void updateLegacyNsDirection(float velNorth_mps) {
  if (fabsf(velNorth_mps) < LEGACY_NS_MIN_VN_MPS) return;
  dirNS = velNorth_mps >= 0.0f ? 0 : 1;
  directionKnown = true;
}

const char *directionName() {
  if (!directionKnown) return "--";
  return dirNS == 0 ? "N" : "S";
}

float bodyHeadingForMotion() {
  if (!northReferenceValid) return -1.0f;
  return normalizeHeadingDeg(bodyHeadingTrue_deg +
                             (motionDirectionSign < 0 ? 180.0f : 0.0f));
}

void formatDateTime(char *dateStr, size_t dateSize,
                    char *timeStr, size_t timeSize) {
  if (gpsTimeValid) {
    snprintf(dateStr, dateSize, "%04u-%02u-%02u", gpsYear, gpsMonth, gpsDay);
    snprintf(timeStr, timeSize, "%02u:%02u:%02u", gpsHour, gpsMinute, gpsSecond);
  } else {
    snprintf(dateStr, dateSize, "0000-00-00");
    snprintf(timeStr, timeSize, "00:00:00");
  }
}

const char *sourceName(PositionSource source) {
  switch (source) {
    case SRC_GNSS_FUSED: return "GNSS+ENU";
    case SRC_GENERAL_DR: return "DR+ENU";
    default: return "NONE";
  }
}

const char *healthName(ImuHealth health) {
  switch (health) {
    case IMU_OK: return "OK";
    case IMU_ICM_DRIFT: return "ICM_DRIFT";
    case IMU_DISAGREE: return "DISAGREE";
    case IMU_SENSOR_FAIL: return "SENSOR_FAIL";
    default: return "UNKNOWN";
  }
}

void setSdUnavailable(const char *reason) {
  if (vibFile) vibFile.close();
  SD.end(false);  // desmonta SDFS sin apagar el bus SPI compartido

  sdOK = false;
  sdLastError = reason ? String(reason) : String("falla desconocida");
  sdConsecutiveErrors = SD_MAX_CONSECUTIVE_ERRORS;
  lastSdRetryMs = millis();
  ledSdOffMs = 0;
  sdFaultLedState = true;
  lastSdFaultBlinkMs = millis();
  digitalWrite(LED_SD, HIGH);

  Serial.print("ERROR SD: ");
  Serial.println(sdLastError);
  Serial.println("El nodo continua en RAM/LTE y reintentara la SD cada 5 s.");
}

void noteSdSuccess() {
  if (!sdOK) return;
  sdConsecutiveErrors = 0;
  sdLastError = "";
}

void noteSdFailure(const char *operation, bool immediate = false) {
  sdLastError = operation ? String(operation) : String("operacion SD");

  if (!sdOK) return;
  if (sdConsecutiveErrors < 255) sdConsecutiveErrors++;

  Serial.print("WARN SD (");
  Serial.print(sdConsecutiveErrors);
  Serial.print('/');
  Serial.print(SD_MAX_CONSECUTIVE_ERRORS);
  Serial.print("): ");
  Serial.println(sdLastError);

  if (immediate || sdConsecutiveErrors >= SD_MAX_CONSECUTIVE_ERRORS) {
    setSdUnavailable(operation);
  }
}

void pulseSdLed(uint32_t durationMs = 30) {
  if (!sdOK) return;
  digitalWrite(LED_SD, HIGH);
  ledSdOffMs = millis() + durationMs;
}

void pulseTsLed(uint32_t durationMs = 100) {
  digitalWrite(LED_TS_OK, HIGH);
  ledTsOffMs = millis() + durationMs;
}

void serviceLedPulses() {
  uint32_t now = millis();

  if (!sdOK) {
    // Patron de falla: GP14 alterna cada 250 ms hasta recuperar la tarjeta.
    ledSdOffMs = 0;
    if (elapsed(now, lastSdFaultBlinkMs, SD_FAULT_LED_BLINK_MS)) {
      lastSdFaultBlinkMs = now;
      sdFaultLedState = !sdFaultLedState;
      digitalWrite(LED_SD, sdFaultLedState ? HIGH : LOW);
    }
  } else if (ledSdOffMs != 0 && (int32_t)(now - ledSdOffMs) >= 0) {
    digitalWrite(LED_SD, LOW);
    ledSdOffMs = 0;
  }

  if (ledTsOffMs != 0 && (int32_t)(now - ledTsOffMs) >= 0) {
    digitalWrite(LED_TS_OK, LOW);
    ledTsOffMs = 0;
  }
}

// ============================================================================
// PERSISTENCIA DE CALIBRACION DE ORIENTACION
// ============================================================================
uint32_t calibrationChecksum(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < length; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool alignmentVectorsAreValid(const Vec3f &up, const Vec3f &forward) {
  float nu = normVec3(up);
  float nf = normVec3(forward);
  if (!isfinite(nu) || !isfinite(nf)) return false;
  if (nu < 0.80f || nu > 1.20f || nf < 0.80f || nf > 1.20f) return false;
  Vec3f u = up;
  Vec3f f = forward;
  if (!normalizeVec3(u) || !normalizeVec3(f)) return false;
  return fabsf(dotVec3(u, f)) < 0.25f;
}

void clearHeadingAccumulator(SensorAlignment &alignment) {
  alignment.headingCorrelation = makeVec3(0.0f, 0.0f, 0.0f);
  alignment.headingSamples = 0;
  alignment.headingExcitation = 0.0f;
  alignment.headingWeight = 0.0f;
}

bool loadImuCalibrationFromSd() {
  if (!sdOK || !SD.exists(IMU_CAL_FILE)) return false;

  File file = SD.open(IMU_CAL_FILE, FILE_READ);
  if (!file) {
    noteSdFailure("no se pudo abrir IMUCAL.BIN");
    return false;
  }

  ImuCalibrationRecord record = {};
  size_t readCount = file.read((uint8_t *)&record, sizeof(record));
  file.close();
  if (readCount != sizeof(record)) return false;

  uint32_t expected = record.crc;
  record.crc = 0;
  uint32_t actual = calibrationChecksum((const uint8_t *)&record, sizeof(record));
  if (record.magic != IMU_CAL_MAGIC ||
      record.version != IMU_CAL_VERSION ||
      record.size != sizeof(record) ||
      expected != actual) {
    Serial.println("WARN: IMUCAL.BIN es de otra version o esta danado; se recalibrara.");
    return false;
  }

  Vec3f icmUp = makeVec3(record.icmUp[0], record.icmUp[1], record.icmUp[2]);
  Vec3f icmForward = makeVec3(record.icmForward[0], record.icmForward[1], record.icmForward[2]);
  Vec3f mpuUp = makeVec3(record.mpuUp[0], record.mpuUp[1], record.mpuUp[2]);
  Vec3f mpuForward = makeVec3(record.mpuForward[0], record.mpuForward[1], record.mpuForward[2]);

  if (record.icmFloorValid && alignmentVectorsAreValid(icmUp, icmForward)) {
    icmAlignment.upSensor = icmUp;
    icmAlignment.forwardSensor = icmForward;
    icmAlignment.floorValid = true;
    icmAlignment.headingValid = record.icmHeadingValid != 0;
    icmAlignment.headingQuality = record.icmHeadingQuality;
    rebuildAlignmentBasis(icmAlignment,
                          ICM_BODY_X_SENSOR_AXIS, ICM_BODY_X_SIGN);
  }

  if (record.mpuFloorValid && alignmentVectorsAreValid(mpuUp, mpuForward)) {
    mpuAlignment.upSensor = mpuUp;
    mpuAlignment.forwardSensor = mpuForward;
    mpuAlignment.floorValid = true;
    mpuAlignment.headingValid = record.mpuHeadingValid != 0;
    mpuAlignment.headingQuality = record.mpuHeadingQuality;
    rebuildAlignmentBasis(mpuAlignment,
                          MPU_BODY_X_SENSOR_AXIS, MPU_BODY_X_SIGN);
  }

  bodyHeadingTrue_deg = normalizeHeadingDeg(record.bodyHeadingTrue_deg);
  northReferenceValid = record.northReferenceValid != 0 &&
                        icmAlignment.headingValid &&
                        isfinite(bodyHeadingTrue_deg);
  motionDirectionSign = record.motionDirectionSign < 0 ? -1 : 1;
  northReferenceQuality = northReferenceValid ? 0.35f : 0.0f;

  clearHeadingAccumulator(icmAlignment);
  clearHeadingAccumulator(mpuAlignment);
  imuCalibrationSequence = record.sequence;
  imuCalibrationLoaded = icmAlignment.headingValid || mpuAlignment.headingValid;
  noteSdSuccess();

  Serial.print("Calibracion IMU recuperada. ICM=");
  Serial.print(icmAlignment.headingValid ? "PISO+EJE" :
               (icmAlignment.floorValid ? "PISO" : "NO"));
  Serial.print(" MPU=");
  Serial.print(mpuAlignment.headingValid ? "PISO+EJE" :
               (mpuAlignment.floorValid ? "PISO" : "NO"));
  Serial.print(" NORTE=");
  Serial.println(northReferenceValid ? "RECUPERADO" : "PENDIENTE");
  return imuCalibrationLoaded;
}

bool saveImuCalibrationToSd(bool force = false) {
  if (!sdOK) return false;
  uint32_t now = millis();
  if (!force && (!imuCalibrationDirty ||
                 !elapsed(now, lastImuCalibrationSaveMs, IMU_CAL_SAVE_MIN_MS)))
    return false;

  ImuCalibrationRecord record = {};
  record.magic = IMU_CAL_MAGIC;
  record.version = IMU_CAL_VERSION;
  record.size = sizeof(record);
  record.sequence = ++imuCalibrationSequence;
  record.icmFloorValid = icmAlignment.floorValid ? 1 : 0;
  record.icmHeadingValid = icmAlignment.headingValid ? 1 : 0;
  record.mpuFloorValid = mpuAlignment.floorValid ? 1 : 0;
  record.mpuHeadingValid = mpuAlignment.headingValid ? 1 : 0;
  record.northReferenceValid = northReferenceValid ? 1 : 0;
  record.motionDirectionSign = motionDirectionSign;
  record.icmUp[0] = icmAlignment.upSensor.x;
  record.icmUp[1] = icmAlignment.upSensor.y;
  record.icmUp[2] = icmAlignment.upSensor.z;
  record.icmForward[0] = icmAlignment.forwardSensor.x;
  record.icmForward[1] = icmAlignment.forwardSensor.y;
  record.icmForward[2] = icmAlignment.forwardSensor.z;
  record.mpuUp[0] = mpuAlignment.upSensor.x;
  record.mpuUp[1] = mpuAlignment.upSensor.y;
  record.mpuUp[2] = mpuAlignment.upSensor.z;
  record.mpuForward[0] = mpuAlignment.forwardSensor.x;
  record.mpuForward[1] = mpuAlignment.forwardSensor.y;
  record.mpuForward[2] = mpuAlignment.forwardSensor.z;
  record.icmHeadingQuality = icmAlignment.headingQuality;
  record.mpuHeadingQuality = mpuAlignment.headingQuality;
  record.bodyHeadingTrue_deg = bodyHeadingTrue_deg;
  record.crc = 0;
  record.crc = calibrationChecksum((const uint8_t *)&record, sizeof(record));

  if (SD.exists(IMU_CAL_TEMP_FILE) && !SD.remove(IMU_CAL_TEMP_FILE)) {
    noteSdFailure("no se pudo limpiar IMUCAL.TMP");
    return false;
  }

  File file = SD.open(IMU_CAL_TEMP_FILE, FILE_WRITE);
  if (!file) {
    noteSdFailure("no se pudo crear IMUCAL.TMP");
    return false;
  }
  file.clearWriteError();
  size_t written = file.write((const uint8_t *)&record, sizeof(record));
  file.flush();
  bool failed = written != sizeof(record) || file.getWriteError();
  file.close();
  if (failed) {
    noteSdFailure("fallo al escribir IMUCAL.TMP");
    return false;
  }

  if (SD.exists(IMU_CAL_FILE) && !SD.remove(IMU_CAL_FILE)) {
    noteSdFailure("no se pudo reemplazar IMUCAL.BIN");
    return false;
  }
  if (!SD.rename(IMU_CAL_TEMP_FILE, IMU_CAL_FILE)) {
    noteSdFailure("no se pudo promover IMUCAL.TMP");
    return false;
  }

  imuCalibrationDirty = false;
  lastImuCalibrationSaveMs = now;
  noteSdSuccess();
  Serial.println("Calibracion general guardada en IMUCAL.BIN.");
  return true;
}

// ============================================================================
// I2C / MPU6500
// I2C / MPU6500
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

bool initMpu6500() {
  uint8_t whoAmI = 0;
  if (!i2cReadN(MPU_ADDR, MPU_RA_WHO_AM_I, &whoAmI, 1)) return false;

  Serial.print("MPU6500 WHO_AM_I=0x");
  Serial.println(whoAmI, HEX);

  if (!i2cWrite8(MPU_ADDR, MPU_RA_PWR_MGMT_1, 0x00)) return false;
  delay(10);
  if (!i2cWrite8(MPU_ADDR, MPU_RA_SMPLRT_DIV, 9)) return false;       // ~100 Hz interno
  if (!i2cWrite8(MPU_ADDR, MPU_RA_CONFIG, 0x03)) return false;        // DLPF
  if (!i2cWrite8(MPU_ADDR, MPU_RA_ACCEL_CONFIG, 0x08)) return false;  // +/-4 g
  return true;
}

bool readMpuRawG(float &axG, float &ayG, float &azG) {
  uint8_t bytes[6];
  if (!i2cReadN(MPU_ADDR, MPU_RA_ACCEL_XOUT_H, bytes, 6)) return false;

  int16_t ax = (int16_t)((bytes[0] << 8) | bytes[1]);
  int16_t ay = (int16_t)((bytes[2] << 8) | bytes[3]);
  int16_t az = (int16_t)((bytes[4] << 8) | bytes[5]);

  axG = (float)ax / MPU_LSB_PER_G;
  ayG = (float)ay / MPU_LSB_PER_G;
  azG = (float)az / MPU_LSB_PER_G;
  return true;
}

void initializeMpuGravityEstimate() {
  if (!mpuPresent) return;
  Vec3f sum = makeVec3(0.0f, 0.0f, 0.0f);
  int valid = 0;
  uint32_t start = millis();
  Serial.println("Calibrando piso MPU6500, no mover...");
  while (valid < MPU_FLOOR_CAL_SAMPLES && !elapsed(millis(), start, 5000)) {
    float x, y, z;
    if (readMpuRawG(x, y, z)) {
      sum = addVec3(sum, makeVec3(x, y, z));
      valid++;
    }
    delay(8);
  }

  if (valid > 0) {
    Vec3f mean = scaleVec3(sum, 1.0f / valid);
    float magnitude = normVec3(mean);
    gEstX = mean.x;
    gEstY = mean.y;
    gEstZ = mean.z;
    gEstInit = true;

    // Gravedad propia de esta unidad, para la deteccion de reposo.
    if (magnitude >= FLOOR_CAL_MIN_MAG_G && magnitude <= FLOOR_CAL_MAX_MAG_G) {
      mpuGravityRef_g = magnitude;
      mpuGravityRefValid = true;
      Serial.print("Gravedad de referencia MPU="); Serial.println(magnitude, 4);
    }

    if (magnitude >= FLOOR_CAL_MIN_MAG_G &&
        magnitude <= FLOOR_CAL_MAX_MAG_G && normalizeVec3(mean)) {
      Vec3f previousForward = mpuAlignment.forwardSensor;
      bool hadHeading = mpuAlignment.headingValid;
      mpuAlignment.upSensor = mean;
      mpuAlignment.floorValid = true;
      if (hadHeading) mpuAlignment.forwardSensor = previousForward;
      rebuildAlignmentBasis(mpuAlignment,
                            MPU_BODY_X_SENSOR_AXIS, MPU_BODY_X_SIGN);
      clearHeadingAccumulator(mpuAlignment);
      imuCalibrationDirty = true;
      Serial.print("Piso MPU OK, muestras="); Serial.print(valid);
      Serial.print(" |g|="); Serial.println(magnitude, 4);
    } else {
      Serial.print("WARN: piso MPU rechazado, |g|=");
      Serial.println(magnitude, 4);
    }
  }
}

bool updateMpuSample(uint32_t now) {
  if (!mpuPresent || !elapsed(now, lastMpuSampleMs, MPU_SAMPLE_MS)) return false;

  float dt = (lastMpuSampleMs == 0) ? 0.0f : (now - lastMpuSampleMs) / 1000.0f;
  lastMpuSampleMs = now;

  if (!readMpuRawG(mpuRawX_g, mpuRawY_g, mpuRawZ_g)) return false;

  mpuMag_g = sqrtf(mpuRawX_g * mpuRawX_g +
                   mpuRawY_g * mpuRawY_g +
                   mpuRawZ_g * mpuRawZ_g);

  if (dt <= 0.0f || dt > 0.25f) dt = MPU_SAMPLE_MS / 1000.0f;
  float rc = 1.0f / (2.0f * PI * GRAV_CUTOFF_HZ);
  float alpha = dt / (rc + dt);

  if (!gEstInit) {
    gEstX = mpuRawX_g;
    gEstY = mpuRawY_g;
    gEstZ = mpuRawZ_g;
    gEstInit = true;
  } else {
    gEstX += alpha * (mpuRawX_g - gEstX);
    gEstY += alpha * (mpuRawY_g - gEstY);
    gEstZ += alpha * (mpuRawZ_g - gEstZ);
  }

  // Las vibraciones se guardan en ejes del nodo:
  // longitudinal aprendido, lateral derecha y vertical arriba.
  mpuSensorToTrain(mpuRawX_g, mpuRawY_g, mpuRawZ_g,
                   mpuBodyAx_g, mpuBodyAy_g, mpuBodyAz_g);
  mpuSensorToTrain(mpuRawX_g - gEstX,
                   mpuRawY_g - gEstY,
                   mpuRawZ_g - gEstZ,
                   vibAxLin_g, vibAyLin_g, vibAzLin_g);
  vibMagLin_g = sqrtf(vibAxLin_g * vibAxLin_g +
                      vibAyLin_g * vibAyLin_g +
                      vibAzLin_g * vibAzLin_g);
  return true;
}

// ============================================================================
// CALIBRACION DEL EJE DEL NODO Y REFERENCIA NORTE CON GNSS
// ============================================================================
int8_t observeMotionDirectionSign(const SensorAlignment &alignment,
                                  const Vec3f &rawAccelSensor_g,
                                  float gpsSpeedAccel_mps2) {
  if (!alignment.floorValid || !alignment.headingValid ||
      fabsf(gpsSpeedAccel_mps2) < HEADING_CAL_MIN_GPS_ACCEL_MPS2) return 0;

  Vec3f horizontal = projectToPlane(rawAccelSensor_g, alignment.upSensor);
  if (!normalizeVec3(horizontal)) return 0;
  // Al frenar, la aceleracion apunta contra el movimiento. Multiplicar por el
  // signo de d|v|/dt produce un vector que apunta en el sentido de movimiento.
  if (gpsSpeedAccel_mps2 < 0.0f) horizontal = scaleVec3(horizontal, -1.0f);
  return dotVec3(horizontal, alignment.forwardSensor) >= 0.0f ? 1 : -1;
}

void addHeadingCalibrationObservation(SensorAlignment &alignment,
                                      const Vec3f &rawAccelSensor_g,
                                      float gpsSpeedAccel_mps2,
                                      float dt,
                                      uint8_t fallbackForwardAxis,
                                      float fallbackForwardSign,
                                      const char *sensorName,
                                      bool navigationSensor) {
  if (!alignment.floorValid) return;

  Vec3f horizontal = projectToPlane(rawAccelSensor_g, alignment.upSensor);
  float horizontalMag = normVec3(horizontal);
  if (horizontalMag < HEADING_CAL_MIN_HORIZONTAL_G) return;

  // Convierte la aceleracion en un vector que apunta en la direccion de marcha.
  if (gpsSpeedAccel_mps2 < 0.0f) horizontal = scaleVec3(horizontal, -1.0f);

  int8_t observedSign = 1;
  if (alignment.headingValid) {
    observedSign = dotVec3(horizontal, alignment.forwardSensor) >= 0.0f ? 1 : -1;
    // Para refinar un eje ya establecido se alinean observaciones de marcha
    // directa e inversa hacia el mismo +X del nodo.
    if (observedSign < 0) horizontal = scaleVec3(horizontal, -1.0f);
    if (navigationSensor) {
      motionDirectionSign = observedSign;
      lastMotionSignMs = millis();
    }
  }

  float weight = fabsf(gpsSpeedAccel_mps2) * dt;
  alignment.headingCorrelation = addVec3(
      alignment.headingCorrelation, scaleVec3(horizontal, weight));
  alignment.headingSamples++;
  alignment.headingExcitation += fabsf(gpsSpeedAccel_mps2) * dt;
  alignment.headingWeight += horizontalMag * weight;

  Vec3f candidate = projectToPlane(alignment.headingCorrelation,
                                   alignment.upSensor);
  float correlationNorm = normVec3(candidate);
  alignment.headingQuality = alignment.headingWeight > 1.0e-6f
      ? correlationNorm / alignment.headingWeight : 0.0f;
  if (alignment.headingQuality > 1.0f) alignment.headingQuality = 1.0f;

  bool enough = alignment.headingSamples >= HEADING_CAL_MIN_SAMPLES &&
                alignment.headingExcitation >= HEADING_CAL_MIN_EXCITATION_MPS &&
                alignment.headingQuality >= HEADING_CAL_MIN_QUALITY &&
                normalizeVec3(candidate);
  if (!enough) return;

  bool firstSolution = !alignment.headingValid;
  if (alignment.headingValid) {
    if (dotVec3(candidate, alignment.forwardSensor) < 0.0f)
      candidate = scaleVec3(candidate, -1.0f);
    Vec3f blended = addVec3(scaleVec3(alignment.forwardSensor, 0.90f),
                            scaleVec3(candidate, 0.10f));
    if (normalizeVec3(blended)) candidate = blended;
  }

  alignment.forwardSensor = candidate;
  alignment.headingValid = true;
  if (!rebuildAlignmentBasis(alignment,
                             fallbackForwardAxis, fallbackForwardSign)) {
    alignment.headingValid = false;
    return;
  }

  if (navigationSensor && firstSolution) {
    // La primera solucion define +X en el sentido de la marcha actual.
    motionDirectionSign = 1;
    lastMotionSignMs = millis();
  }

  imuCalibrationDirty = true;
  clearHeadingAccumulator(alignment);
  Serial.print("Autocal "); Serial.print(sensorName);
  Serial.print(firstSolution ? " completada" : " refinada");
  Serial.print(". Calidad=");
  Serial.println(alignment.headingQuality, 3);
}

void correctNorthReferenceFromGps(uint32_t now) {
  if (!gpsQualityCurrent || !icmAlignment.headingValid ||
      gpsSpeed_mps < HEADING_CAL_MIN_COURSE_SPEED_MPS ||
      gpsHeadingAcc_deg > HEADING_CAL_MAX_HEADACC_DEG) return;

  float course = normalizeHeadingDeg(gpsHeading_deg);
  float candidateBodyHeading = course;

  if (northReferenceValid) {
    float opposite = normalizeHeadingDeg(course + 180.0f);
    bool recentSign = lastMotionSignMs != 0 &&
                      !elapsed(now, lastMotionSignMs, MOTION_SIGN_VALID_MS);
    if (recentSign) {
      candidateBodyHeading = motionDirectionSign > 0 ? course : opposite;
    } else {
      candidateBodyHeading = angularDifferenceDeg(course, bodyHeadingTrue_deg) <=
                             angularDifferenceDeg(opposite, bodyHeadingTrue_deg)
          ? course : opposite;
      motionDirectionSign =
          angularDifferenceDeg(candidateBodyHeading, course) < 90.0f ? 1 : -1;
    }

    float gain = elapsed(now, lastNorthCorrectionMs, 5000)
        ? NORTH_REACQUIRE_GAIN : NORTH_CORRECTION_GAIN;
    bodyHeadingTrue_deg = blendHeadingDeg(bodyHeadingTrue_deg,
                                          candidateBodyHeading, gain);
  } else {
    // Al crearse el primer eje, +X se define en el sentido de esa marcha.
    candidateBodyHeading = motionDirectionSign > 0
        ? course : normalizeHeadingDeg(course + 180.0f);
    bodyHeadingTrue_deg = candidateBodyHeading;
    northReferenceValid = true;
  }

  currentMotionHeadingTrue_deg = course;
  lastNorthCorrectionMs = now;
  float headQuality = 1.0f - fminf(gpsHeadingAcc_deg / HEADING_CAL_MAX_HEADACC_DEG, 1.0f);
  northReferenceQuality = 0.85f * northReferenceQuality + 0.15f * headQuality;
  imuCalibrationDirty = true;
}

void updateHeadingCalibrationFromGps(uint32_t now) {
  if (!gpsQualityCurrent ||
      gpsSAcc_mmps / 1000.0f > HEADING_CAL_MAX_SACC_MPS) {
    headingCalPrevGpsValid = false;
    headingCalGpsAccel_mps2 = 0.0f;
    return;
  }

  if (!headingCalPrevGpsValid) {
    headingCalPrevGpsValid = true;
    headingCalPrevSpeed_mps = gpsSpeed_mps;
    headingCalPrevGpsMs = now;
    correctNorthReferenceFromGps(now);
    return;
  }

  float dt = (now - headingCalPrevGpsMs) / 1000.0f;
  float rawAccel = (dt > 0.05f) ?
      (gpsSpeed_mps - headingCalPrevSpeed_mps) / dt : 0.0f;
  headingCalPrevSpeed_mps = gpsSpeed_mps;
  headingCalPrevGpsMs = now;

  if (dt < 0.10f || dt > 1.0f || !isfinite(rawAccel) ||
      fabsf(rawAccel) > HEADING_CAL_MAX_GPS_ACCEL_MPS2) {
    correctNorthReferenceFromGps(now);
    return;
  }

  float rc = 1.0f / (2.0f * PI * HEADING_ACCEL_LPF_HZ);
  float alpha = dt / (rc + dt);
  headingCalGpsAccel_mps2 += alpha * (rawAccel - headingCalGpsAccel_mps2);

  if (fabsf(headingCalGpsAccel_mps2) >= HEADING_CAL_MIN_GPS_ACCEL_MPS2 &&
      fabsf(headingCalGpsAccel_mps2) <= HEADING_CAL_MAX_GPS_ACCEL_MPS2) {
    if (icmPresent) {
      addHeadingCalibrationObservation(
          icmAlignment,
          makeVec3(icmRawX_mg / 1000.0f,
                   icmRawY_mg / 1000.0f,
                   icmRawZ_mg / 1000.0f),
          headingCalGpsAccel_mps2, dt,
          ICM_BODY_X_SENSOR_AXIS, ICM_BODY_X_SIGN, "ICM", true);
    }

    if (mpuPresent) {
      addHeadingCalibrationObservation(
          mpuAlignment,
          makeVec3(mpuRawX_g, mpuRawY_g, mpuRawZ_g),
          headingCalGpsAccel_mps2, dt,
          MPU_BODY_X_SENSOR_AXIS, MPU_BODY_X_SIGN, "MPU", false);
    }
  }

  correctNorthReferenceFromGps(now);
  if (icmAlignment.headingValid || mpuAlignment.headingValid)
    saveImuCalibrationToSd(false);
}

// ============================================================================
// FILTRO DE KALMAN GENERAL 2D
// FILTRO DE KALMAN LONGITUDINAL
// ============================================================================
void axisInitialize(AxisKalman &axis, float position, float velocity,
                    bool recovered) {
  axis.p = position;
  axis.v = fminf(fmaxf(velocity, -FUSION_MAX_SPEED_MPS), FUSION_MAX_SPEED_MPS);
  axis.bias = 0.0f;
  for (uint8_t r = 0; r < 3; ++r)
    for (uint8_t c = 0; c < 3; ++c)
      axis.P[r][c] = 0.0f;
  axis.P[0][0] = recovered ? 400.0f : 25.0f;
  axis.P[1][1] = recovered ? 25.0f : 4.0f;
  axis.P[2][2] = 0.25f;
}

void fusionInitialize(float east_m, float north_m,
                      float velE_mps, float velN_mps,
                      bool recovered = false) {
  axisInitialize(fusionEast, east_m, velE_mps, recovered);
  axisInitialize(fusionNorth, north_m, velN_mps, recovered);
  fusionInitialized = true;
  hasPositionReference = true;
  localToLatLon(east_m, north_m, refLat, refLon);
}

void axisPredict(AxisKalman &axis, float measuredAccel_mps2, float dt) {
  float correctedAccel = measuredAccel_mps2 - axis.bias;
  float dt2 = dt * dt;
  axis.p += axis.v * dt + 0.5f * correctedAccel * dt2;
  axis.v += correctedAccel * dt;
  axis.v = fminf(fmaxf(axis.v, -FUSION_MAX_SPEED_MPS), FUSION_MAX_SPEED_MPS);

  float F[3][3] = {
    {1.0f, dt, -0.5f * dt2},
    {0.0f, 1.0f, -dt},
    {0.0f, 0.0f, 1.0f}
  };
  float FP[3][3] = {{0}};
  float Pnew[3][3] = {{0}};
  for (uint8_t i = 0; i < 3; ++i)
    for (uint8_t j = 0; j < 3; ++j)
      for (uint8_t k = 0; k < 3; ++k)
        FP[i][j] += F[i][k] * axis.P[k][j];

  for (uint8_t i = 0; i < 3; ++i)
    for (uint8_t j = 0; j < 3; ++j)
      for (uint8_t k = 0; k < 3; ++k)
        Pnew[i][j] += FP[i][k] * F[j][k];

  float qA = FUSION_ACCEL_NOISE_MPS2 * FUSION_ACCEL_NOISE_MPS2;
  float g0 = 0.5f * dt2;
  float g1 = dt;
  Pnew[0][0] += qA * g0 * g0;
  Pnew[0][1] += qA * g0 * g1;
  Pnew[1][0] += qA * g1 * g0;
  Pnew[1][1] += qA * g1 * g1;
  Pnew[2][2] += FUSION_BIAS_RW_MPS2 * FUSION_BIAS_RW_MPS2 * dt;

  for (uint8_t i = 0; i < 3; ++i)
    for (uint8_t j = 0; j < 3; ++j)
      axis.P[i][j] = Pnew[i][j];
}

void fusionPredict(float accelE_mps2, float accelN_mps2, float dt) {
  if (!fusionInitialized || dt <= 0.0f || dt > 0.20f) return;
  axisPredict(fusionEast, accelE_mps2, dt);
  axisPredict(fusionNorth, accelN_mps2, dt);
}

void axisUpdateScalar(AxisKalman &axis, uint8_t stateIndex,
                      float measurement, float variance) {
  if (stateIndex > 2 || variance <= 0.0f) return;
  float state[3] = {axis.p, axis.v, axis.bias};
  float innovation = measurement - state[stateIndex];
  float S = axis.P[stateIndex][stateIndex] + variance;
  if (S < 1.0e-6f) return;

  float K[3];
  float selectedRow[3];
  for (uint8_t i = 0; i < 3; ++i) {
    K[i] = axis.P[i][stateIndex] / S;
    selectedRow[i] = axis.P[stateIndex][i];
  }
  for (uint8_t i = 0; i < 3; ++i) state[i] += K[i] * innovation;

  float Pnew[3][3];
  for (uint8_t i = 0; i < 3; ++i)
    for (uint8_t j = 0; j < 3; ++j)
      Pnew[i][j] = axis.P[i][j] - K[i] * selectedRow[j];

  for (uint8_t i = 0; i < 3; ++i) {
    for (uint8_t j = 0; j < 3; ++j)
      axis.P[i][j] = 0.5f * (Pnew[i][j] + Pnew[j][i]);
    if (axis.P[i][i] < 1.0e-6f) axis.P[i][i] = 1.0e-6f;
  }

  axis.p = state[0];
  axis.v = fminf(fmaxf(state[1], -FUSION_MAX_SPEED_MPS), FUSION_MAX_SPEED_MPS);
  axis.bias = fminf(fmaxf(state[2], -1.5f), 1.5f);
}

void fusionUpdatePosition(float east_m, float north_m, float sigma_m) {
  if (!fusionInitialized) return;
  float de = east_m - fusionEast.p;
  float dn = north_m - fusionNorth.p;
  lastPositionInnovation_m = sqrtf(de * de + dn * dn);
  float variance = sigma_m * sigma_m;
  axisUpdateScalar(fusionEast, 0, east_m, variance);
  axisUpdateScalar(fusionNorth, 0, north_m, variance);
}

void fusionUpdateVelocity(float velE_mps, float velN_mps, float sigma_mps) {
  if (!fusionInitialized) return;
  float de = velE_mps - fusionEast.v;
  float dn = velN_mps - fusionNorth.v;
  lastSpeedInnovation_mps = sqrtf(de * de + dn * dn);
  float variance = sigma_mps * sigma_mps;
  axisUpdateScalar(fusionEast, 1, velE_mps, variance);
  axisUpdateScalar(fusionNorth, 1, velN_mps, variance);
}

void rebaseLocalFrameIfNeeded() {
  if (!fusionInitialized || !localOriginValid || !gpsUsable) return;
  float distance = sqrtf(fusionEast.p * fusionEast.p +
                         fusionNorth.p * fusionNorth.p);
  if (distance < LOCAL_REBASE_DISTANCE_M) return;

  double newLat, newLon;
  localToLatLon(fusionEast.p, fusionNorth.p, newLat, newLon);
  if (!coordinatesAreValid(newLat, newLon)) return;
  setLocalOrigin(newLat, newLon);
  fusionEast.p = 0.0f;
  fusionNorth.p = 0.0f;
  gpsLocalE_m = gpsLocalN_m = 0.0f;
  refLat = newLat;
  refLon = newLon;
  Serial.println("Marco ENU recentrado para conservar precision numerica.");
}

// ============================================================================
// ICM-20948: CONFIGURACION, CALIBRACION, ACTITUD Y PREDICCION
// ICM-20948: CONFIGURACION, CALIBRACION, ACTITUD Y PREDICCION
// ============================================================================
bool configureIcm20948() {
  if (!icmPresent) return false;

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

void calibrateIcm20948() {
  if (!icmPresent) return;

  Vec3f sumAcc = makeVec3(0.0f, 0.0f, 0.0f);
  Vec3f sumGyr = makeVec3(0.0f, 0.0f, 0.0f);
  int valid = 0;
  uint32_t start = millis();

  Serial.println("Calibrando piso y gyro ICM-20948, no mover...");
  while (valid < ICM_FLOOR_CAL_SAMPLES && !elapsed(millis(), start, 6000)) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      sumAcc = addVec3(sumAcc,
                       makeVec3(myICM.accX() / 1000.0f,
                                myICM.accY() / 1000.0f,
                                myICM.accZ() / 1000.0f));
      sumGyr = addVec3(sumGyr,
                       makeVec3(myICM.gyrX(), myICM.gyrY(), myICM.gyrZ()));
      valid++;
    }
    delay(2);
  }

  if (valid == 0) {
    Serial.println("WARN: calibracion ICM sin muestras validas.");
    return;
  }

  Vec3f meanAcc = scaleVec3(sumAcc, 1.0f / valid);
  Vec3f meanGyr = scaleVec3(sumGyr, 1.0f / valid);
  float magnitude = normVec3(meanAcc);

  // Gravedad propia de esta unidad, para la deteccion de reposo.
  if (magnitude >= FLOOR_CAL_MIN_MAG_G && magnitude <= FLOOR_CAL_MAX_MAG_G) {
    icmGravityRef_g = magnitude;
    icmGravityRefValid = true;
  }

  icmGyroBiasSensorX_dps = meanGyr.x;
  icmGyroBiasSensorY_dps = meanGyr.y;
  icmGyroBiasSensorZ_dps = meanGyr.z;

  if (magnitude >= FLOOR_CAL_MIN_MAG_G &&
      magnitude <= FLOOR_CAL_MAX_MAG_G && normalizeVec3(meanAcc)) {
    Vec3f previousForward = icmAlignment.forwardSensor;
    bool hadHeading = icmAlignment.headingValid;
    icmAlignment.upSensor = meanAcc;
    icmAlignment.floorValid = true;
    if (hadHeading) icmAlignment.forwardSensor = previousForward;
    rebuildAlignmentBasis(icmAlignment,
                          ICM_BODY_X_SENSOR_AXIS, ICM_BODY_X_SIGN);
    clearHeadingAccumulator(icmAlignment);
    imuCalibrationDirty = true;
  } else {
    Serial.print("WARN: piso ICM rechazado, |g|=");
    Serial.println(magnitude, 4);
  }

  float ax, ay, az;
  icmSensorToTrain(sumAcc.x / valid, sumAcc.y / valid, sumAcc.z / valid,
                   ax, ay, az);
  icmRoll_deg = atan2f(ay, az) * 180.0f / PI;
  icmPitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  float rollRad = icmRoll_deg * PI / 180.0f;
  float pitchRad = icmPitch_deg * PI / 180.0f;
  icmBodyZeroX_mps2 = ax * GRAVITY_MPS2 + GRAVITY_MPS2 * sinf(pitchRad);
  icmBodyZeroY_mps2 = ay * GRAVITY_MPS2 -
      GRAVITY_MPS2 * sinf(rollRad) * cosf(pitchRad);
  attitudeInitialized = true;
  lastIcmSampleUs = micros();

  Serial.print("ICM muestras validas="); Serial.println(valid);
  Serial.print("Gyro bias sensor dps: ");
  Serial.print(icmGyroBiasSensorX_dps, 4); Serial.print(",");
  Serial.print(icmGyroBiasSensorY_dps, 4); Serial.print(",");
  Serial.println(icmGyroBiasSensorZ_dps, 4);
  Serial.print("Piso ICM |g|="); Serial.print(magnitude, 4);
  Serial.print(" Roll/Pitch inicial: ");
  Serial.print(icmRoll_deg, 2); Serial.print(" / ");
  Serial.println(icmPitch_deg, 2);
}

bool updateIcmSample(uint32_t nowMs) {
  if (!icmPresent || !myICM.dataReady()) return false;

  myICM.getAGMT();
  icmRawX_mg = myICM.accX();
  icmRawY_mg = myICM.accY();
  icmRawZ_mg = myICM.accZ();
  icmRawGyrX_dps = myICM.gyrX();
  icmRawGyrY_dps = myICM.gyrY();
  icmRawGyrZ_dps = myICM.gyrZ();

  icmSensorToTrain(icmRawX_mg / 1000.0f,
                   icmRawY_mg / 1000.0f,
                   icmRawZ_mg / 1000.0f,
                   icmBodyAx_g, icmBodyAy_g, icmBodyAz_g);
  icmSensorToTrain(icmRawGyrX_dps - icmGyroBiasSensorX_dps,
                   icmRawGyrY_dps - icmGyroBiasSensorY_dps,
                   icmRawGyrZ_dps - icmGyroBiasSensorZ_dps,
                   icmBodyGx_dps, icmBodyGy_dps, icmBodyGz_dps);

  icmMag_g = sqrtf(icmBodyAx_g * icmBodyAx_g +
                   icmBodyAy_g * icmBodyAy_g +
                   icmBodyAz_g * icmBodyAz_g);

  uint32_t nowUs = micros();
  float dt = (lastIcmSampleUs == 0) ? 0.0f :
             (uint32_t)(nowUs - lastIcmSampleUs) / 1000000.0f;
  lastIcmSampleUs = nowUs;
  if (dt <= 0.0f || dt > 0.20f) return true;

  float rollAcc = atan2f(icmBodyAy_g, icmBodyAz_g) * 180.0f / PI;
  float pitchAcc = atan2f(-icmBodyAx_g,
                         sqrtf(icmBodyAy_g * icmBodyAy_g +
                               icmBodyAz_g * icmBodyAz_g)) * 180.0f / PI;

  if (!attitudeInitialized) {
    icmRoll_deg = rollAcc;
    icmPitch_deg = pitchAcc;
    attitudeInitialized = true;
  } else {
    float alphaAtt = ATTITUDE_TAU_S / (ATTITUDE_TAU_S + dt);
    icmRoll_deg = alphaAtt * (icmRoll_deg + icmBodyGx_dps * dt) +
                  (1.0f - alphaAtt) * rollAcc;
    icmPitch_deg = alphaAtt * (icmPitch_deg + icmBodyGy_dps * dt) +
                   (1.0f - alphaAtt) * pitchAcc;
  }

  if (northReferenceValid) {
    bodyHeadingTrue_deg = normalizeHeadingDeg(
        bodyHeadingTrue_deg + GYRO_TO_HEADING_SIGN * icmBodyGz_dps * dt);
  }

  float rollRad = icmRoll_deg * PI / 180.0f;
  float pitchRad = icmPitch_deg * PI / 180.0f;
  icmBodyAccelXRaw_mps2 = icmBodyAx_g * GRAVITY_MPS2 +
                          GRAVITY_MPS2 * sinf(pitchRad) -
                          icmBodyZeroX_mps2;
  icmBodyAccelYRaw_mps2 = icmBodyAy_g * GRAVITY_MPS2 -
                          GRAVITY_MPS2 * sinf(rollRad) * cosf(pitchRad) -
                          icmBodyZeroY_mps2;

  float rc = 1.0f / (2.0f * PI * NAV_ACCEL_LPF_HZ);
  float alphaAcc = dt / (rc + dt);
  icmBodyAccelX_mps2 += alphaAcc *
      (icmBodyAccelXRaw_mps2 - icmBodyAccelX_mps2);
  icmBodyAccelY_mps2 += alphaAcc *
      (icmBodyAccelYRaw_mps2 - icmBodyAccelY_mps2);
  if (fabsf(icmBodyAccelX_mps2) < ACC_DEADBAND_MPS2)
    icmBodyAccelX_mps2 = 0.0f;
  if (fabsf(icmBodyAccelY_mps2) < ACC_DEADBAND_MPS2)
    icmBodyAccelY_mps2 = 0.0f;

  icmAccelE_mps2 = 0.0f;
  icmAccelN_mps2 = 0.0f;
  if (northReferenceValid && icmAlignment.headingValid) {
    float h = bodyHeadingTrue_deg * PI / 180.0f;
    // +X del nodo: [sin(h), cos(h)] en EN; +Y derecha: [cos(h),-sin(h)].
    icmAccelE_mps2 = icmBodyAccelX_mps2 * sinf(h) +
                     icmBodyAccelY_mps2 * cosf(h);
    icmAccelN_mps2 = icmBodyAccelX_mps2 * cosf(h) -
                     icmBodyAccelY_mps2 * sinf(h);
    if (fusionInitialized) fusionPredict(icmAccelE_mps2, icmAccelN_mps2, dt);
  }

  (void)nowMs;
  return true;
}

// ============================================================================
// GNSS Y ACTUALIZACIONES DEL FILTRO GENERAL
// DIRECCION, GNSS Y ACTUALIZACIONES DEL FILTRO
// ============================================================================
void pollGnss(uint32_t now) {
  if (!gpsPresent) {
    gpsQualityCurrent = false;
    gpsUsable = false;
    return;
  }

  if (myGPS.getPVT()) {
    lastPvtMs = now;

    gpsFixType = myGPS.getFixType();
    gpsGnssFixOk = myGPS.getGnssFixOk();
    gpsInvalidLlh = myGPS.getInvalidLlh();
    gpsLat = myGPS.getLatitude() / 1e7;
    gpsLon = myGPS.getLongitude() / 1e7;
    gpsGroundSpeed_mps = myGPS.getGroundSpeed() / 1000.0f;
    gpsVelN_mps = myGPS.getNedNorthVel() / 1000.0f;
    gpsVelE_mps = myGPS.getNedEastVel() / 1000.0f;
    gpsSpeed_mps = sqrtf(gpsVelE_mps * gpsVelE_mps +
                         gpsVelN_mps * gpsVelN_mps);
    if (!isfinite(gpsSpeed_mps)) gpsSpeed_mps = gpsGroundSpeed_mps;
    gpsHeading_deg = normalizeHeadingDeg(myGPS.getHeading() / 100000.0f);
    gpsHeadingAcc_deg = myGPS.getHeadingAccEst() / 100000.0f;
    gpsSats = (int)myGPS.getSIV();
    gpsHAcc_mm = myGPS.getHorizontalAccEst();
    gpsSAcc_mmps = myGPS.getSpeedAccEst();
    gpsPDOP_centi = myGPS.getPDOP();
    gpsCarrierSolution = myGPS.getCarrierSolutionType();

    gpsYear = myGPS.getYear();
    gpsMonth = myGPS.getMonth();
    gpsDay = myGPS.getDay();
    gpsHour = myGPS.getHour();
    gpsMinute = myGPS.getMinute();
    gpsSecond = myGPS.getSecond();
    gpsTimeValid = (gpsYear >= 2020 && gpsMonth >= 1 && gpsMonth <= 12 &&
                    gpsDay >= 1 && gpsDay <= 31 && gpsHour <= 23 &&
                    gpsMinute <= 59 && gpsSecond <= 60);

    bool fixOK = gpsFixType >= 3 && gpsGnssFixOk;
    bool coordinateOK = coordinatesAreValid(gpsLat, gpsLon) && !gpsInvalidLlh;
    bool satellitesOK = gpsSats >= MIN_GNSS_SATS;
    bool hAccOK = gpsHAcc_mm <= HACC_LIMIT_MM;
    bool sAccOK = gpsSAcc_mmps <= SACC_LIMIT_MMPS;
    bool pdopOK = gpsPDOP_centi <= PDOP_LIMIT_CENTI;
    bool rtkOK = !REQUIRE_RTK_FOR_POSITION || gpsCarrierSolution > 0;

    gpsFixRaw = fixOK;
    gpsQualityCurrent = fixOK && coordinateOK && satellitesOK && hAccOK &&
                        sAccOK && pdopOK && rtkOK;

    if (gpsQualityCurrent) {
      lastGpsValidMs = now;
      referenceRecoveredFromSd = false;
      recoveredReferenceMs = 0;
      refLat = gpsLat;
      refLon = gpsLon;
      hasPositionReference = true;

      if (!localOriginValid) setLocalOrigin(gpsLat, gpsLon);
      latLonToLocal(gpsLat, gpsLon, gpsLocalE_m, gpsLocalN_m);

      if (!fusionInitialized) {
        fusionInitialize(gpsLocalE_m, gpsLocalN_m,
                         gpsVelE_mps, gpsVelN_mps, false);
      } else {
        float sigmaPos = fmaxf(gpsHAcc_mm / 1000.0f, 0.30f);
        if (gpsCarrierSolution == 2) sigmaPos = fmaxf(sigmaPos, 0.05f);
        else if (gpsCarrierSolution == 1) sigmaPos = fmaxf(sigmaPos, 0.20f);
        float sigmaSpeed = fmaxf(gpsSAcc_mmps / 1000.0f, 0.05f);
        fusionUpdatePosition(gpsLocalE_m, gpsLocalN_m, sigmaPos);
        fusionUpdateVelocity(gpsVelE_mps, gpsVelN_mps, sigmaSpeed);
      }

      updateLegacyNsDirection(gpsVelN_mps);
      updateHeadingCalibrationFromGps(now);
    } else {
      headingCalPrevGpsValid = false;
    }
  }

  gpsUsable = gpsQualityCurrent &&
              !elapsed(now, lastPvtMs, GPS_RECENT_MS) &&
              !elapsed(now, lastGpsValidMs, GPS_RECENT_MS);

  if (gpsUsable && !previousGpsUsable) lastPosSavedFromCurrentFix = false;
  if (gpsUsable) rebaseLocalFrameIfNeeded();
}

// ============================================================================
// ZUPT, AJUSTE LENTO Y SALUD DE LA IMU
// ZUPT, AJUSTE LENTO Y SALUD DE LA IMU
// ============================================================================
void applyStillnessCorrection(uint32_t now) {
  // La banda se centra en la gravedad que mide cada sensor. Si la calibracion
  // de piso no llego a completarse, se recurre a la banda absoluta.
  float icmLow  = icmGravityRefValid ? icmGravityRef_g - STILL_MAG_TOL_G : STILL_MAG_MIN_G;
  float icmHigh = icmGravityRefValid ? icmGravityRef_g + STILL_MAG_TOL_G : STILL_MAG_MAX_G;
  float mpuLow  = mpuGravityRefValid ? mpuGravityRef_g - STILL_MAG_TOL_G : STILL_MAG_MIN_G;
  float mpuHigh = mpuGravityRefValid ? mpuGravityRef_g + STILL_MAG_TOL_G : STILL_MAG_MAX_G;

  bool icmStill = icmPresent &&
                  icmMag_g >= icmLow && icmMag_g <= icmHigh &&
                  fabsf(icmBodyGx_dps) <= STILL_GYRO_MAX_DPS &&
                  fabsf(icmBodyGy_dps) <= STILL_GYRO_MAX_DPS &&
                  fabsf(icmBodyGz_dps) <= STILL_GYRO_MAX_DPS;
  bool mpuStill = mpuPresent &&
                  mpuMag_g >= mpuLow && mpuMag_g <= mpuHigh &&
                  vibMagLin_g <= STILL_VIB_MAX_G;
  bool externalStill = gpsUsable && gpsSpeed_mps < GPS_STILL_MPS;

#if ENABLE_WHEEL_ODOMETRY
  externalStill = externalStill || wheelSpeed_mps < GPS_STILL_MPS;
#endif

  stillDetected = externalStill && icmStill && mpuStill;
  zuptApplied = false;

  if (stillDetected) {
    if (stillStartMs == 0) stillStartMs = now;
    if (elapsed(now, stillStartMs, STILL_HOLD_MS)) {
      zuptApplied = true;
      if (fusionInitialized) fusionUpdateVelocity(0.0f, 0.0f, 0.03f);

      icmGyroBiasSensorX_dps += BIAS_ADAPT_ALPHA *
          (icmRawGyrX_dps - icmGyroBiasSensorX_dps);
      icmGyroBiasSensorY_dps += BIAS_ADAPT_ALPHA *
          (icmRawGyrY_dps - icmGyroBiasSensorY_dps);
      icmGyroBiasSensorZ_dps += BIAS_ADAPT_ALPHA *
          (icmRawGyrZ_dps - icmGyroBiasSensorZ_dps);
      icmBodyZeroX_mps2 += BIAS_ADAPT_ALPHA * icmBodyAccelXRaw_mps2;
      icmBodyZeroY_mps2 += BIAS_ADAPT_ALPHA * icmBodyAccelYRaw_mps2;
    }
  } else {
    stillStartMs = 0;
  }
}

void evaluateImuHealth(uint32_t now) {
  if (!icmPresent) {
    imuHealth = IMU_SENSOR_FAIL;
    return;
  }

  bool speedInnovationBad = gpsUsable &&
                            lastSpeedInnovation_mps >
                            IMU_SPEED_INNOV_LIMIT_MPS;
  if (speedInnovationBad) {
    if (icmDriftStartMs == 0) icmDriftStartMs = now;
  } else {
    icmDriftStartMs = 0;
  }

  float accelHorizontal = sqrtf(icmBodyAccelX_mps2 * icmBodyAccelX_mps2 +
                                icmBodyAccelY_mps2 * icmBodyAccelY_mps2);
  bool stillDisagreement = gpsUsable && gpsSpeed_mps < GPS_STILL_MPS &&
                           accelHorizontal > IMU_STILL_ACCEL_LIMIT_MPS2;
  if (stillDisagreement) {
    if (imuDisagreeStartMs == 0) imuDisagreeStartMs = now;
  } else {
    imuDisagreeStartMs = 0;
  }

  if (icmDriftStartMs != 0 && elapsed(now, icmDriftStartMs, IMU_ERROR_HOLD_MS)) {
    imuHealth = IMU_ICM_DRIFT;
  } else if (imuDisagreeStartMs != 0 &&
             elapsed(now, imuDisagreeStartMs, IMU_ERROR_HOLD_MS)) {
    imuHealth = IMU_DISAGREE;
  } else {
    imuHealth = fusionInitialized ? IMU_OK : IMU_UNKNOWN;
  }
}

#if ENABLE_WHEEL_ODOMETRY
void wheelPulseIsr() {
  wheelPulseCounter++;
}

void updateWheelOdometry(uint32_t now) {
  if (!elapsed(now, lastWheelUpdateMs, WHEEL_UPDATE_MS)) return;
  uint32_t dtMs = now - lastWheelUpdateMs;
  lastWheelUpdateMs = now;

  noInterrupts();
  uint32_t snapshot = wheelPulseCounter;
  interrupts();
  uint32_t delta = snapshot - lastWheelPulseSnapshot;
  lastWheelPulseSnapshot = snapshot;

  wheelSpeed_mps = delta * WHEEL_METERS_PER_PULSE / (dtMs / 1000.0f);
  if (!fusionInitialized || !northReferenceValid) return;

  float signedBodySpeed = motionDirectionSign * wheelSpeed_mps;
  float h = bodyHeadingTrue_deg * PI / 180.0f;
  float vE = signedBodySpeed * sinf(h);
  float vN = signedBodySpeed * cosf(h);
  fusionUpdateVelocity(vE, vN, WHEEL_SPEED_SIGMA_MPS);
}
#endif

// ============================================================================
// SELECCION DE FUENTE FINAL
// ============================================================================
void updateFinalPosition(uint32_t now) {
  if (fusionInitialized && localOriginValid) {
    localToLatLon(fusionEast.p, fusionNorth.p, fusedLat, fusedLon);
  } else {
    fusedLat = fusedLon = 0.0;
  }

  if (gpsUsable && fusionInitialized) {
    txSource = SRC_GNSS_FUSED;
  } else if (fusionInitialized && icmPresent &&
             !elapsed(now, lastGpsValidMs, LAST_VALID_MAX_AGE_MS)) {
    txSource = SRC_GENERAL_DR;
  } else {
    txSource = SRC_NONE;
  }

  if (txSource != SRC_NONE && coordinatesAreValid(fusedLat, fusedLon)) {
    txLat = fusedLat;
    txLon = fusedLon;
    txVelE_mps = fusionEast.v;
    txVelN_mps = fusionNorth.v;
    txSpeedKmh = sqrtf(txVelE_mps * txVelE_mps +
                       txVelN_mps * txVelN_mps) * 3.6f;

    if (gpsUsable && gpsSpeed_mps >= HEADING_OUTPUT_MIN_SPEED_MPS &&
        gpsHeadingAcc_deg <= HEADING_CAL_MAX_HEADACC_DEG) {
      txHeading_deg = gpsHeading_deg;
    } else {
      txHeading_deg = bodyHeadingForMotion();
      if (txHeading_deg < 0.0f && txSpeedKmh < 0.8f && northReferenceValid)
        txHeading_deg = bodyHeadingTrue_deg;
    }
    updateLegacyNsDirection(txVelN_mps);
  } else {
    txLat = 0.0;
    txLon = 0.0;
    txSpeedKmh = 0.0f;
    txVelE_mps = txVelN_mps = 0.0f;
    txHeading_deg = -1.0f;
  }

  if (fabsf(txSpeedKmh) < 0.8f) txSpeedKmh = 0.0f;
  digitalWrite(LED_GPSFAIL, gpsUsable ? LOW : HIGH);
}

// ============================================================================
// SD: ALMACENAMIENTO PERMANENTE DE VIBRACION
// SD: ALMACENAMIENTO PERMANENTE DE VIBRACION
// ============================================================================
void closeVibFile(bool flushFirst = true) {
  if (!vibFile) return;
  if (flushFirst && sdOK) vibFile.flush();
  vibFile.close();
}

bool openVibFileForDate(uint16_t year, uint8_t month, uint8_t day) {
  if (!sdOK) return false;

  snprintf(vibFilename, sizeof(vibFilename), "VIB_%04u%02u%02u.CSV",
           year, month, day);
  bool existed = SD.exists(vibFilename);
  vibFile = SD.open(vibFilename, FILE_WRITE);
  if (!vibFile) {
    noteSdFailure("no se pudo abrir archivo VIB");
    return false;
  }

  if (!existed) {
    vibFile.clearWriteError();
    vibFile.print("# firmware=");
    vibFile.println(FW_VERSION);
    vibFile.println(
      "date,time,millis,axBodyLin_g,ayBodyLin_g,azUpLin_g,magLin_g,"
      "gpsUsable,sats,speedKmh,headingTrue_deg,legacyDirNS,source,imuHealth,"
      "localE_m,localN_m,velE_mps,velN_mps,roll_deg,pitch_deg,"
      "aE_mps2,aN_mps2,rtk,pdop,sAcc_mps,icmFloorCal,icmAxisCal,"
      "mpuFloorCal,mpuAxisCal,northValid,bodyHeading_deg,icmCalQ,mpuCalQ,"
      "lat,lon,hAcc_m,secsSinGnss,still,zupt,icmMag_g,mpuMag_g"
    );
    vibFile.flush();

    if (vibFile.getWriteError()) {
      closeVibFile(false);
      noteSdFailure("fallo al crear encabezado VIB");
      return false;
    }
  }

  lastVibYear = year;
  lastVibMonth = month;
  lastVibDay = day;
  noteSdSuccess();
  return true;
}

void ensureVibFileReady() {
  if (!sdOK) return;

  uint16_t year = gpsTimeValid ? gpsYear : 0;
  uint8_t month = gpsTimeValid ? gpsMonth : 0;
  uint8_t day = gpsTimeValid ? gpsDay : 0;

  if (!vibFile || year != lastVibYear || month != lastVibMonth || day != lastVibDay) {
    closeVibFile();
    openVibFileForDate(year, month, day);
  }
}

void writeVibrationLog(uint32_t now) {
  if (!sdOK || !mpuPresent || !elapsed(now, lastVibLogMs, VIB_LOG_INTERVAL_MS)) return;
  lastVibLogMs = now;

  ensureVibFileReady();
  if (!sdOK || !vibFile) return;

  char dateStr[11];
  char timeStr[9];
  formatDateTime(dateStr, sizeof(dateStr), timeStr, sizeof(timeStr));

  vibFile.clearWriteError();
  vibFile.print(dateStr); vibFile.print(',');
  vibFile.print(timeStr); vibFile.print(',');
  vibFile.print(now); vibFile.print(',');
  vibFile.print(vibAxLin_g, 4); vibFile.print(',');
  vibFile.print(vibAyLin_g, 4); vibFile.print(',');
  vibFile.print(vibAzLin_g, 4); vibFile.print(',');
  vibFile.print(vibMagLin_g, 4); vibFile.print(',');
  vibFile.print(gpsUsable ? 1 : 0); vibFile.print(',');
  vibFile.print(gpsSats); vibFile.print(',');
  vibFile.print(txSpeedKmh, 2); vibFile.print(',');
  vibFile.print(txHeading_deg, 2); vibFile.print(',');
  vibFile.print(dirNS); vibFile.print(',');
  vibFile.print((int)txSource); vibFile.print(',');
  vibFile.print((int)imuHealth); vibFile.print(',');
  vibFile.print(fusionInitialized ? fusionEast.p : 0.0f, 2); vibFile.print(',');
  vibFile.print(fusionInitialized ? fusionNorth.p : 0.0f, 2); vibFile.print(',');
  vibFile.print(fusionInitialized ? fusionEast.v : 0.0f, 3); vibFile.print(',');
  vibFile.print(fusionInitialized ? fusionNorth.v : 0.0f, 3); vibFile.print(',');
  vibFile.print(icmRoll_deg, 2); vibFile.print(',');
  vibFile.print(icmPitch_deg, 2); vibFile.print(',');
  vibFile.print(icmAccelE_mps2, 4); vibFile.print(',');
  vibFile.print(icmAccelN_mps2, 4); vibFile.print(',');
  vibFile.print(gpsCarrierSolution); vibFile.print(',');
  vibFile.print(gpsPDOP_centi / 100.0f, 2); vibFile.print(',');
  vibFile.print(gpsSAcc_mmps / 1000.0f, 3); vibFile.print(',');
  vibFile.print(icmAlignment.floorValid ? 1 : 0); vibFile.print(',');
  vibFile.print(icmAlignment.headingValid ? 1 : 0); vibFile.print(',');
  vibFile.print(mpuAlignment.floorValid ? 1 : 0); vibFile.print(',');
  vibFile.print(mpuAlignment.headingValid ? 1 : 0); vibFile.print(',');
  vibFile.print(northReferenceValid ? 1 : 0); vibFile.print(',');
  vibFile.print(bodyHeadingTrue_deg, 2); vibFile.print(',');
  vibFile.print(icmAlignment.headingQuality, 3); vibFile.print(',');
  vibFile.print(mpuAlignment.headingQuality, 3); vibFile.print(',');

  // Posicion GNSS cruda, calidad y edad de la referencia: P-01 mide dispersion
  // sobre estas columnas y P-09 las exige entre los datos por registrar.
  vibFile.print(gpsLat, 7); vibFile.print(',');
  vibFile.print(gpsLon, 7); vibFile.print(',');
  vibFile.print(gpsHAcc_mm / 1000.0f, 2); vibFile.print(',');
  vibFile.print(hasPositionReference ? (now - lastGpsValidMs) / 1000.0f : -1.0f, 1);
  vibFile.print(',');

  // Banderas de reposo y ZUPT: evidencia directa de P-06.
  vibFile.print(stillDetected ? 1 : 0); vibFile.print(',');
  vibFile.print(zuptApplied ? 1 : 0); vibFile.print(',');
  vibFile.print(icmMag_g, 4); vibFile.print(',');
  vibFile.println(mpuMag_g, 4);

  if (elapsed(now, lastVibFlushMs, FILE_FLUSH_MS)) {
    vibFile.flush();
    lastVibFlushMs = now;
  }

  if (vibFile.getWriteError()) {
    noteSdFailure("fallo al escribir/flush archivo VIB");
    return;
  }

  noteSdSuccess();
  pulseSdLed();
}

// ============================================================================
// SD: ULTIMA POSICION GNSS VALIDA
// ============================================================================
bool parseLastPositionLine(const String &line,
                           double &lat, double &lon,
                           uint32_t &hAcc, uint8_t &fixType,
                           float &headingDeg,
                           uint16_t &year, uint8_t &month, uint8_t &day,
                           uint8_t &hour, uint8_t &minute, uint8_t &second,
                           float &velE, float &velN) {
  String value[14];
  int count = 0;
  int start = 0;
  while (count < 14 && start <= (int)line.length()) {
    int comma = line.indexOf(',', start);
    if (comma < 0) comma = line.length();
    value[count] = line.substring(start, comma);
    value[count].trim();
    count++;
    if (comma >= (int)line.length()) break;
    start = comma + 1;
  }

  if (count >= 14 && value[0] == "G11") {
    lat = atof(value[1].c_str());
    lon = atof(value[2].c_str());
    hAcc = (uint32_t)strtoul(value[3].c_str(), nullptr, 10);
    fixType = (uint8_t)value[4].toInt();
    headingDeg = atof(value[5].c_str());
    year = (uint16_t)value[6].toInt();
    month = (uint8_t)value[7].toInt();
    day = (uint8_t)value[8].toInt();
    hour = (uint8_t)value[9].toInt();
    minute = (uint8_t)value[10].toInt();
    second = (uint8_t)value[11].toInt();
    velE = atof(value[12].c_str());
    velN = atof(value[13].c_str());
  } else if (count >= 11) {
    // Compatibilidad de lectura con LASTPOS de R8-R10.
    lat = atof(value[0].c_str());
    lon = atof(value[1].c_str());
    hAcc = (uint32_t)strtoul(value[2].c_str(), nullptr, 10);
    fixType = (uint8_t)value[3].toInt();
    headingDeg = value[4].toInt() == 1 ? 180.0f : 0.0f;
    year = (uint16_t)value[5].toInt();
    month = (uint8_t)value[6].toInt();
    day = (uint8_t)value[7].toInt();
    hour = (uint8_t)value[8].toInt();
    minute = (uint8_t)value[9].toInt();
    second = (uint8_t)value[10].toInt();
    velE = velN = 0.0f;
  } else {
    return false;
  }

  return coordinatesAreValid(lat, lon) && fixType >= 2 &&
         isfinite(headingDeg) && isfinite(velE) && isfinite(velN);
}

bool readLastPositionFile(const char *filename) {
  if (!sdOK || !SD.exists(filename)) return false;

  File file = SD.open(filename, FILE_READ);
  if (!file) {
    noteSdFailure("no se pudo abrir LASTPOS para lectura");
    return false;
  }
  String line = file.readStringUntil('\n');
  line.trim();
  file.close();

  double lat = 0.0, lon = 0.0;
  uint32_t hAcc = 0;
  uint8_t fixType = 0;
  float headingDeg = 0.0f;
  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
  float velE = 0.0f, velN = 0.0f;

  if (!parseLastPositionLine(line, lat, lon, hAcc, fixType, headingDeg,
                             year, month, day, hour, minute, second,
                             velE, velN)) {
    Serial.print("WARN: contenido invalido en ");
    Serial.println(filename);
    return false;
  }

  setLocalOrigin(lat, lon);
  refLat = lat;
  refLon = lon;
  hasPositionReference = true;
  referenceRecoveredFromSd = true;
  recoveredReferenceMs = millis();
  lastGpsValidMs = recoveredReferenceMs;
  fusionInitialize(0.0f, 0.0f, velE, velN, true);

  if (!northReferenceValid && icmAlignment.headingValid) {
    bodyHeadingTrue_deg = normalizeHeadingDeg(headingDeg);
    northReferenceValid = true;
    northReferenceQuality = 0.20f;
  }

  noteSdSuccess();
  Serial.print("Ultima posicion general recuperada de ");
  Serial.print(filename);
  Serial.print(": ");
  Serial.print(lat, 6);
  Serial.print(", ");
  Serial.println(lon, 6);
  return true;
}

void loadLastPositionFromSd() {
  if (!sdOK) return;
  if (readLastPositionFile(LASTPOS_FILE)) return;

  if (readLastPositionFile(LASTPOS_TEMP_FILE)) {
    if (SD.exists(LASTPOS_FILE) && !SD.remove(LASTPOS_FILE)) {
      noteSdFailure("no se pudo retirar LASTPOS anterior al recuperar TMP");
      return;
    }
    if (!SD.rename(LASTPOS_TEMP_FILE, LASTPOS_FILE)) {
      noteSdFailure("no se pudo promover LASTPOS.TMP");
      return;
    }
    noteSdSuccess();
  }
}

bool saveLastPositionToSd() {
  if (!sdOK || !gpsUsable || !coordinatesAreValid(gpsLat, gpsLon)) return false;

  if (SD.exists(LASTPOS_TEMP_FILE) && !SD.remove(LASTPOS_TEMP_FILE)) {
    noteSdFailure("no se pudo limpiar LASTPOS.TMP");
    return false;
  }

  File file = SD.open(LASTPOS_TEMP_FILE, FILE_WRITE);
  if (!file) {
    noteSdFailure("no se pudo crear LASTPOS.TMP");
    return false;
  }

  file.clearWriteError();
  // Formato R11: G11,lat,lon,hAcc,fix,rumbo,fecha,hora,velE,velN.
  file.print("G11,");
  file.print(gpsLat, 7); file.print(',');
  file.print(gpsLon, 7); file.print(',');
  file.print(gpsHAcc_mm); file.print(',');
  file.print(gpsFixType); file.print(',');
  file.print(gpsHeading_deg, 2); file.print(',');
  file.print(gpsTimeValid ? gpsYear : 0); file.print(',');
  file.print(gpsTimeValid ? gpsMonth : 0); file.print(',');
  file.print(gpsTimeValid ? gpsDay : 0); file.print(',');
  file.print(gpsTimeValid ? gpsHour : 0); file.print(',');
  file.print(gpsTimeValid ? gpsMinute : 0); file.print(',');
  file.print(gpsTimeValid ? gpsSecond : 0); file.print(',');
  file.print(gpsVelE_mps, 3); file.print(',');
  file.println(gpsVelN_mps, 3);
  file.flush();

  bool writeFailed = file.getWriteError() != 0;
  file.close();
  if (writeFailed) {
    noteSdFailure("fallo al escribir LASTPOS.TMP");
    return false;
  }

  if (SD.exists(LASTPOS_FILE) && !SD.remove(LASTPOS_FILE)) {
    noteSdFailure("no se pudo retirar LASTPOS.TXT anterior");
    return false;
  }
  if (!SD.rename(LASTPOS_TEMP_FILE, LASTPOS_FILE)) {
    noteSdFailure("no se pudo renombrar LASTPOS.TMP");
    return false;
  }

  noteSdSuccess();
  pulseSdLed();
  return true;
}

void saveLastPositionIfDue(uint32_t now) {
  if (!gpsUsable) return;
  if (lastPosSavedFromCurrentFix &&
      !elapsed(now, lastPosSaveMs, LASTPOS_SAVE_INTERVAL_MS)) return;

  if (saveLastPositionToSd()) {
    lastPosSaveMs = now;
    lastPosSavedFromCurrentFix = true;
  }
}

// ============================================================================
// SD: CONTROL DE OCUPACION Y BORRADO DE VIBRACIONES ANTIGUAS
// SD: CONTROL DE OCUPACION Y BORRADO DE VIBRACIONES ANTIGUAS
// ============================================================================
bool readSdUsage(uint64_t &usedBytes, uint64_t &totalBytes, uint8_t &percent) {
  usedBytes = 0;
  totalBytes = 0;
  percent = 0;
  if (!sdOK) return false;

  FSInfo info;
  if (!SDFS.info(info) || info.totalBytes == 0) return false;

  usedBytes = info.usedBytes;
  totalBytes = info.totalBytes;
  percent = (uint8_t)((usedBytes * 100ULL) / totalBytes);
  return true;
}

String normalizedBaseName(const char *rawName) {
  String name = rawName ? String(rawName) : String("");
  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  return name;
}

bool isVibrationCsvName(const String &name) {
  return name.startsWith("VIB_") && name.endsWith(".CSV");
}

bool deleteOldestVibrationFile() {
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    return false;
  }

  String currentName = normalizedBaseName(vibFilename);
  String oldestName = "";

  File entry = root.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = normalizedBaseName(entry.name());
      if (isVibrationCsvName(name) && name != currentName) {
        // El formato VIB_YYYYMMDD.CSV permite ordenar por nombre/fecha.
        if (oldestName.length() == 0 || name.compareTo(oldestName) < 0) oldestName = name;
      }
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();

  if (oldestName.length() == 0) return false;

  bool removed = SD.remove(oldestName.c_str());
  if (removed) {
    noteSdSuccess();
    Serial.print("SD >= 80%: eliminado archivo antiguo ");
    Serial.println(oldestName);
    pulseSdLed();
  } else {
    noteSdFailure("no se pudo eliminar VIB antiguo");
  }
  return removed;
}

void manageVibrationStorage(uint32_t now, bool forceCheck = false) {
  if (!sdOK) return;
  if (!forceCheck && !elapsed(now, lastSdUsageCheckMs, SD_USAGE_CHECK_MS)) return;
  lastSdUsageCheckMs = now;

  uint64_t used = 0, total = 0;
  uint8_t percent = 0;
  if (!readSdUsage(used, total, percent)) {
    noteSdFailure("no se pudo consultar la ocupacion de la SD");
    return;
  }
  noteSdSuccess();

  Serial.print("Uso SD: ");
  Serial.print(percent);
  Serial.println("%");

  if (percent < SD_CLEAN_TRIGGER_PERCENT) return;

  if (vibFile) vibFile.flush();

  // Se eliminan solo VIB_*.CSV antiguos. Nunca se borran el archivo diario
  // activo, PENDING.TXT ni LASTPOS.TXT. Se limpia hasta quedar <= 75%.
  while (percent > SD_CLEAN_TARGET_PERCENT) {
    if (!deleteOldestVibrationFile()) {
      Serial.println("WARN: no quedan archivos VIB antiguos eliminables.");
      break;
    }
    if (!readSdUsage(used, total, percent)) {
      noteSdFailure("fallo al recalcular uso despues de limpiar");
      break;
    }
  }
}

// ============================================================================
// PENDIENTES LTE
// ============================================================================
void loadPendingFromSd() {
  if (!sdOK || havePending || !SD.exists(PENDING_FILE)) return;

  File file = SD.open(PENDING_FILE, FILE_READ);
  if (!file) {
    noteSdFailure("no se pudo abrir PENDING.TXT");
    return;
  }

  String recovered = file.readStringUntil('\n');
  recovered.trim();
  file.close();

  if (recovered.length() > 0) {
    pendingLine = recovered;
    havePending = true;
    pendingNeedsRefresh = true;
    lastPendingRefreshMs = 0;
    noteSdSuccess();
    Serial.println("Paquete pendiente recuperado; se actualizara antes del reintento si existe una posicion valida.");
  } else {
    Serial.println("WARN: PENDING.TXT estaba vacio.");
  }
}

bool persistPendingToSd() {
  if (!havePending || !sdOK) return false;

  if (SD.exists(PENDING_FILE) && !SD.remove(PENDING_FILE)) {
    noteSdFailure("no se pudo reemplazar PENDING.TXT");
    return false;
  }

  File file = SD.open(PENDING_FILE, FILE_WRITE);
  if (!file) {
    noteSdFailure("no se pudo crear PENDING.TXT");
    return false;
  }

  file.clearWriteError();
  file.println(pendingLine);
  file.flush();
  bool writeFailed = file.getWriteError() != 0;
  file.close();

  if (writeFailed) {
    noteSdFailure("fallo al escribir PENDING.TXT");
    return false;
  }

  noteSdSuccess();
  pulseSdLed();
  Serial.println("Paquete de posicion guardado temporalmente en PENDING.TXT.");
  return true;
}

void savePendingToSd(const String &line) {
  pendingLine = line;
  pendingLine.trim();
  havePending = pendingLine.length() > 0;
  if (!havePending) return;

  // Si la SD esta fuera de servicio, la copia permanece en RAM. Al recuperarse
  // la tarjeta, serviceSdManager() la persistira automaticamente.
  if (sdOK) persistPendingToSd();
}

void clearPendingAfterSuccess() {
  bool removedOrAbsent = true;

  if (sdOK) {
    if (SD.exists(PENDING_FILE)) {
      removedOrAbsent = SD.remove(PENDING_FILE);
      if (!removedOrAbsent) {
        pendingRemovalRequired = true;
        noteSdFailure("HTTP OK pero no se pudo eliminar PENDING.TXT");
      } else {
        noteSdSuccess();
      }
    } else {
      // Antes de aceptar que el archivo no existe, se confirma que el sistema
      // de archivos sigue montado. Asi se evita cargar un PENDING antiguo tras
      // una reinsercion si exists() fallo por desconexion.
      uint64_t used = 0, total = 0;
      uint8_t percent = 0;
      if (!readSdUsage(used, total, percent)) {
        removedOrAbsent = false;
        pendingRemovalRequired = true;
        noteSdFailure("no se pudo verificar ausencia de PENDING.TXT", true);
      }
    }
  } else {
    // Puede existir una copia antigua en la tarjeta retirada. Se eliminara al
    // recuperar la SD antes de intentar leer pendientes desde ella.
    pendingRemovalRequired = true;
  }

  havePending = false;
  pendingLine = "";
  pendingNeedsRefresh = false;
  lastPendingRefreshMs = millis();
  if (removedOrAbsent && sdOK) pulseSdLed();
  Serial.println("HTTP 2xx confirmado: pendiente eliminado de RAM.");
}

// ============================================================================
// SD: DETECCION DE FALLAS Y RECUPERACION AUTOMATICA
// ============================================================================
bool probeSdReadWrite() {
  if (!sdOK) return false;

  if (SD.exists(SD_PROBE_FILE) && !SD.remove(SD_PROBE_FILE)) return false;

  File probe = SD.open(SD_PROBE_FILE, FILE_WRITE);
  if (!probe) return false;
  probe.clearWriteError();
  probe.println("SD_OK");
  probe.flush();
  bool writeFailed = probe.getWriteError() != 0;
  probe.close();
  if (writeFailed) return false;

  probe = SD.open(SD_PROBE_FILE, FILE_READ);
  if (!probe) return false;
  String value = probe.readStringUntil('\n');
  value.trim();
  probe.close();

  bool contentOK = value == "SD_OK";
  if (!SD.remove(SD_PROBE_FILE)) return false;
  return contentOK;
}

bool initializeSdStorage(bool startup) {
  closeVibFile(false);
  if (!startup) SD.end(false);  // remonta desde cero tras retirar/reinsertar

  if (!SD.begin(SD_CS_PIN)) {
    setSdUnavailable("SD.begin fallo");
    return false;
  }

  // Se marca temporalmente como disponible para ejecutar la prueba de E/S.
  sdOK = true;
  if (!probeSdReadWrite()) {
    setSdUnavailable("prueba de lectura/escritura SD fallo");
    return false;
  }

  sdConsecutiveErrors = 0;
  sdLastError = "";
  lastSdHealthCheckMs = millis();
  lastSdRetryMs = millis();
  lastSdUsageCheckMs = millis();
  sdFaultLedState = false;
  ledSdOffMs = 0;
  digitalWrite(LED_SD, LOW);

  Serial.println(startup ? "SD OK al iniciar." : "SD recuperada correctamente.");

  // Si un envio ya fue confirmado mientras la SD estaba ausente, se elimina
  // primero cualquier PENDING antiguo para no retransmitirlo.
  if (pendingRemovalRequired) {
    if (!SD.exists(PENDING_FILE) || SD.remove(PENDING_FILE)) {
      pendingRemovalRequired = false;
      noteSdSuccess();
    } else {
      noteSdFailure("no se pudo limpiar PENDING antiguo al recuperar SD");
    }
  }

  // En el arranque se recuperan LASTPOS y PENDING. En una reinsercion durante
  // la marcha, LASTPOS solo se carga si no existe una referencia mejor en RAM.
  if (startup || (!hasPositionReference && !gpsUsable)) loadLastPositionFromSd();

  if (havePending) {
    persistPendingToSd();
  } else if (!pendingRemovalRequired) {
    loadPendingFromSd();
  }

  ensureVibFileReady();
  if (gpsUsable) {
    if (saveLastPositionToSd()) {
      lastPosSaveMs = millis();
      lastPosSavedFromCurrentFix = true;
    }
  }

  manageVibrationStorage(millis(), true);
  return sdOK;
}

void serviceSdManager(uint32_t now) {
  if (!sdOK) {
    if (!elapsed(now, lastSdRetryMs, SD_RETRY_INTERVAL_MS)) return;
    lastSdRetryMs = now;
    Serial.println("Reintentando inicializacion de microSD...");
    initializeSdStorage(false);
    return;
  }

  if (!elapsed(now, lastSdHealthCheckMs, SD_HEALTH_CHECK_MS)) return;
  lastSdHealthCheckMs = now;

  // Sonda de montaje barata: una consulta de directorio.
  //
  // Antes se usaba SDFS.info() aqui. En la tarjeta de 15 GB del prototipo esa
  // llamada recorre la FAT y tarda ~6.15 s, mas que el propio intervalo de
  // 5 s, de modo que el lazo principal quedaba atrapado midiendo ocupacion:
  // el registro caia de 20 Hz a 0.16 Hz, la FSM del LTE avanzaba un estado por
  // vuelta y el ZED-F9P desbordaba su buffer I2C entre lecturas.
  // La ocupacion real sigue midiendose, pero solo en manageVibrationStorage,
  // que corre con un periodo mucho mayor.
  if (!SD.exists(PENDING_FILE) && !SD.exists(LASTPOS_FILE) &&
      !SD.exists(IMU_CAL_FILE) && !SD.exists(vibFilename)) {
    // Ninguno de los archivos propios responde: la tarjeta ya no esta montada.
    noteSdFailure("sonda periodica de directorio fallo", true);
    return;
  }
  noteSdSuccess();

  // Tareas de reconciliacion tras fallos parciales.
  if (pendingRemovalRequired) {
    if (!SD.exists(PENDING_FILE) || SD.remove(PENDING_FILE)) {
      pendingRemovalRequired = false;
      noteSdSuccess();
      pulseSdLed();
    } else {
      noteSdFailure("no se pudo eliminar PENDING confirmado");
    }
  } else if (havePending && !SD.exists(PENDING_FILE)) {
    persistPendingToSd();
  }
}

// ============================================================================
// OLED
// ============================================================================
void updateOled() {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(sourceName(txSource));
  display.print(" H:");
  if (txHeading_deg >= 0.0f) display.print(txHeading_deg, 0);
  else display.print("--");
  display.print(" N:"); display.println(northReferenceValid ? "Y" : "N");

  display.setCursor(0, 11);
  display.print("Lat:"); display.println((float)txLat, 6);
  display.setCursor(0, 21);
  display.print("Lon:"); display.println((float)txLon, 6);

  display.setCursor(0, 31);
  display.print("V:"); display.print(txSpeedKmh, 1);
  display.print(" F:"); display.print(gpsFixType);
  display.print(" R:"); display.println(gpsCarrierSolution);

  display.setCursor(0, 41);
  display.print("E:"); display.print(fusionInitialized ? fusionEast.v : 0.0f, 1);
  display.print(" N:"); display.println(fusionInitialized ? fusionNorth.v : 0.0f, 1);

  display.setCursor(0, 52);
  display.print("CAL:I");
  display.print(icmAlignment.floorValid ? 1 : 0);
  display.print(icmAlignment.headingValid ? 1 : 0);
  display.print(" M");
  display.print(mpuAlignment.floorValid ? 1 : 0);
  display.print(mpuAlignment.headingValid ? 1 : 0);
  display.print(" SD:"); display.print(sdOK ? "O" : "F");

  display.display();
}

// ============================================================================
// MODEM Y FSM LTE
// MODEM Y FSM LTE
// ============================================================================
void clearLteRxFlags() {
  rxOK = false;
  rxERROR = false;
  rxDOWNLOAD = false;
  rxNetOpenSuccess = false;
  rxNetAlreadyOpen = false;
  rxHttpAction = false;
}

void enterLteState(LteState state) {
  lteState = state;
  lteStateStartMs = millis();
  clearLteRxFlags();
}

void modemSend(const String &command) {
  Serial.print("MDM> ");
  Serial.println(command);
  modem->println(command);
}

bool readModemLine(String &line) {
  while (modem->available()) {
    char c = (char)modem->read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (modemRxBuffer.length() > 0) {
        line = modemRxBuffer;
        modemRxBuffer = "";
        line.trim();
        return true;
      }
    } else {
      modemRxBuffer += c;
      if (modemRxBuffer.length() > 300) modemRxBuffer.remove(0, 150);
    }
  }
  return false;
}

void parseModemLine(const String &line) {
  Serial.print("MDM< ");
  Serial.println(line);

  if (line == "OK") rxOK = true;
  if (line.indexOf("ERROR") >= 0) rxERROR = true;
  if (line.indexOf("DOWNLOAD") >= 0) rxDOWNLOAD = true;
  if (line.indexOf("Network is already opened") >= 0) rxNetAlreadyOpen = true;

  if (line.startsWith("+NETOPEN:")) {
    int result = line.substring(line.indexOf(':') + 1).toInt();
    if (result == 0) rxNetOpenSuccess = true;
  }

  if (line.startsWith("+HTTPACTION:")) {
    int firstComma = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);
    if (firstComma >= 0 && secondComma > firstComma) {
      httpCode = line.substring(firstComma + 1, secondComma).toInt();
      httpResponseLength = line.substring(secondComma + 1).toInt();
      rxHttpAction = true;
    }
  }
}

String buildPostData() {
  uint32_t now = millis();
  float secondsWithoutGps = hasPositionReference ?
                            (now - lastGpsValidMs) / 1000.0f : -1.0f;
  float latestVibration_g =
      (mpuPresent && isfinite(vibMagLin_g)) ? vibMagLin_g : -1.0f;
  float field4Value = FIELD4_MODE == FIELD4_HEADING_DEG
      ? txHeading_deg : (float)dirNS;

  String status = "SRC_" + String((int)txSource) +
                  "_IMU_" + String((int)imuHealth) +
                  "_HDG_" + String(txHeading_deg, 0) +
                  "_NORTH_" + String(northReferenceValid ? 1 : 0) +
                  "_RTK_" + String((int)gpsCarrierSolution);
  if (referenceRecoveredFromSd && !gpsUsable) status += "_REFSD";
  status += "_CAL_I" + String(icmAlignment.floorValid ? 1 : 0) +
            String(icmAlignment.headingValid ? 1 : 0) +
            "M" + String(mpuAlignment.floorValid ? 1 : 0) +
            String(mpuAlignment.headingValid ? 1 : 0);

  String post = "api_key=" + String(API_KEY) +
                "&field1=" + String(txLat, 6) +
                "&field2=" + String(txLon, 6) +
                "&field3=" + String(txSpeedKmh, 1) +
                "&field4=" + String(field4Value, FIELD4_MODE == FIELD4_HEADING_DEG ? 1 : 0) +
                "&field5=" + String(secondsWithoutGps, 1) +
                "&field6=" + String(ID_TREN) +
                "&field7=" + String(ID_NODO) +
                "&field8=" + String(latestVibration_g, 4) +
                "&status=" + status;
  return post;
}

bool refreshPendingWithLatest(uint32_t now, const char *reason) {
  if (txSource == SRC_NONE || !coordinatesAreValid(txLat, txLon)) return false;

  String latest = buildPostData();
  latest.trim();
  if (latest.length() == 0) return false;

  // La sustitucion solo se realiza con la FSM en reposo. De esta forma no cambia
  // el tamano ni el contenido de un cuerpo HTTP que ya se esta cargando al modem.
  savePendingToSd(latest);
  lastPendingRefreshMs = now;
  pendingNeedsRefresh = false;

  Serial.print("PENDING actualizado con la lectura mas reciente");
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.println(".");
  return true;
}

void startLteCycle() {
  if (!havePending) {
    // Se toma una instantanea de posicion y se guarda ANTES de transmitir.
    // Asi, si ocurre un reinicio o una perdida de energia durante el envio,
    // el paquete puede recuperarse desde PENDING.TXT al volver a encender.
    refreshPendingWithLatest(millis(), "nuevo paquete");
  }

  if (!havePending || pendingLine.length() == 0) return;

  httpCode = -1;
  httpResponseLength = 0;
  cycleHttpSuccess = false;
  modemRxBuffer = "";
  enterLteState(LTE_SEND_AT);
}

void lteStep() {
  uint32_t now = millis();

  if (lteState == LTE_IDLE) {
    bool canBuildNewPayload = txSource != SRC_NONE && coordinatesAreValid(txLat, txLon);

    // Mientras exista un pendiente, se conserva una sola muestra. Cada 15 s se
    // reemplaza por el estado valido mas reciente. Un paquete recuperado al
    // arrancar se actualiza en cuanto haya una referencia utilizable.
    bool pendingRefreshDue = havePending && canBuildNewPayload &&
                             (pendingNeedsRefresh ||
                              elapsed(now, lastPendingRefreshMs, SEND_PERIOD_MS));
    if (pendingRefreshDue) {
      refreshPendingWithLatest(now,
          pendingNeedsRefresh ? "recuperado de SD" : "actualizacion de 15 s");
    }

    bool timeForNew = !havePending && canBuildNewPayload &&
                      elapsed(now, lastSendMs, SEND_PERIOD_MS);
    bool timeForRetry = havePending && elapsed(now, lastRetryMs, RETRY_AFTER_MS);

    if (timeForNew || timeForRetry) {
      if (timeForNew) lastSendMs = now;
      if (timeForRetry) lastRetryMs = now;
      startLteCycle();
    }
    return;
  }

  String line;
  while (readModemLine(line)) parseModemLine(line);

  const uint32_t TO_SHORT = 3000;
  const uint32_t TO_MEDIUM = 10000;
  const uint32_t TO_LONG = 30000;

  switch (lteState) {
    case LTE_SEND_AT:
      modemSend("AT");
      enterLteState(LTE_WAIT_AT);
      break;

    case LTE_WAIT_AT:
      if (rxOK) enterLteState(LTE_SEND_ATE0);
      else if (rxERROR || lteTimedOut(TO_SHORT)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_ATE0:
      modemSend("ATE0");
      enterLteState(LTE_WAIT_ATE0);
      break;

    case LTE_WAIT_ATE0:
      if (rxOK) enterLteState(LTE_SEND_CFUN);
      else if (rxERROR || lteTimedOut(TO_SHORT)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_CFUN:
      modemSend("AT+CFUN=1");
      enterLteState(LTE_WAIT_CFUN);
      break;

    case LTE_WAIT_CFUN:
      if (rxOK) enterLteState(LTE_SEND_CGDCONT);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_CGDCONT:
      modemSend(String("AT+CGDCONT=1,\"IP\",\"") + APN + "\"");
      enterLteState(LTE_WAIT_CGDCONT);
      break;

    case LTE_WAIT_CGDCONT:
      if (rxOK) enterLteState(LTE_SEND_CGATT);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_CGATT:
      modemSend("AT+CGATT=1");
      enterLteState(LTE_WAIT_CGATT);
      break;

    case LTE_WAIT_CGATT:
      if (rxOK) enterLteState(LTE_SEND_CGACT);
      else if (rxERROR || lteTimedOut(TO_LONG)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_CGACT:
      modemSend("AT+CGACT=1,1");
      enterLteState(LTE_WAIT_CGACT);
      break;

    case LTE_WAIT_CGACT:
      if (rxOK) enterLteState(LTE_SEND_NETOPEN);
      else if (rxERROR || lteTimedOut(TO_LONG)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_NETOPEN:
      modemSend("AT+NETOPEN");
      enterLteState(LTE_WAIT_NETOPEN);
      break;

    case LTE_WAIT_NETOPEN:
      if (rxNetOpenSuccess || rxNetAlreadyOpen) {
        enterLteState(LTE_SEND_HTTPTERM_PRE);
      } else if (rxERROR && !rxNetAlreadyOpen) {
        // Algunos firmwares responden ERROR si la red ya estaba abierta.
        // Se intenta continuar; HTTPINIT confirmara si el contexto es utilizable.
        enterLteState(LTE_SEND_HTTPTERM_PRE);
      } else if (lteTimedOut(TO_LONG)) {
        enterLteState(LTE_FAIL);
      }
      break;

    case LTE_SEND_HTTPTERM_PRE:
      modemSend("AT+HTTPTERM");
      enterLteState(LTE_WAIT_HTTPTERM_PRE);
      break;

    case LTE_WAIT_HTTPTERM_PRE:
      // ERROR es aceptable: significa que no habia sesion HTTP previa.
      if (rxOK || rxERROR || lteTimedOut(TO_SHORT)) enterLteState(LTE_SEND_HTTPINIT);
      break;

    case LTE_SEND_HTTPINIT:
      modemSend("AT+HTTPINIT");
      enterLteState(LTE_WAIT_HTTPINIT);
      break;

    case LTE_WAIT_HTTPINIT:
      if (rxOK) enterLteState(LTE_SEND_CID);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_CID:
      modemSend("AT+HTTPPARA=\"CID\",1");
      enterLteState(LTE_WAIT_CID);
      break;

    case LTE_WAIT_CID:
      if (rxOK) enterLteState(LTE_SEND_URL);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_URL:
      modemSend(String("AT+HTTPPARA=\"URL\",\"") + SERVER_URL + "\"");
      enterLteState(LTE_WAIT_URL);
      break;

    case LTE_WAIT_URL:
      if (rxOK) enterLteState(LTE_SEND_CONTENT);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_CONTENT:
      modemSend("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"");
      enterLteState(LTE_WAIT_CONTENT);
      break;

    case LTE_WAIT_CONTENT:
      if (rxOK) enterLteState(LTE_SEND_HTTPDATA);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_HTTPDATA:
      modemSend("AT+HTTPDATA=" + String(pendingLine.length()) + ",10000");
      enterLteState(LTE_WAIT_DOWNLOAD);
      break;

    case LTE_WAIT_DOWNLOAD:
      if (rxDOWNLOAD) enterLteState(LTE_SEND_BODY);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_BODY:
      Serial.print("MDM> [payload "); Serial.print(pendingLine.length()); Serial.println(" bytes]");
      modem->print(pendingLine);
      enterLteState(LTE_WAIT_BODY_OK);
      break;

    case LTE_WAIT_BODY_OK:
      if (rxOK) enterLteState(LTE_SEND_HTTPACTION);
      else if (rxERROR || lteTimedOut(TO_MEDIUM)) enterLteState(LTE_FAIL);
      break;

    case LTE_SEND_HTTPACTION:
      modemSend("AT+HTTPACTION=1");
      enterLteState(LTE_WAIT_HTTPACTION);
      break;

    case LTE_WAIT_HTTPACTION:
      if (rxHttpAction) {
        cycleHttpSuccess = httpCode >= 200 && httpCode < 300;
        enterLteState(LTE_SEND_HTTPTERM_POST);
      } else if (rxERROR || lteTimedOut(TO_LONG)) {
        enterLteState(LTE_FAIL);
      }
      break;

    case LTE_SEND_HTTPTERM_POST:
      modemSend("AT+HTTPTERM");
      enterLteState(LTE_WAIT_HTTPTERM_POST);
      break;

    case LTE_WAIT_HTTPTERM_POST:
      if (rxOK || rxERROR || lteTimedOut(TO_SHORT)) {
        enterLteState(cycleHttpSuccess ? LTE_DONE : LTE_FAIL);
      }
      break;

    case LTE_DONE:
      Serial.print("HTTP exitoso: "); Serial.println(httpCode);
      clearPendingAfterSuccess();
      pulseTsLed();
      lastSendMs = millis();
      enterLteState(LTE_IDLE);
      break;

    case LTE_FAIL:
      Serial.print("Fallo LTE/HTTP. Codigo: "); Serial.println(httpCode);
      // Se conserva el cuerpo exacto que se intento enviar. La proxima vez que
      // la FSM este en IDLE, se reemplazara por la lectura mas reciente cuando
      // hayan transcurrido 15 s desde la ultima instantanea.
      if (pendingLine.length() > 0) persistPendingToSd();
      lastRetryMs = millis();
      enterLteState(LTE_IDLE);
      break;

    default:
      enterLteState(LTE_IDLE);
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1200);

  pinMode(LED_ONBOARD, OUTPUT);
  pinMode(LED_HEART_EXT, OUTPUT);
  pinMode(LED_SD, OUTPUT);
  pinMode(LED_GPSFAIL, OUTPUT);
  pinMode(LED_TS_OK, OUTPUT);

  digitalWrite(LED_ONBOARD, LOW);
  digitalWrite(LED_HEART_EXT, LOW);
  digitalWrite(LED_SD, LOW);
  digitalWrite(LED_GPSFAIL, HIGH);
  digitalWrite(LED_TS_OK, LOW);

  Serial.println();
  Serial.print("Iniciando ");
  Serial.println(FW_VERSION);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeout(50);

  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(FW_VERSION);
    display.println("Inicializando...");
    display.display();
    display.setTextWrap(false);
  } else {
    Serial.println("WARN: OLED no detectada.");
  }

  if (!initializeSdStorage(true)) {
    Serial.println("ERROR: microSD no disponible al iniciar.");
    Serial.println("Se continuara sin almacenamiento y se reintentara cada 5 s.");
  }
  else {
    loadImuCalibrationFromSd();
  }

  mpuPresent = initMpu6500();
  if (mpuPresent) {
    Serial.println("MPU6500 OK: +/-4 g, lectura 50 Hz, log 20 Hz, autocal general activa.");
    initializeMpuGravityEstimate();
  } else {
    Serial.println("WARN: MPU6500 no detectado.");
  }

  gpsPresent = myGPS.begin(Wire);
  if (gpsPresent) {
    Serial.println("ZED-F9P OK: salida I2C exclusivamente UBX-NAV-PVT.");
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(5);
    myGPS.setAutoPVT(true);
  } else {
    Serial.println("WARN: ZED-F9P no detectado.");
  }

  ICM_20948_Status_e icmStatus = myICM.begin(Wire, ICM_ADDR);
  icmPresent = icmStatus == ICM_20948_Stat_Ok;
  if (icmPresent) {
    Serial.println("ICM-20948 detectada.");
    if (configureIcm20948()) {
      Serial.println("ICM configurada: +/-2 g, +/-250 dps, DLPF ~24 Hz, ~100 Hz.");
      calibrateIcm20948();
    } else {
      Serial.println("WARN: fallo al configurar ICM-20948.");
      icmPresent = false;
    }
  } else {
    Serial.print("WARN: ICM-20948 no disponible. Estado=");
    Serial.println(icmStatus);
  }

  // Guarda el piso recalculado; el eje y el norte se refinan durante la marcha.
  if (sdOK && imuCalibrationDirty) saveImuCalibrationToSd(true);

#if ENABLE_WHEEL_ODOMETRY
  pinMode(WHEEL_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_PULSE_PIN), wheelPulseIsr, RISING);
  lastWheelUpdateMs = millis();
#endif

  // Encendido del SIM7600 con el pulso utilizado en la version anterior.
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(100);
  digitalWrite(PWRKEY, HIGH);
  delay(900);
  digitalWrite(PWRKEY, LOW);
  delay(4500);

  // Serial1 usa GP0 (TX) y GP1 (RX) en la configuracion del proyecto.
  modem->begin(115200);
  Serial.println("SIM7600 inicializado; FSM en espera.");

  uint32_t now = millis();
  lastHeartbeatMs = now;
  lastVibLogMs = now;
  lastOledMs = now;
  lastVibFlushMs = now;
  lastPosSaveMs = now;
  lastSdUsageCheckMs = now;
  lastSdHealthCheckMs = now;
  lastSdRetryMs = now;
  lastSdFaultBlinkMs = now;
  lastSendMs = now;
  lastRetryMs = now;
  lastPendingRefreshMs = now;
  enterLteState(LTE_IDLE);
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
struct FaseperfilT { const char *nombre; uint32_t maxUs; uint32_t totalUs; };
FaseperfilT perfil[] = {
  {"serviceLedPulses",0,0},{"serviceSdManager",0,0},{"updateIcmSample",0,0},
  {"updateMpuSample",0,0},{"pollGnss",0,0},{"saveImuCal",0,0},
  {"applyStillness",0,0},{"evaluateImuHealth",0,0},{"updateFinalPosition",0,0},
  {"saveLastPos",0,0},{"writeVibrationLog",0,0},{"manageVibStorage",0,0},
  {"lteStep",0,0},{"updateOled",0,0}
};
static const uint8_t PERFIL_N = sizeof(perfil)/sizeof(perfil[0]);
uint32_t perfilVueltas = 0;
uint32_t perfilUltimoReporteMs = 0;
uint32_t perfilPeorVueltaUs = 0;

#define PERFILAR(indice, llamada) do {     uint32_t _t0 = micros(); llamada; uint32_t _d = micros() - _t0;     if (_d > perfil[indice].maxUs) perfil[indice].maxUs = _d;     perfil[indice].totalUs += _d;   } while (0)

void perfilReportar(uint32_t now) {
  if (perfilVueltas == 0) return;
  Serial.println();
  Serial.print("=== PERFIL tras "); Serial.print(perfilVueltas);
  Serial.println(" vueltas ===");
  Serial.print("peor vuelta completa: "); Serial.print(perfilPeorVueltaUs / 1000.0f, 1);
  Serial.println(" ms");
  Serial.println("etapa                 max[ms]   media[ms]");
  for (uint8_t i = 0; i < PERFIL_N; ++i) {
    Serial.print("  ");
    Serial.print(perfil[i].nombre);
    for (int k = strlen(perfil[i].nombre); k < 20; ++k) Serial.print(' ');
    Serial.print(perfil[i].maxUs / 1000.0f, 1);
    Serial.print("      ");
    Serial.println((perfil[i].totalUs / (float)perfilVueltas) / 1000.0f, 2);
    perfil[i].maxUs = 0;
    perfil[i].totalUs = 0;
  }
  perfilVueltas = 0;
  perfilPeorVueltaUs = 0;
  perfilUltimoReporteMs = now;
}

void loop() {
  uint32_t now = millis();
  uint32_t _vuelta0 = micros();

  PERFILAR(0, serviceLedPulses());
  PERFILAR(1, serviceSdManager(now));

  if (elapsed(now, lastHeartbeatMs, 500)) {
    lastHeartbeatMs = now;
    heartbeatState = !heartbeatState;
    digitalWrite(LED_ONBOARD, heartbeatState);
    digitalWrite(LED_HEART_EXT, heartbeatState);
  }

  previousGpsUsable = gpsUsable;

  // La IMU predice en ENU; GNSS y odometria corrigen el filtro 2D.
  PERFILAR(2, updateIcmSample(now));
  PERFILAR(3, updateMpuSample(now));
  PERFILAR(4, pollGnss(now));
  PERFILAR(5, { if (imuCalibrationDirty) saveImuCalibrationToSd(false); });
#if ENABLE_WHEEL_ODOMETRY
  updateWheelOdometry(now);
#endif

  PERFILAR(6, applyStillnessCorrection(now));
  PERFILAR(7, evaluateImuHealth(now));
  PERFILAR(8, updateFinalPosition(now));

  PERFILAR(9, saveLastPositionIfDue(now));
  PERFILAR(10, writeVibrationLog(now));
  PERFILAR(11, manageVibrationStorage(now));

  PERFILAR(12, lteStep());

  if (elapsed(now, lastOledMs, OLED_INTERVAL_MS)) {
    lastOledMs = now;
    PERFILAR(13, updateOled());
  }

  uint32_t _vd = micros() - _vuelta0;
  if (_vd > perfilPeorVueltaUs) perfilPeorVueltaUs = _vd;
  perfilVueltas++;
  if (elapsed(now, perfilUltimoReporteMs, 5000)) perfilReportar(now);

  delay(2);
}
