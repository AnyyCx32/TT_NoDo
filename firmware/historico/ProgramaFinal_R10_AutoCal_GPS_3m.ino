// ============================================================================
//  PROGRAMA FINAL R10 - MAP FUSION + AUTOCALIBRACION IMU
//  Raspberry Pi Pico (RP2040) + ZED-F9P + ICM-20948 + MPU6500 + microSD
//  + OLED SSD1306 + SIM7600G-H
//
//  CAMBIOS PRINCIPALES RESPECTO A R8:
//    1) La direccion ya no se decide por el signo de la latitud. La posicion
//       GNSS se proyecta sobre un modelo ordenado de la via Taxquena-Xochimilco.
//       El sentido se obtiene mediante la velocidad GNSS proyectada sobre la
//       tangente local de la via y, como respaldo, mediante el avance en la
//       coordenada longitudinal s. Funciona en estaciones y entre estaciones.
//    2) La localizacion se restringe a una dimension sobre la via. Se utiliza
//       un filtro de Kalman de tres estados: distancia sobre la via, velocidad
//       longitudinal y sesgo del acelerometro.
//    3) La ICM-20948 usa acelerometro Y giroscopio. Se configuran expresamente
//       +/-2 g, +/-250 dps, modo continuo, DLPF y una tasa cercana a 100 Hz.
//       Un filtro complementario estima roll/pitch para compensar gravedad.
//    4) El MPU6500 queda dedicado a vibracion, confirmacion de reposo y
//       diagnostico. Ya no sustituye automaticamente la velocidad de navegacion.
//    5) Se validan fix 3D, gnssFixOK, coordenadas, satelites, hAcc, sAcc, pDOP,
//       antiguedad del PVT y distancia transversal respecto de la via.
//    6) Se registra el estado RTK. No se exige RTK por defecto: para precision
//       centimetrica deben entrar correcciones RTCM/SPARTN por otra interfaz.
//    7) Se incluye una entrada opcional de odometria de rueda. Esta desactivada
//       hasta instalar y calibrar el sensor de pulsos.
//    8) El bus I2C queda configurado explicitamente a 400 kHz.
//    9) La orientacion de ambos acelerometros se calibra automaticamente:
//       el piso se obtiene del vector de gravedad en reposo y el eje
//       longitudinal positivo se aprende correlacionando la aceleracion de
//       cada sensor con la aceleracion GNSS proyectada sobre la via.
//   10) El mapa aporta continuamente la tangente local y, con ella, la
//       referencia geografica respecto al norte en curvas y estaciones.
//       La calibracion se guarda en IMUCAL.BIN y se refina durante la marcha.
//
//  ADVERTENCIAS DE INTEGRACION:
//    - Las coordenadas incluidas son anclas aproximadas de las 18 estaciones.
//      Para validacion final deben reemplazarse o complementarse con una
//      polilinea densa levantada sobre el centro real de la via.
//    - ICM_BODY_* y MPU_BODY_* solo son una orientacion provisional de
//      respaldo mientras no se complete la calibracion automatica con GNSS.
//      La primera calibracion de rumbo necesita movimiento con aceleraciones o
//      frenados observables; a velocidad constante un acelerometro no revela yaw.
//    - En reposo dentro de una estacion intermedia no existe informacion fisica
//      suficiente para conocer el siguiente sentido. Se conserva el ultimo
//      sentido valido guardado. En las terminales se asume salida hacia el
//      extremo opuesto. Al iniciar movimiento, GNSS confirma el sentido.
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
static const char FW_VERSION[] = "TT-NODO-R10-AUTOCAL";
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
// field1=latitud, field2=longitud, field3=velocidad_kmh, field4=direccion,
// field5=segundos_sin_GNSS, field6=id_tren, field7=id_nodo,
// field8=vibracion_lineal_mas_reciente_g.
// El campo status incluye fuente, salud IMU, estacion, XTE y estado RTK.
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
// MODELO DE VIA Y ORIENTACION DE SENSORES
// ============================================================================
// Orden positivo de la via: Taxquena -> Xochimilco.
// dirNS conserva el formato de la plataforma:
//   0 = hacia Taxquena
//   1 = hacia Xochimilco
struct TrackPoint {
  const char *name;
  double lat;
  double lon;
};

// Anclas aproximadas de estaciones. Agregar puntos intermedios medidos mejora
// el ajuste en curvas sin cambiar el algoritmo de proyeccion.
static const TrackPoint TRACK_POINTS[] = {
  {"Taxquena",          19.34367, -99.14048},
  {"Las Torres",        19.34080, -99.14342},
  {"Ciudad Jardin",     19.33575, -99.14186},
  {"La Virgen",         19.33167, -99.14060},
  {"Xotepingo",         19.32748, -99.13930},
  {"Nezahualpilli",     19.32380, -99.13816},
  {"Registro Federal",  19.31788, -99.13881},
  {"Textitlan",         19.31255, -99.14058},
  {"El Vergel",         19.30732, -99.14294},
  {"Estadio Azteca",    19.30217, -99.14704},
  {"Huipulco",          19.29763, -99.15069},
  {"Xomali",            19.28878, -99.14684},
  {"Periferico",        19.28274, -99.13968},
  {"Tepepan",           19.27946, -99.13313},
  {"La Noria",          19.26788, -99.12555},
  {"Huichapan",         19.26439, -99.11862},
  {"Francisco Goitia",  19.26073, -99.11128},
  {"Xochimilco",        19.25946, -99.10813}
};
static const uint8_t TRACK_POINT_COUNT =
    sizeof(TRACK_POINTS) / sizeof(TRACK_POINTS[0]);
static const double TRACK_ORIGIN_LAT = 19.30217;
static const double TRACK_ORIGIN_LON = -99.14704;
float trackEastM[TRACK_POINT_COUNT];
float trackNorthM[TRACK_POINT_COUNT];
float trackCumM[TRACK_POINT_COUNT];
float trackTotalM = 0.0f;

struct TrackProjection {
  bool valid;
  float s_m;
  float crossTrack_m;
  uint8_t segment;
  float fraction;
  float tangentE;
  float tangentN;
  double lat;
  double lon;
};

// Orientacion provisional de respaldo. Se usa solo hasta que la calibracion
// automatica determine el piso y el eje longitudinal de cada tarjeta.
// 0=X del sensor, 1=Y, 2=Z.
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

// Marco calibrado del tren usado por el algoritmo:
//   X = sentido positivo de la via, Taxquena -> Xochimilco.
//   Y = lateral derecha respecto a +X.
//   Z = vertical hacia arriba.
// El mapa convierte +X a este/norte mediante la tangente local de la via.
struct Vec3f {
  float x;
  float y;
  float z;
};

struct SensorAlignment {
  Vec3f upSensor;          // vertical positiva expresada en ejes del sensor
  Vec3f forwardSensor;     // +via expresado en ejes del sensor
  Vec3f rightSensor;       // lateral derecha respecto al eje +via
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
static const uint16_t IMU_CAL_VERSION = 1;

struct ImuCalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t sequence;
  uint8_t icmFloorValid;
  uint8_t icmHeadingValid;
  uint8_t mpuFloorValid;
  uint8_t mpuHeadingValid;
  float icmUp[3];
  float icmForward[3];
  float mpuUp[3];
  float mpuForward[3];
  float icmHeadingQuality;
  float mpuHeadingQuality;
  uint32_t crc;
};

uint32_t imuCalibrationSequence = 0;
bool imuCalibrationLoaded = false;
bool imuCalibrationDirty = false;
uint32_t lastImuCalibrationSaveMs = 0;

// ============================================================================
// INTERVALOS
// ============================================================================
static const uint32_t I2C_CLOCK_HZ         = 400000;
static const uint32_t OLED_INTERVAL_MS     = 250;
static const uint32_t MPU_SAMPLE_MS        = 20;   // lectura efectiva 50 Hz
static const uint32_t VIB_LOG_INTERVAL_MS  = 50;   // almacenamiento 20 Hz
static const uint32_t FILE_FLUSH_MS         = 1000;
static const uint32_t LASTPOS_SAVE_INTERVAL_MS = 15000;
static const uint32_t SD_USAGE_CHECK_MS        = 300000;
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
// CRITERIOS GNSS Y MAP MATCHING
// ============================================================================
static const uint8_t  MIN_GNSS_SATS         = 6;
static const uint32_t HACC_LIMIT_MM         = 3000;  // 3 m
static const uint32_t SACC_LIMIT_MMPS       = 1500;  // 1.5 m/s
static const uint16_t PDOP_LIMIT_CENTI      = 600;   // 6.00
static const float    MAP_MAX_CROSSTRACK_M  = 150.0f;
static const float    DIRECTION_SPEED_MPS   = 0.50f;
static const float    DIRECTION_DELTA_S_M   = 3.0f;
static const uint8_t  DIRECTION_CONFIRM_SAMPLES = 3;
static const float    TERMINAL_GEOFENCE_M   = 100.0f;
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
// TIEMPO, GNSS Y POSICION SOBRE LA VIA
// ============================================================================
bool gpsTimeValid = false;
uint16_t gpsYear = 0;
uint8_t gpsMonth = 0, gpsDay = 0;
uint8_t gpsHour = 0, gpsMinute = 0, gpsSecond = 0;

double gpsLat = 0.0, gpsLon = 0.0; // posicion cruda UBX-NAV-PVT
float gpsGroundSpeed_mps = 0.0f;
float gpsSpeed_mps = 0.0f;         // magnitud longitudinal sobre la via
float gpsTrackSpeedSigned_mps = 0.0f;
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
TrackProjection gpsTrackProjection = {};
float gpsTrackS_m = 0.0f;
float gpsCrossTrack_m = 9999.0f;
uint8_t nearestStation = 0;

uint32_t lastPvtMs = 0;
uint32_t lastGpsValidMs = 0;

// Referencia persistente. En R9 coincide con una posicion ajustada a la via.
bool hasPositionReference = false;
double refLat = 0.0, refLon = 0.0;
bool referenceRecoveredFromSd = false;
uint32_t recoveredReferenceMs = 0;

// 0=hacia Taxquena; 1=hacia Xochimilco.
int dirNS = 0;
bool directionKnown = false;
int directionCandidate = -1;
uint8_t directionCandidateCount = 0;
float directionRefS_m = 0.0f;
bool directionRefValid = false;
uint32_t directionRefMs = 0;

// ============================================================================
// ESTADO ICM-20948: ACTITUD Y ACELERACION LONGITUDINAL
// ============================================================================
float icmRawX_mg = 0.0f, icmRawY_mg = 0.0f, icmRawZ_mg = 0.0f;
float icmRawGyrX_dps = 0.0f, icmRawGyrY_dps = 0.0f, icmRawGyrZ_dps = 0.0f;
float icmBodyAx_g = 0.0f, icmBodyAy_g = 0.0f, icmBodyAz_g = 0.0f;
float icmBodyGx_dps = 0.0f, icmBodyGy_dps = 0.0f, icmBodyGz_dps = 0.0f;
float icmGyroBiasSensorX_dps = 0.0f, icmGyroBiasSensorY_dps = 0.0f, icmGyroBiasSensorZ_dps = 0.0f;
float icmLongZero_mps2 = 0.0f;
float icmMag_g = 0.0f;
float icmRoll_deg = 0.0f, icmPitch_deg = 0.0f;
float icmTrackAccelRaw_mps2 = 0.0f;
float icmTrackAccel_mps2 = 0.0f;
bool attitudeInitialized = false;
uint32_t lastIcmSampleUs = 0;

// ============================================================================
// ESTADO MPU6500: VIBRACION Y CONFIRMACION DE REPOSO
// ============================================================================
float mpuRawX_g = 0.0f, mpuRawY_g = 0.0f, mpuRawZ_g = 0.0f;
float mpuBodyAx_g = 0.0f, mpuBodyAy_g = 0.0f, mpuBodyAz_g = 0.0f;
float mpuMag_g = 0.0f;
uint32_t lastMpuSampleMs = 0;

// Derivada GNSS usada exclusivamente para aprender el eje +via de las IMU.
bool headingCalPrevGpsValid = false;
float headingCalPrevTrackSpeed_mps = 0.0f;
uint8_t headingCalPrevSegment = 255;
uint32_t headingCalPrevGpsMs = 0;
float headingCalGpsAccel_mps2 = 0.0f;
float currentTrackHeadingTrue_deg = 0.0f;
float gEstX = 0.0f, gEstY = 0.0f, gEstZ = 0.0f;
bool gEstInit = false;
float vibAxLin_g = 0.0f, vibAyLin_g = 0.0f, vibAzLin_g = 0.0f;
float vibMagLin_g = 0.0f;

// ============================================================================
// FILTRO DE KALMAN LONGITUDINAL x=[s, v, bias_acc]
// ============================================================================
bool fusionInitialized = false;
float fusionS_m = 0.0f;
float fusionV_mps = 0.0f;
float fusionBias_mps2 = 0.0f;
float fusionP[3][3] = {{0}};
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
  SRC_MAP_GNSS = 1,
  SRC_MAP_DR = 2
};

PositionSource txSource = SRC_NONE;
double txLat = 0.0, txLon = 0.0;
float txSpeedKmh = 0.0f;

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

void latLonToLocal(double lat, double lon, float &east, float &north) {
  const double earthRadius = 6378137.0;
  const double lat0Rad = TRACK_ORIGIN_LAT * PI / 180.0;
  east = (float)((lon - TRACK_ORIGIN_LON) * PI / 180.0 *
                 earthRadius * cos(lat0Rad));
  north = (float)((lat - TRACK_ORIGIN_LAT) * PI / 180.0 * earthRadius);
}

void localToLatLon(float east, float north, double &lat, double &lon) {
  const double earthRadius = 6378137.0;
  const double lat0Rad = TRACK_ORIGIN_LAT * PI / 180.0;
  lat = TRACK_ORIGIN_LAT + (north / earthRadius) * 180.0 / PI;
  double denom = earthRadius * cos(lat0Rad);
  if (fabs(denom) < 1.0) denom = 1.0;
  lon = TRACK_ORIGIN_LON + (east / denom) * 180.0 / PI;
}

void initializeTrackModel() {
  trackCumM[0] = 0.0f;
  for (uint8_t i = 0; i < TRACK_POINT_COUNT; ++i) {
    latLonToLocal(TRACK_POINTS[i].lat, TRACK_POINTS[i].lon,
                  trackEastM[i], trackNorthM[i]);
    if (i > 0) {
      float de = trackEastM[i] - trackEastM[i - 1];
      float dn = trackNorthM[i] - trackNorthM[i - 1];
      trackCumM[i] = trackCumM[i - 1] + sqrtf(de * de + dn * dn);
    }
  }
  trackTotalM = trackCumM[TRACK_POINT_COUNT - 1];
  Serial.print("Longitud geometrica del modelo de estaciones: ");
  Serial.print(trackTotalM, 1);
  Serial.println(" m");
}

TrackProjection projectToTrack(double lat, double lon) {
  TrackProjection best = {};
  best.valid = false;
  best.crossTrack_m = 1.0e9f;

  if (!coordinatesAreValid(lat, lon) || TRACK_POINT_COUNT < 2) return best;

  float pe, pn;
  latLonToLocal(lat, lon, pe, pn);

  for (uint8_t i = 0; i < TRACK_POINT_COUNT - 1; ++i) {
    float ax = trackEastM[i];
    float ay = trackNorthM[i];
    float dx = trackEastM[i + 1] - ax;
    float dy = trackNorthM[i + 1] - ay;
    float len2 = dx * dx + dy * dy;
    if (len2 < 1.0f) continue;

    float t = ((pe - ax) * dx + (pn - ay) * dy) / len2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float qx = ax + t * dx;
    float qy = ay + t * dy;
    float de = pe - qx;
    float dn = pn - qy;
    float dist = sqrtf(de * de + dn * dn);

    if (dist < best.crossTrack_m) {
      float len = sqrtf(len2);
      best.valid = true;
      best.crossTrack_m = dist;
      best.segment = i;
      best.fraction = t;
      best.s_m = trackCumM[i] + t * len;
      best.tangentE = dx / len;
      best.tangentN = dy / len;
      localToLatLon(qx, qy, best.lat, best.lon);
    }
  }
  return best;
}

void pointAtTrackS(float s_m, double &lat, double &lon,
                   float *tangentE = nullptr, float *tangentN = nullptr,
                   uint8_t *segmentOut = nullptr) {
  if (TRACK_POINT_COUNT < 2) {
    lat = lon = 0.0;
    return;
  }

  if (s_m < 0.0f) s_m = 0.0f;
  if (s_m > trackTotalM) s_m = trackTotalM;

  uint8_t seg = TRACK_POINT_COUNT - 2;
  for (uint8_t i = 0; i < TRACK_POINT_COUNT - 1; ++i) {
    if (s_m <= trackCumM[i + 1]) {
      seg = i;
      break;
    }
  }

  float dx = trackEastM[seg + 1] - trackEastM[seg];
  float dy = trackNorthM[seg + 1] - trackNorthM[seg];
  float len = sqrtf(dx * dx + dy * dy);
  float t = (len > 0.01f) ? (s_m - trackCumM[seg]) / len : 0.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  float e = trackEastM[seg] + t * dx;
  float n = trackNorthM[seg] + t * dy;
  localToLatLon(e, n, lat, lon);

  if (tangentE) *tangentE = (len > 0.01f) ? dx / len : 0.0f;
  if (tangentN) *tangentN = (len > 0.01f) ? dy / len : 1.0f;
  if (segmentOut) *segmentOut = seg;
}

uint8_t nearestStationForS(float s_m) {
  uint8_t best = 0;
  float bestDistance = fabsf(s_m - trackCumM[0]);
  for (uint8_t i = 1; i < TRACK_POINT_COUNT; ++i) {
    float d = fabsf(s_m - trackCumM[i]);
    if (d < bestDistance) {
      bestDistance = d;
      best = i;
    }
  }
  return best;
}

const char *directionName() {
  return dirNS == 1 ? "XOCH" : "TAXQ";
}

void submitDirectionCandidate(int candidate) {
  if (candidate != 0 && candidate != 1) return;
  if (candidate == directionCandidate) {
    if (directionCandidateCount < 255) directionCandidateCount++;
  } else {
    directionCandidate = candidate;
    directionCandidateCount = 1;
  }

  if (directionCandidateCount >= DIRECTION_CONFIRM_SAMPLES) {
    dirNS = candidate;
    directionKnown = true;
  }
}

void applyTerminalDirectionRule(float s_m) {
  if (s_m <= TERMINAL_GEOFENCE_M) {
    dirNS = 1; // desde Taxquena solo puede salir hacia Xochimilco
    directionKnown = true;
  } else if (trackTotalM - s_m <= TERMINAL_GEOFENCE_M) {
    dirNS = 0; // desde Xochimilco solo puede salir hacia Taxquena
    directionKnown = true;
  }
}

void updateDirectionFromTrack(uint32_t now, float s_m, float signedSpeed_mps) {
  if (fabsf(signedSpeed_mps) >= DIRECTION_SPEED_MPS) {
    submitDirectionCandidate(signedSpeed_mps > 0.0f ? 1 : 0);
  }

  if (!directionRefValid) {
    directionRefS_m = s_m;
    directionRefMs = now;
    directionRefValid = true;
  } else if (elapsed(now, directionRefMs, DIRECTION_REF_MS)) {
    float ds = s_m - directionRefS_m;
    if (fabsf(ds) >= DIRECTION_DELTA_S_M) {
      submitDirectionCandidate(ds > 0.0f ? 1 : 0);
    }
    directionRefS_m = s_m;
    directionRefMs = now;
  }

  if (fabsf(signedSpeed_mps) < GPS_STILL_MPS) applyTerminalDirectionRule(s_m);
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
    case SRC_MAP_GNSS: return "GNSS+MAP";
    case SRC_MAP_DR: return "DR+MAP";
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
    Serial.println("WARN: IMUCAL.BIN invalido; se recalibrara.");
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

  clearHeadingAccumulator(icmAlignment);
  clearHeadingAccumulator(mpuAlignment);
  imuCalibrationSequence = record.sequence;
  imuCalibrationLoaded = icmAlignment.headingValid || mpuAlignment.headingValid;
  noteSdSuccess();

  Serial.print("Calibracion IMU recuperada. ICM=");
  Serial.print(icmAlignment.headingValid ? "PISO+GNSS" :
               (icmAlignment.floorValid ? "PISO" : "NO"));
  Serial.print(" MPU=");
  Serial.println(mpuAlignment.headingValid ? "PISO+GNSS" :
                 (mpuAlignment.floorValid ? "PISO" : "NO"));
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
  Serial.println("Calibracion de orientacion guardada en IMUCAL.BIN.");
  return true;
}

// ============================================================================
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

  // Las vibraciones se guardan ya expresadas en ejes del tren:
  // longitudinal +via, lateral derecha y vertical arriba.
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
// CALIBRACION DE RUMBO DE LAS IMU CON GNSS + MAPA
// ============================================================================
void addHeadingCalibrationObservation(SensorAlignment &alignment,
                                      const Vec3f &rawAccelSensor_g,
                                      float gpsTrackAccel_mps2,
                                      float dt,
                                      uint8_t fallbackForwardAxis,
                                      float fallbackForwardSign,
                                      const char *sensorName) {
  if (!alignment.floorValid) return;

  Vec3f horizontal = projectToPlane(rawAccelSensor_g, alignment.upSensor);
  float horizontalMag = normVec3(horizontal);
  if (horizontalMag < HEADING_CAL_MIN_HORIZONTAL_G) return;

  float weight = gpsTrackAccel_mps2 * dt;
  alignment.headingCorrelation = addVec3(
      alignment.headingCorrelation, scaleVec3(horizontal, weight));
  alignment.headingSamples++;
  alignment.headingExcitation += fabsf(gpsTrackAccel_mps2) * dt;
  alignment.headingWeight += horizontalMag * fabsf(weight);

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
    // Refinamiento lento para no cambiar bruscamente los ejes durante marcha.
    if (dotVec3(candidate, alignment.forwardSensor) < 0.0f)
      candidate = scaleVec3(candidate, -1.0f);
    Vec3f blended = addVec3(scaleVec3(alignment.forwardSensor, 0.85f),
                            scaleVec3(candidate, 0.15f));
    if (normalizeVec3(blended)) candidate = blended;
  }

  alignment.forwardSensor = candidate;
  alignment.headingValid = true;
  if (!rebuildAlignmentBasis(alignment,
                             fallbackForwardAxis, fallbackForwardSign)) {
    alignment.headingValid = false;
    return;
  }

  imuCalibrationDirty = true;
  clearHeadingAccumulator(alignment);
  Serial.print("Autocal "); Serial.print(sensorName);
  Serial.print(firstSolution ? " completada" : " refinada");
  Serial.print(". Calidad=");
  Serial.println(alignment.headingQuality, 3);
}

void updateHeadingCalibrationFromGps(uint32_t now) {
  if (!gpsQualityCurrent || !gpsTrackProjection.valid ||
      gpsSAcc_mmps / 1000.0f > HEADING_CAL_MAX_SACC_MPS) {
    headingCalPrevGpsValid = false;
    headingCalGpsAccel_mps2 = 0.0f;
    return;
  }

  currentTrackHeadingTrue_deg = atan2f(gpsTrackProjection.tangentE,
                                      gpsTrackProjection.tangentN) *
                                180.0f / PI;
  if (currentTrackHeadingTrue_deg < 0.0f) currentTrackHeadingTrue_deg += 360.0f;

  // Verificacion adicional respecto al norte verdadero reportado por GNSS.
  // La tangente positiva apunta a Xochimilco; si v<0 el rumbo de movimiento
  // esperado se invierte 180 grados.
  if (fabsf(gpsTrackSpeedSigned_mps) >= HEADING_CAL_MIN_COURSE_SPEED_MPS &&
      gpsHeadingAcc_deg <= HEADING_CAL_MAX_HEADACC_DEG) {
    float expectedCourse = currentTrackHeadingTrue_deg +
                           (gpsTrackSpeedSigned_mps < 0.0f ? 180.0f : 0.0f);
    if (expectedCourse >= 360.0f) expectedCourse -= 360.0f;
    if (angularDifferenceDeg(gpsHeading_deg, expectedCourse) >
        HEADING_CAL_MAX_COURSE_ERROR_DEG) {
      headingCalPrevGpsValid = false;
      headingCalGpsAccel_mps2 = 0.0f;
      return;
    }
  }

  if (!headingCalPrevGpsValid) {
    headingCalPrevGpsValid = true;
    headingCalPrevTrackSpeed_mps = gpsTrackSpeedSigned_mps;
    headingCalPrevSegment = gpsTrackProjection.segment;
    headingCalPrevGpsMs = now;
    return;
  }

  float dt = (now - headingCalPrevGpsMs) / 1000.0f;
  bool sameSegment = headingCalPrevSegment == gpsTrackProjection.segment;
  float rawAccel = (dt > 0.05f) ?
      (gpsTrackSpeedSigned_mps - headingCalPrevTrackSpeed_mps) / dt : 0.0f;

  headingCalPrevTrackSpeed_mps = gpsTrackSpeedSigned_mps;
  headingCalPrevSegment = gpsTrackProjection.segment;
  headingCalPrevGpsMs = now;

  if (!sameSegment || dt < 0.10f || dt > 1.0f ||
      !isfinite(rawAccel) ||
      fabsf(rawAccel) > HEADING_CAL_MAX_GPS_ACCEL_MPS2) return;

  float rc = 1.0f / (2.0f * PI * HEADING_ACCEL_LPF_HZ);
  float alpha = dt / (rc + dt);
  headingCalGpsAccel_mps2 += alpha * (rawAccel - headingCalGpsAccel_mps2);

  if (fabsf(headingCalGpsAccel_mps2) < HEADING_CAL_MIN_GPS_ACCEL_MPS2 ||
      fabsf(headingCalGpsAccel_mps2) > HEADING_CAL_MAX_GPS_ACCEL_MPS2) return;

  if (icmPresent) {
    addHeadingCalibrationObservation(
        icmAlignment,
        makeVec3(icmRawX_mg / 1000.0f,
                 icmRawY_mg / 1000.0f,
                 icmRawZ_mg / 1000.0f),
        headingCalGpsAccel_mps2, dt,
        ICM_BODY_X_SENSOR_AXIS, ICM_BODY_X_SIGN, "ICM");
  }

  if (mpuPresent) {
    addHeadingCalibrationObservation(
        mpuAlignment,
        makeVec3(mpuRawX_g, mpuRawY_g, mpuRawZ_g),
        headingCalGpsAccel_mps2, dt,
        MPU_BODY_X_SENSOR_AXIS, MPU_BODY_X_SIGN, "MPU");
  }

  if (icmAlignment.headingValid || mpuAlignment.headingValid)
    saveImuCalibrationToSd(false);
}

// ============================================================================
// FILTRO DE KALMAN LONGITUDINAL
// ============================================================================
void fusionInitialize(float s_m, float v_mps, bool recovered = false) {
  fusionInitialized = true;
  fusionS_m = fminf(fmaxf(s_m, 0.0f), trackTotalM);
  fusionV_mps = fminf(fmaxf(v_mps, -FUSION_MAX_SPEED_MPS), FUSION_MAX_SPEED_MPS);
  fusionBias_mps2 = 0.0f;

  float posVar = recovered ? 400.0f : 25.0f;
  float velVar = recovered ? 25.0f : 4.0f;
  float biasVar = 0.25f;
  for (uint8_t r = 0; r < 3; ++r)
    for (uint8_t c = 0; c < 3; ++c)
      fusionP[r][c] = 0.0f;
  fusionP[0][0] = posVar;
  fusionP[1][1] = velVar;
  fusionP[2][2] = biasVar;

  hasPositionReference = true;
  pointAtTrackS(fusionS_m, refLat, refLon);
}

void fusionPredict(float measuredAccel_mps2, float dt) {
  if (!fusionInitialized || dt <= 0.0f || dt > 0.20f) return;

  float correctedAccel = measuredAccel_mps2 - fusionBias_mps2;
  float dt2 = dt * dt;
  fusionS_m += fusionV_mps * dt + 0.5f * correctedAccel * dt2;
  fusionV_mps += correctedAccel * dt;

  if (fusionS_m < 0.0f) { fusionS_m = 0.0f; if (fusionV_mps < 0.0f) fusionV_mps = 0.0f; }
  if (fusionS_m > trackTotalM) { fusionS_m = trackTotalM; if (fusionV_mps > 0.0f) fusionV_mps = 0.0f; }
  fusionV_mps = fminf(fmaxf(fusionV_mps, -FUSION_MAX_SPEED_MPS), FUSION_MAX_SPEED_MPS);

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
        FP[i][j] += F[i][k] * fusionP[k][j];

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
      fusionP[i][j] = Pnew[i][j];
}

void fusionUpdateScalar(uint8_t stateIndex, float measurement, float variance) {
  if (!fusionInitialized || stateIndex > 2 || variance <= 0.0f) return;

  float state[3] = {fusionS_m, fusionV_mps, fusionBias_mps2};
  float innovation = measurement - state[stateIndex];
  float S = fusionP[stateIndex][stateIndex] + variance;
  if (S < 1.0e-6f) return;

  float K[3];
  float selectedRow[3];
  for (uint8_t i = 0; i < 3; ++i) {
    K[i] = fusionP[i][stateIndex] / S;
    selectedRow[i] = fusionP[stateIndex][i];
  }

  state[0] += K[0] * innovation;
  state[1] += K[1] * innovation;
  state[2] += K[2] * innovation;

  float Pnew[3][3];
  for (uint8_t i = 0; i < 3; ++i)
    for (uint8_t j = 0; j < 3; ++j)
      Pnew[i][j] = fusionP[i][j] - K[i] * selectedRow[j];

  // Fuerza simetria numerica y limita diagonales a valores positivos.
  for (uint8_t i = 0; i < 3; ++i) {
    for (uint8_t j = 0; j < 3; ++j) {
      fusionP[i][j] = 0.5f * (Pnew[i][j] + Pnew[j][i]);
    }
    if (fusionP[i][i] < 1.0e-6f) fusionP[i][i] = 1.0e-6f;
  }

  fusionS_m = fminf(fmaxf(state[0], 0.0f), trackTotalM);
  fusionV_mps = fminf(fmaxf(state[1], -FUSION_MAX_SPEED_MPS), FUSION_MAX_SPEED_MPS);
  fusionBias_mps2 = fminf(fmaxf(state[2], -1.5f), 1.5f);
}

void fusionUpdatePosition(float s_m, float sigma_m) {
  lastPositionInnovation_m = fusionInitialized ? s_m - fusionS_m : 0.0f;
  fusionUpdateScalar(0, s_m, sigma_m * sigma_m);
}

void fusionUpdateSpeed(float v_mps, float sigma_mps) {
  lastSpeedInnovation_mps = fusionInitialized ? v_mps - fusionV_mps : 0.0f;
  fusionUpdateScalar(1, v_mps, sigma_mps * sigma_mps);
}

// ============================================================================
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
  float pitchRad = icmPitch_deg * PI / 180.0f;
  icmLongZero_mps2 = ax * GRAVITY_MPS2 + GRAVITY_MPS2 * sinf(pitchRad);
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

  float pitchRad = icmPitch_deg * PI / 180.0f;
  float bodyLinearForward = icmBodyAx_g * GRAVITY_MPS2 +
                            GRAVITY_MPS2 * sinf(pitchRad) -
                            icmLongZero_mps2;
  icmTrackAccelRaw_mps2 = bodyLinearForward;

  float rc = 1.0f / (2.0f * PI * NAV_ACCEL_LPF_HZ);
  float alphaAcc = dt / (rc + dt);
  icmTrackAccel_mps2 += alphaAcc *
                        (icmTrackAccelRaw_mps2 - icmTrackAccel_mps2);
  if (fabsf(icmTrackAccel_mps2) < ACC_DEADBAND_MPS2)
    icmTrackAccel_mps2 = 0.0f;

  if (fusionInitialized) fusionPredict(icmTrackAccel_mps2, dt);
  (void)nowMs;
  return true;
}

// ============================================================================
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
    gpsHeading_deg = myGPS.getHeading() / 100000.0f;
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

    gpsTrackProjection = projectToTrack(gpsLat, gpsLon);
    if (gpsTrackProjection.valid) {
      gpsTrackS_m = gpsTrackProjection.s_m;
      gpsCrossTrack_m = gpsTrackProjection.crossTrack_m;
      gpsTrackSpeedSigned_mps =
          gpsVelE_mps * gpsTrackProjection.tangentE +
          gpsVelN_mps * gpsTrackProjection.tangentN;
      gpsSpeed_mps = fabsf(gpsTrackSpeedSigned_mps);
      nearestStation = nearestStationForS(gpsTrackS_m);
    } else {
      gpsCrossTrack_m = 9999.0f;
      gpsTrackSpeedSigned_mps = 0.0f;
      gpsSpeed_mps = gpsGroundSpeed_mps;
    }

    bool fixOK = gpsFixType >= 3 && gpsGnssFixOk;
    bool coordinateOK = coordinatesAreValid(gpsLat, gpsLon) && !gpsInvalidLlh;
    bool satellitesOK = gpsSats >= MIN_GNSS_SATS;
    bool hAccOK = gpsHAcc_mm <= HACC_LIMIT_MM;
    bool sAccOK = gpsSAcc_mmps <= SACC_LIMIT_MMPS;
    bool pdopOK = gpsPDOP_centi <= PDOP_LIMIT_CENTI;
    bool mapOK = gpsTrackProjection.valid &&
                 gpsCrossTrack_m <= MAP_MAX_CROSSTRACK_M;
    bool rtkOK = !REQUIRE_RTK_FOR_POSITION || gpsCarrierSolution > 0;

    gpsFixRaw = fixOK;
    gpsQualityCurrent = fixOK && coordinateOK && satellitesOK && hAccOK &&
                        sAccOK && pdopOK && mapOK && rtkOK;

    updateHeadingCalibrationFromGps(now);

    if (gpsQualityCurrent) {
      lastGpsValidMs = now;
      referenceRecoveredFromSd = false;
      recoveredReferenceMs = 0;
      refLat = gpsTrackProjection.lat;
      refLon = gpsTrackProjection.lon;
      hasPositionReference = true;

      updateDirectionFromTrack(now, gpsTrackS_m, gpsTrackSpeedSigned_mps);

      if (!fusionInitialized) {
        fusionInitialize(gpsTrackS_m, gpsTrackSpeedSigned_mps, false);
      } else {
        float sigmaPos = fmaxf(gpsHAcc_mm / 1000.0f, 0.30f);
        if (gpsCarrierSolution == 2) sigmaPos = fmaxf(sigmaPos, 0.05f);
        else if (gpsCarrierSolution == 1) sigmaPos = fmaxf(sigmaPos, 0.20f);
        float sigmaSpeed = fmaxf(gpsSAcc_mmps / 1000.0f, 0.05f);

        // Guardar innovaciones antes de actualizar permite diagnosticar deriva.
        lastPositionInnovation_m = gpsTrackS_m - fusionS_m;
        lastSpeedInnovation_mps = gpsTrackSpeedSigned_mps - fusionV_mps;
        fusionUpdatePosition(gpsTrackS_m, sigmaPos);
        fusionUpdateSpeed(gpsTrackSpeedSigned_mps, sigmaSpeed);
      }
    }
  }

  gpsUsable = gpsQualityCurrent &&
              !elapsed(now, lastPvtMs, GPS_RECENT_MS) &&
              !elapsed(now, lastGpsValidMs, GPS_RECENT_MS);

  if (gpsUsable && !previousGpsUsable) {
    lastPosSavedFromCurrentFix = false;
  }
}

// ============================================================================
// ZUPT, AJUSTE LENTO Y SALUD DE LA IMU
// ============================================================================
void applyStillnessCorrection(uint32_t now) {
  bool icmStill = icmPresent &&
                  icmMag_g >= STILL_MAG_MIN_G && icmMag_g <= STILL_MAG_MAX_G &&
                  fabsf(icmBodyGx_dps) <= STILL_GYRO_MAX_DPS &&
                  fabsf(icmBodyGy_dps) <= STILL_GYRO_MAX_DPS &&
                  fabsf(icmBodyGz_dps) <= STILL_GYRO_MAX_DPS;
  bool mpuStill = mpuPresent &&
                  mpuMag_g >= STILL_MAG_MIN_G && mpuMag_g <= STILL_MAG_MAX_G &&
                  vibMagLin_g <= STILL_VIB_MAX_G;
  bool externalStill = gpsUsable && gpsSpeed_mps < GPS_STILL_MPS;

#if ENABLE_WHEEL_ODOMETRY
  externalStill = externalStill || wheelSpeed_mps < GPS_STILL_MPS;
#endif

  if (externalStill && icmStill && mpuStill) {
    if (stillStartMs == 0) stillStartMs = now;

    if (elapsed(now, stillStartMs, STILL_HOLD_MS)) {
      if (fusionInitialized) fusionUpdateSpeed(0.0f, 0.03f);

      // Ajuste muy lento solo bajo una referencia externa de velocidad cero.
      icmGyroBiasSensorX_dps += BIAS_ADAPT_ALPHA *
          (icmRawGyrX_dps - icmGyroBiasSensorX_dps);
      icmGyroBiasSensorY_dps += BIAS_ADAPT_ALPHA *
          (icmRawGyrY_dps - icmGyroBiasSensorY_dps);
      icmGyroBiasSensorZ_dps += BIAS_ADAPT_ALPHA *
          (icmRawGyrZ_dps - icmGyroBiasSensorZ_dps);
      icmLongZero_mps2 += BIAS_ADAPT_ALPHA * icmTrackAccelRaw_mps2;
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
                            fabsf(lastSpeedInnovation_mps) >
                            IMU_SPEED_INNOV_LIMIT_MPS;
  if (speedInnovationBad) {
    if (icmDriftStartMs == 0) icmDriftStartMs = now;
  } else {
    icmDriftStartMs = 0;
  }

  bool stillDisagreement = gpsUsable && gpsSpeed_mps < GPS_STILL_MPS &&
                           fabsf(icmTrackAccel_mps2) >
                           IMU_STILL_ACCEL_LIMIT_MPS2;
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

  float magnitude = delta * WHEEL_METERS_PER_PULSE / (dtMs / 1000.0f);
  wheelSpeed_mps = magnitude;
  float signedSpeed = dirNS == 1 ? magnitude : -magnitude;
  if (fusionInitialized) fusionUpdateSpeed(signedSpeed, WHEEL_SPEED_SIGMA_MPS);
}
#endif

// ============================================================================
// SELECCION DE FUENTE FINAL
// ============================================================================
void updateFinalPosition(uint32_t now) {
  if (fusionInitialized) {
    pointAtTrackS(fusionS_m, fusedLat, fusedLon);
    nearestStation = nearestStationForS(fusionS_m);
  } else {
    fusedLat = fusedLon = 0.0;
  }

  if (gpsUsable && fusionInitialized) {
    txSource = SRC_MAP_GNSS;
    txLat = fusedLat;
    txLon = fusedLon;
    txSpeedKmh = fabsf(fusionV_mps) * 3.6f;
  } else if (fusionInitialized && icmPresent &&
             !elapsed(now, lastGpsValidMs, LAST_VALID_MAX_AGE_MS)) {
    txSource = SRC_MAP_DR;
    txLat = fusedLat;
    txLon = fusedLon;
    txSpeedKmh = fabsf(fusionV_mps) * 3.6f;
  } else {
    txSource = SRC_NONE;
    txLat = 0.0;
    txLon = 0.0;
    txSpeedKmh = 0.0f;
  }

  if (fabsf(txSpeedKmh) < 0.8f) txSpeedKmh = 0.0f;
  digitalWrite(LED_GPSFAIL, gpsUsable ? LOW : HIGH);
}

// ============================================================================
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
      "date,time,millis,axLin_g,ayLin_g,azLin_g,magLin_g,"
      "gpsUsable,sats,speedKmh,dirTerminal,source,imuHealth,"
      "trackS_m,crossTrack_m,station,roll_deg,pitch_deg,aTrack_mps2,"
      "rtk,pdop,sAcc_mps,icmFloorCal,icmHeadingCal,mpuFloorCal,"
      "mpuHeadingCal,trackHeadingTrue_deg,icmCalQ,mpuCalQ"
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
  vibFile.print(dirNS); vibFile.print(',');
  vibFile.print((int)txSource); vibFile.print(',');
  vibFile.print((int)imuHealth); vibFile.print(',');
  vibFile.print(fusionInitialized ? fusionS_m : -1.0f, 2); vibFile.print(',');
  vibFile.print(gpsTrackProjection.valid ? gpsCrossTrack_m : -1.0f, 2); vibFile.print(',');
  vibFile.print(TRACK_POINTS[nearestStation].name); vibFile.print(',');
  vibFile.print(icmRoll_deg, 2); vibFile.print(',');
  vibFile.print(icmPitch_deg, 2); vibFile.print(',');
  vibFile.print(icmTrackAccel_mps2, 4); vibFile.print(',');
  vibFile.print(gpsCarrierSolution); vibFile.print(',');
  vibFile.print(gpsPDOP_centi / 100.0f, 2); vibFile.print(',');
  vibFile.print(gpsSAcc_mmps / 1000.0f, 3); vibFile.print(',');
  vibFile.print(icmAlignment.floorValid ? 1 : 0); vibFile.print(',');
  vibFile.print(icmAlignment.headingValid ? 1 : 0); vibFile.print(',');
  vibFile.print(mpuAlignment.floorValid ? 1 : 0); vibFile.print(',');
  vibFile.print(mpuAlignment.headingValid ? 1 : 0); vibFile.print(',');
  vibFile.print(currentTrackHeadingTrue_deg, 2); vibFile.print(',');
  vibFile.print(icmAlignment.headingQuality, 3); vibFile.print(',');
  vibFile.println(mpuAlignment.headingQuality, 3);

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
                           uint32_t &hAcc, uint8_t &fixType, int &direction,
                           uint16_t &year, uint8_t &month, uint8_t &day,
                           uint8_t &hour, uint8_t &minute, uint8_t &second,
                           float &trackS, bool &hasTrackS) {
  String value[12];
  int count = 0;
  int start = 0;
  while (count < 12 && start <= (int)line.length()) {
    int comma = line.indexOf(',', start);
    if (comma < 0) comma = line.length();
    value[count] = line.substring(start, comma);
    value[count].trim();
    count++;
    if (comma >= (int)line.length()) break;
    start = comma + 1;
  }
  if (count < 11) return false;

  lat = atof(value[0].c_str());
  lon = atof(value[1].c_str());
  hAcc = (uint32_t)strtoul(value[2].c_str(), nullptr, 10);
  fixType = (uint8_t)value[3].toInt();
  direction = value[4].toInt();
  year = (uint16_t)value[5].toInt();
  month = (uint8_t)value[6].toInt();
  day = (uint8_t)value[7].toInt();
  hour = (uint8_t)value[8].toInt();
  minute = (uint8_t)value[9].toInt();
  second = (uint8_t)value[10].toInt();
  hasTrackS = count >= 12 && value[11].length() > 0;
  trackS = hasTrackS ? atof(value[11].c_str()) : 0.0f;

  return coordinatesAreValid(lat, lon) && fixType >= 3 &&
         hAcc <= HACC_LIMIT_MM &&
         (direction == 0 || direction == 1);
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
  int direction = 0;
  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
  float storedS = 0.0f;
  bool hasStoredS = false;

  if (!parseLastPositionLine(line, lat, lon, hAcc, fixType, direction,
                             year, month, day, hour, minute, second,
                             storedS, hasStoredS)) {
    Serial.print("WARN: contenido invalido en ");
    Serial.println(filename);
    return false;
  }

  TrackProjection recovered = projectToTrack(lat, lon);
  if (!recovered.valid) return false;
  float s = hasStoredS ? storedS : recovered.s_m;
  if (s < 0.0f || s > trackTotalM) s = recovered.s_m;

  refLat = recovered.lat;
  refLon = recovered.lon;
  hasPositionReference = true;
  referenceRecoveredFromSd = true;
  recoveredReferenceMs = millis();
  lastGpsValidMs = recoveredReferenceMs;
  dirNS = direction;
  directionKnown = true;
  fusionInitialize(s, 0.0f, true);
  nearestStation = nearestStationForS(s);

  noteSdSuccess();
  Serial.print("Ultima posicion recuperada de ");
  Serial.print(filename);
  Serial.print("; s=");
  Serial.print(s, 1);
  Serial.print(" m, estacion cercana=");
  Serial.println(TRACK_POINTS[nearestStation].name);
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
  if (!sdOK || !gpsUsable || !gpsTrackProjection.valid) return false;

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
  // R9: latAjustada,lonAjustada,hAcc,fix,dir,fecha,hora,sSobreVia.
  file.print(gpsTrackProjection.lat, 7); file.print(',');
  file.print(gpsTrackProjection.lon, 7); file.print(',');
  file.print(gpsHAcc_mm); file.print(',');
  file.print(gpsFixType); file.print(',');
  file.print(dirNS); file.print(',');
  file.print(gpsTimeValid ? gpsYear : 0); file.print(',');
  file.print(gpsTimeValid ? gpsMonth : 0); file.print(',');
  file.print(gpsTimeValid ? gpsDay : 0); file.print(',');
  file.print(gpsTimeValid ? gpsHour : 0); file.print(',');
  file.print(gpsTimeValid ? gpsMinute : 0); file.print(',');
  file.print(gpsTimeValid ? gpsSecond : 0); file.print(',');
  file.println(gpsTrackS_m, 2);
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

  uint64_t used = 0, total = 0;
  uint8_t percent = 0;
  if (!readSdUsage(used, total, percent)) {
    // SDFS.info es la comprobacion directa del montaje. Si falla, se declara la
    // tarjeta fuera de servicio de inmediato, aunque un File aun tenga buffer.
    noteSdFailure("comprobacion periodica SDFS.info fallo", true);
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
  display.print(" D:"); display.print(directionName());
  display.print(" H:"); display.println((int)imuHealth);

  display.setCursor(0, 11);
  display.print("ST:"); display.println(TRACK_POINTS[nearestStation].name);

  display.setCursor(0, 21);
  display.print("Lat:"); display.println((float)txLat, 6);
  display.setCursor(0, 31);
  display.print("Lon:"); display.println((float)txLon, 6);

  display.setCursor(0, 41);
  display.print("V:"); display.print(txSpeedKmh, 1);
  display.print(" F:"); display.print(gpsFixType);
  display.print(" R:"); display.println(gpsCarrierSolution);

  display.setCursor(0, 52);
  display.print("XTE:");
  if (gpsTrackProjection.valid) display.print(gpsCrossTrack_m, 0);
  else display.print("--");
  display.print(" SD:"); display.print(sdOK ? "OK" : "F");
  display.print(" P:"); display.print(havePending ? "Y" : "N");

  display.display();
}

// ============================================================================
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

  String status = "SRC_" + String((int)txSource) +
                  "_IMU_" + String((int)imuHealth) +
                  "_ST_" + String((int)nearestStation) +
                  "_XTE_" + String(gpsTrackProjection.valid ?
                                     gpsCrossTrack_m : -1.0f, 0) +
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
                "&field4=" + String(dirNS) +
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
  initializeTrackModel();

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
    Serial.println("MPU6500 OK: +/-4 g, lectura 50 Hz, log 20 Hz, autocal activa.");
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

  // Guarda el piso recalculado; el rumbo GNSS se agregara durante la marcha.
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
void loop() {
  uint32_t now = millis();

  serviceLedPulses();
  serviceSdManager(now);

  if (elapsed(now, lastHeartbeatMs, 500)) {
    lastHeartbeatMs = now;
    heartbeatState = !heartbeatState;
    digitalWrite(LED_ONBOARD, heartbeatState);
    digitalWrite(LED_HEART_EXT, heartbeatState);
  }

  previousGpsUsable = gpsUsable;

  // La IMU predice a alta frecuencia; GNSS y odometria corrigen el filtro.
  updateIcmSample(now);
  updateMpuSample(now);
  pollGnss(now);
  if (imuCalibrationDirty) saveImuCalibrationToSd(false);
#if ENABLE_WHEEL_ODOMETRY
  updateWheelOdometry(now);
#endif

  applyStillnessCorrection(now);
  evaluateImuHealth(now);
  updateFinalPosition(now);

  saveLastPositionIfDue(now);
  writeVibrationLog(now);
  manageVibrationStorage(now);

  lteStep();

  if (elapsed(now, lastOledMs, OLED_INTERVAL_MS)) {
    lastOledMs = now;
    updateOled();
  }

  delay(2);
}
