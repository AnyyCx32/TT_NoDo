// ---------------------------------------------------------------------------
//  R11_LEDTEST - verificacion de indicadores uno por uno
//
//  Enciende un solo LED a la vez y lo mantiene encendido hasta el siguiente
//  comando. Sirve para confirmar la correspondencia entre cada pin y el
//  indicador fisico sin depender de la cadencia de una secuencia automatica.
//
//  Los pines son los mismos de ProgramaFinal_R11_General_ENU.ino y R11_DIAG.
//  Consola a 115200. Escribir la cifra y ENTER.
// ---------------------------------------------------------------------------

#define LED_ONBOARD    25
#define LED_HEART_EXT  15
#define LED_SD         14
#define LED_GPSFAIL     3
#define LED_TS_OK       2

struct Indicador {
  uint8_t     pin;
  const char *nombre;
};

const Indicador LEDS[] = {
  { LED_ONBOARD,   "GP25  latido interno (LED de la placa)" },
  { LED_HEART_EXT, "GP15  latido externo" },
  { LED_SD,        "GP14  actividad microSD" },
  { LED_GPSFAIL,   "GP3   falla GNSS" },
  { LED_TS_OK,     "GP2   envio ThingSpeak" },
};
const uint8_t N_LEDS = sizeof(LEDS) / sizeof(LEDS[0]);

void apagarTodos() {
  for (uint8_t i = 0; i < N_LEDS; ++i) digitalWrite(LEDS[i].pin, LOW);
}

void menu() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("  R11_LEDTEST - un indicador a la vez");
  Serial.println("============================================================");
  for (uint8_t i = 0; i < N_LEDS; ++i) {
    Serial.print("    ");
    Serial.print(i + 1);
    Serial.print("  ");
    Serial.println(LEDS[i].nombre);
  }
  Serial.println("    0  apagar todos");
  Serial.println("    p  parpadear el encendido actual (5 veces)");
  Serial.println("    ?  volver a mostrar este menu");
  Serial.println();
}

int8_t actual = -1;   // indice del LED encendido, -1 si ninguno

void encender(uint8_t idx) {
  apagarTodos();
  digitalWrite(LEDS[idx].pin, HIGH);
  actual = idx;
  Serial.print("  ENCENDIDO -> ");
  Serial.println(LEDS[idx].nombre);
  Serial.println("  Los otros cuatro deben estar apagados.");
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N_LEDS; ++i) {
    pinMode(LEDS[i].pin, OUTPUT);
    digitalWrite(LEDS[i].pin, LOW);
  }
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }
  menu();
  Serial.println("  Todos apagados. Esperando comando.");
}

void loop() {
  if (!Serial.available()) return;

  int c = Serial.read();
  if (c == '\r' || c == '\n' || c == ' ') return;

  if (c >= '1' && c <= '0' + N_LEDS) {
    encender((uint8_t)(c - '1'));
    return;
  }

  switch (c) {
    case '0':
      apagarTodos();
      actual = -1;
      Serial.println("  Todos apagados.");
      break;

    case 'p':
    case 'P':
      if (actual < 0) {
        Serial.println("  No hay ninguno encendido.");
        break;
      }
      Serial.print("  Parpadeando ");
      Serial.println(LEDS[actual].nombre);
      for (uint8_t k = 0; k < 5; ++k) {
        digitalWrite(LEDS[actual].pin, LOW);  delay(250);
        digitalWrite(LEDS[actual].pin, HIGH); delay(250);
      }
      break;

    case '?':
      menu();
      break;

    default:
      Serial.print("  Comando no reconocido: ");
      Serial.println((char)c);
      break;
  }
}
