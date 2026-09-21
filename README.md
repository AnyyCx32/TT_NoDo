# Nodo TT-R11 — Sistema de seguimiento y localización para tren ligero

Firmware y bancos de prueba del nodo de telemetría desarrollado para el Trabajo Terminal
TTM-2024/2-44 de la Unidad Profesional Interdisciplinaria de Ingeniería y Tecnologías
Avanzadas del Instituto Politécnico Nacional.

El nodo adquiere posición GNSS y vibración, fusiona ambas con una unidad inercial para
mantener la posición cuando el satélite no está disponible, registra todo en una microSD a
20 Hz, y transmite por LTE a un canal de ThingSpeak que alimenta una interfaz web.

> **Nota sobre credenciales.** La clave de escritura de ThingSpeak está sustituida por
> `"AQUI_TU_CLAVE_DE_ESCRITURA"` en todos los sketches publicados aquí. Es la única
> diferencia respecto al código que corrió en el prototipo. Para compilar hay que
> reponerla en la constante `API_KEY` de cada sketch que transmita.

---

## 1. Qué hay en este repositorio

```
firmware/
  operacion/     ProgramaFinal_R11_General_ENU.ino    el firmware de servicio
  variantes/     R11_DEMO_TS/      demostración de cadencia contra ThingSpeak
                 R11_TEST_MODEM/   prueba de la reanimación del módem
  bancos/        R11_DIAG/         diagnóstico completo por consola USB
                 SD_TEST/          banco dedicado a la microSD
                 R11_PROFILE/      perfilado del lazo principal
                 R11_LEDTEST/      indicadores locales
                 MPU6500_TEST/     banco del acelerómetro
  historico/     ProgramaFinal_R10_AutoCal_GPS_3m.ino  versión anterior, para referencia
herramientas/    scripts de consola y de análisis
docs/            documentación por tema
```

Las variantes y los bancos **reemplazan** al firmware de operación cuando se cargan. Para
devolver el nodo a servicio hay que volver a cargar `firmware/operacion/`.

## 2. Plataforma y conexiones

Microcontrolador **RP2040** (Raspberry Pi Pico), núcleo `rp2040` 5.7.0 del proyecto
arduino-pico.

| Función | Pin | Notas |
|---|---|---|
| Módem SIM7600G-H | GP0 TX, GP1 RX | `Serial1` a 115200, FIFO de recepción ampliada a 1024 B |
| PWRKEY del módem | GP6 | conmutador, no interruptor; ver §5.2 |
| Bus I2C | GP4 SDA, GP5 SCL | ZED-F9P, ICM-20948, MPU6500 y OLED |
| microSD (SPI) | GP17 CS, GP16 MISO, GP18 SCK, GP19 MOSI | |
| Indicadores | GP15 latido, GP14 SD, GP3 GPSFAIL, GP2 TS_OK | |
| Vigilancia de alimentación | GP29 / A3 | VSYS dividido entre 3 |

## 3. Arquitectura del firmware de operación

Todo corre en un **único lazo no bloqueante**. No hay hilos ni interrupciones salvo la del
odómetro opcional. Cada subsistema se atiende por turnos y ninguno puede detener a los
demás; ésa es la propiedad que hace posible que el nodo siga adquiriendo mientras el módem
espera una respuesta o mientras la microSD está ausente.

**Adquisición.** El MPU6500 se lee a 50 Hz y se registra a 20 Hz. El ZED-F9P entrega
UBX-NAV-PVT por I2C a 5 Hz. La ICM-20948 aporta actitud y aceleración a ~100 Hz.

**Fusión y selección de fuente.** La posición publicada proviene de una de tres fuentes,
en este orden de preferencia:

| Fuente | Condición |
|---|---|
| `SRC_GNSS_FUSED` | hay fijo utilizable (`hAcc ≤ 10 m`) y la fusión está inicializada |
| `SRC_GENERAL_DR` | no hay fijo, pero la última referencia válida tiene menos de `LAST_VALID_MAX_AGE_MS` (120 s) |
| `SRC_NONE` | ninguna de las anteriores |

Con `SRC_NONE` el nodo **no arma paquete y no transmite**. Es una decisión de diseño, no
una falla: sin coordenadas válidas no hay telemetría que enviar. Explica el silencio de
radio bajo tierra y en interiores.

El umbral de 120 s está marcado `// validar experimentalmente` y sigue sin validar.
Determinarlo a partir del crecimiento real del error es el objetivo de la prueba P-07.

**Detección de reposo y ZUPT.** La corrección de velocidad cero exige acuerdo de tres
condiciones independientes. La banda de reposo se centra en la gravedad que **cada sensor**
mide durante la calibración de piso del arranque, no en un valor absoluto; el porqué está
en §5.1.

**Persistencia.** Cuatro archivos en la raíz de la tarjeta:

| Archivo | Contenido |
|---|---|
| `VIB_YYYYMMDD.CSV` | registro a 20 Hz, 43 columnas |
| `EVENTOS.LOG` | bitácora fechada: arranque, calibración, cada envío, estado cada 30 s, resumen cada 5 min |
| `LASTPOS.TXT` | última posición válida; da origen de coordenadas al arrancar sin GNSS |
| `IMUCAL.BIN` | calibración inercial, 84 B con magic, versión, tamaño y suma FNV-1a |
| `PENDING.TXT` | un único paquete no confirmado |

**Transmisión.** Máquina de estados de 33 pasos sobre comandos AT, un paso por vuelta de
lazo. El paquete se escribe en `PENDING.TXT` **antes** de intentar enviarlo y se borra al
confirmar el 200, de modo que un corte de energía a media transmisión no pierde la muestra.

## 4. El periodo de transmisión lo impone ThingSpeak

Es el resultado que más condiciona la lectura del sistema, así que conviene que quede
explícito.

| | Medido |
|---|---|
| Tiempo de una transacción completa del nodo | **1.03 s** de mediana (n=115) |
| Mínimo intervalo que ThingSpeak acepta | **15 s**; observado 16 s, 0 de 166 por debajo |
| Periodo efectivo configurado | 15.8 s aproximados |

El nodo es capaz de transmitir en torno a un quinceavo del periodo con que opera. **La
cadencia de la telemetría está impuesta por el plan gratuito del servicio externo, no por
el prototipo.** La variante `R11_DEMO_TS` existe para demostrarlo: fuerza el periodo a 2 s
y permite comparar los envíos del nodo contra las entradas que el canal llega a registrar.

`SEND_PERIOD_MS` vale 14800 ms y no 15000 porque el temporizador se ancla en el instante
del `HTTP OK` anterior, de modo que al periodo se le suma la duración del ciclo siguiente.
Un piso duro independiente, `TS_MIN_GAP_MS = 15300`, se mide desde el último envío
**aceptado** y hace imposible por construcción que dos entregas queden a menos de 15 s.
Importa porque ThingSpeak no señala el rechazo con un error: responde 200 con el cuerpo
`0` y simplemente no crea la entrada.

## 5. Defectos hallados y corregidos

La documentación detallada está en [`docs/defectos.md`](docs/defectos.md). Resumen de los
que cambiaron el comportamiento del sistema:

### 5.1 De la campaña de agosto de 2026

- **Banda de reposo referida a la gravedad absoluta.** El MPU6500 de este nodo mide
  1.0998 g en reposo por sesgo de fábrica, fuera de la banda 0.92–1.08 g que exigía el
  código. La condición de reposo era permanentemente falsa y **el ZUPT nunca podía
  aplicarse**. Parecía un acelerómetro defectuoso.
- **Sonda de salud de la microSD.** `SDFS.info()` cada 5 s tardaba 6151 ms en la tarjeta
  de 15 GB y se realimentaba. El registro caía a 0.303 Hz contra 20 Hz nominales. Parecía
  falla de GNSS, porque el receptor desbordaba su buffer I2C entre lecturas separadas 6.2 s.
- **FIFO de recepción del módem.** 32 bytes se llenan en 2.8 ms a 115200 baudios y el lazo
  se bloquea rutinariamente más que eso. Se perdían respuestas a media línea. Parecía falta
  de cobertura.

Los tres compartían una característica que conviene recordar: **el síntoma apuntaba al
subsistema equivocado**. Perfilar el lazo antes de sospechar del hardware.

### 5.2 De la campaña de septiembre de 2026

- **Un `HTTPACTION` que expira deja al SIM7600 mudo.** Deja de responder incluso a un `AT`
  simple. El 20 de septiembre produjo dos huecos de 5.7 y 7.1 minutos; el segundo sólo
  terminó cuando el nodo perdió energía. No existía ninguna ruta de recuperación: `AT+CFUN`
  y `AT+CRESET` viajan por el mismo puerto que dejó de responder. Se añadió una reanimación
  por **PWRKEY**, que es una línea física, integrada en la máquina de estados para no
  bloquear el lazo.

  PWRKEY **conmuta**: el mismo pulso que enciende un módem apagado apaga uno encendido. Por
  eso la secuencia es *pulsar, preguntar, y sólo si sigue mudo volver a pulsar*, nunca dos
  pulsos encadenados.

- **Relleno en el campo de rumbo.** `String(float, 0)` de Arduino formatea con `dtostrf` y
  un ancho mínimo de dos caracteres, así que un rumbo de un solo dígito viajaba como
  `_HDG_ 4_`, con un espacio. La plataforma no reconocía el campo y descartaba la fuente y
  el estado de la IMU de esa entrada completa. Afectó a 3 de 167 entradas.

- **La bandera de calibración medía otra cosa.** `calibracionCargada` reportaba si se había
  recuperado el **rumbo**, no si se había cargado la calibración, y el piso sí se restauraba
  siempre. Se separó en `calibracionPiso` y `calibracionRumbo`.

### 5.3 Defectos conocidos, sin corregir

- **El guardado sobrescribe una calibración buena con una peor.** Si un sensor falta al
  arrancar, el rumbo persistido no se puede reconstruir y la rutina reescribe `IMUCAL.BIN`
  sin él. Debería conservar lo persistido cuando no puede mejorarlo.
- **La reanimación del módem no está demostrada causalmente.** El disparador y la secuencia
  se verificaron; que el pulso sea lo que cura al módem no, porque la tarjeta de adaptación
  lo vuelve a encender sola unos 28–30 s después de un apagado, y ese tiempo enmascara el
  efecto del pulso.

## 6. Cómo compilar y cargar

No hace falta abrir el IDE. El `arduino-cli` que trae el Arduino IDE sirve, pasándole la
configuración del IDE para que encuentre las librerías:

```powershell
$cli = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$cfg = "$env:USERPROFILE\.arduinoIDE\arduino-cli.yaml"
& $cli --config-file $cfg compile --fqbn rp2040:rp2040:rpipico <carpeta del sketch>
& $cli --config-file $cfg upload -p <puerto> --fqbn rp2040:rp2040:rpipico <carpeta del sketch>
```

Dos trampas:

1. **`arduino-cli` exige que la carpeta se llame igual que el `.ino`.** El firmware de
   operación vive suelto; hay que copiarlo a una carpeta con el nombre correcto.
2. **El número de puerto cambia entre placas.** Listar antes de cargar y confirmar
   `VID_2E8A`; no darlo por sentado.

La carga usa el reinicio por USB y monta la unidad `RPI-RP2`; no hace falta tocar BOOTSEL.

## 7. Procedimientos de operación aprendidos

- **Si faltan sensores del bus I2C al arrancar, cortar la energía por completo.** Un
  reinicio por software no libera a un esclavo colgado: la recuperación del bus que trae el
  firmware sólo actúa sobre una línea SDA retenida. Se observó el 21 de septiembre de 2026,
  con el ZED-F9P y la ICM-20948 ausentes tras varias recargas seguidas; el corte los
  devolvió a los dos.
- **Encender siempre con el nodo quieto.** Los primeros 11 segundos son calibración de
  piso. Si hay movimiento, el piso se rechaza y el firmware cae a la banda de reposo
  absoluta de respaldo, que en este nodo deja el ZUPT inoperante durante toda la sesión.
- **Nunca extraer la microSD con el equipo encendido.** La reinserción en caliente sí es
  segura y el firmware la detecta en menos de 5 s.
- **Un `vsysMin` bajo aislado es un artefacto del ADC**, no una caída del riel; sólo la
  línea `ALERTA alimentacion baja`, que exige cinco muestras consecutivas, indica un
  hundimiento real.
- **Un `+HTTPACTION: 1,302,14` es el portal del operador**, no ThingSpeak: significa línea
  sin saldo. El módem se ve sano y el contexto de datos se activa igual.

## 8. Herramientas

| Script | Para qué |
|---|---|
| `analizar_vib.py` | procesa un `VIB_*.CSV` y calcula lo que piden las fichas de adquisición y fusión |
| `diag.ps1` | maneja el banco `R11_DIAG` por menú de una letra |
| `sd.ps1` | maneja el banco `SD_TEST` |
| `capturar_P18.ps1` | transcribe la consola para medir periodicidad de transmisión |
| `probe_*.py` | sondas de continuidad, pull-ups y barrido del bus I2C |

Al procesar un CSV conviene **filtrar por `gpsUsable` antes de calcular dispersión**: las
columnas `lat` y `lon` se copian del receptor sin comprobar validez, así que los tramos sin
fijo traen coordenadas preliminares a miles de kilómetros.

## 9. Licencia y autoría

Trabajo Terminal TTM-2024/2-44. Los derechos corresponden a sus autores y al Instituto
Politécnico Nacional.
