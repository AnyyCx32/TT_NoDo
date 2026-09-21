# Catálogo de defectos

Cada entrada anota el síntoma tal como se presentó, la causa medida y la corrección. El
orden es cronológico.

Conviene leerlo con una advertencia por delante: **en este nodo los síntomas apuntan
sistemáticamente al subsistema equivocado**. Tres de los defectos incapacitantes se
manifestaron como fallas de un componente que estaba sano. Perfilar el lazo antes de
sospechar del hardware ahorra días.

---

## Campaña del 26 de agosto de 2026

### M1 — La banda de reposo impedía el ZUPT

**Síntoma.** La corrección de velocidad cero no se aplicaba nunca. Parecía un acelerómetro
defectuoso.

**Causa.** La detección de reposo comparaba la magnitud de la aceleración contra una banda
absoluta de 0.92 a 1.08 g. El MPU6500 de este prototipo mide **1.0998 g en reposo**, por un
sesgo de fábrica de aproximadamente +0.10 g en su eje Z. La condición era por tanto
permanentemente falsa, y como el ZUPT exige el acuerdo de tres condiciones, la corrección
no podía aplicarse jamás. Con ella se perdía además la adaptación del sesgo del giroscopio,
que vive en el mismo bloque.

**Corrección.** Cada sensor aprende su propia gravedad de referencia durante la calibración
de piso del arranque, que por procedimiento se ejecuta con el nodo detenido. La banda se
centra en ese valor con una tolerancia de 0.08 g. Si la calibración no llega a completarse,
se conserva la banda absoluta como respaldo.

**Verificación.** En la sesión registrada, la magnitud recorrió el intervalo 1.0890 a
1.1100 g: ninguna muestra caía dentro de la banda antigua. Con la corrección, el registro
muestra reposo detectado en 44.8 % de las muestras y ZUPT aplicado en 14.4 %.

### M4 — La sonda de salud de la microSD paralizaba el lazo

**Síntoma.** El GNSS dejaba de entregar tramas a los 26 s y sólo se producía una
transmisión LTE en siete minutos. Parecía un receptor GNSS muerto.

**Causa.** `serviceSdManager` llamaba a `SDFS.info()` cada 5 s para comprobar el montaje.
En la microSD de 15 GB esa llamada recorre la FAT y tarda **6151 ms**, más que el propio
intervalo de comprobación, de modo que se realimentaba y el lazo quedaba atrapado.

Consecuencias medidas, todas atribuibles a esa única causa:

| Efecto | Medido |
|---|---|
| Frecuencia de registro | 0.303 Hz contra 20 Hz nominales |
| Intervalo mediano entre muestras | 6191 ms contra 50 ms |
| Peor intervalo | 12 337 ms |
| Transmisiones LTE | una sola en 7 minutos |

El GNSS «muerto» era consecuencia: entre lecturas separadas 6.2 s el ZED-F9P desborda su
buffer I2C. Y la máquina de estados del LTE avanza un estado por vuelta, así que a 6.2 s
por vuelta un ciclo completo tardaba minutos.

**Corrección.** La comprobación de montaje se sustituyó por una consulta de directorio
sobre los archivos propios del nodo, que es barata. La ocupación real sigue midiéndose,
pero sólo cada 30 minutos.

**Verificación.** La etapa pasó de 6151 ms a **11.5 ms** máximos por vuelta.

### M6 — El bus I2C quedaba retenido tras un reinicio

**Síntoma.** El nodo arrancaba ciego y permanecía así. El refresco de la pantalla tardaba
1851.8 ms, equivalente a 37 accesos expirados seguidos.

**Causa.** Si el microcontrolador se reinicia en medio de una lectura, el esclavo puede
quedarse conduciendo SDA a masa esperando los pulsos de reloj que le faltan. `Wire.begin()`
no corrige esa condición y cada acceso expira en los 50 ms del tiempo de espera.

**Corrección.** Antes de inicializar el bus se comprueba el estado de SDA. Si está en bajo,
se emiten hasta nueve pulsos de reloj para que el esclavo termine su byte y libere la
línea, y se cierra con una condición de paro.

**Límite conocido.** Esta recuperación sólo actúa sobre una línea SDA retenida. Un esclavo
colgado que no responde en absoluto necesita perder alimentación; ver la nota de operación
correspondiente en el README.

### M7 — La FIFO del módem perdía respuestas a media línea

**Síntoma.** Respuestas truncadas del módem, como `+IP ERROR: Network is already` en lugar
de la línea completa. Parecía falta de cobertura.

**Causa.** `Serial1` operaba con la FIFO de recepción por omisión, de 32 bytes. A 115200
baudios se llena en **2.8 ms**, y el lazo se bloquea rutinariamente más que eso: el refresco
de la pantalla tarda 29 ms y una escritura en la microSD entre 9 y 11 ms.

**Corrección.** `Serial1.setFIFOSize(1024)` antes de `begin()`.

### M2, M3, M5 y M8 — Observabilidad y temporización

- **M2 y M3** añadieron al registro las banderas de reposo y ZUPT, la posición GNSS cruda,
  la precisión horizontal y la edad de la referencia. El CSV pasó de 33 a 41 columnas.
- **M5** espació la medición de ocupación de la tarjeta de 5 a 30 minutos.
- **M8** corrigió la deriva del temporizador de registro, que entregaba 17.6 Hz reales
  contra 20 nominales.

---

## Sesión de energía del 4 de septiembre de 2026

### El 302 del módem es el portal del operador

Un `+HTTPACTION: 1,302,14` **no** viene de ThingSpeak: es el portal de recarga de la
línea, que estaba sin saldo. El operador deja registrar y activar el contexto de datos
—`CGATT` y `CGACT` responden `OK` y el módem se ve sano— y desvía el HTTP en la capa de
aplicación.

Se distingue desde una computadora: una petición a la misma URL con clave inválida devuelve
400 con cuerpo de un byte, no un 302. **No migrar a HTTPS por este síntoma**; la URL en
texto plano es correcta.

### Coordenadas sin validar en el registro

`gpsLat` y `gpsLon` se copian de UBX-NAV-PVT sin filtrar por validez; las banderas sólo
alimentan la decisión de usabilidad. Sin fijo aparecen coordenadas preliminares a miles de
kilómetros, y esas mismas variables alimentan las columnas `lat` y `lon` del CSV sobre las
que se mide dispersión.

**Al procesar hay que filtrar por `gpsUsable`.** El script de análisis sólo descarta el par
(0,0).

---

## Campaña del 20 y 21 de septiembre de 2026

### Un HTTPACTION expirado deja al módem mudo

**Síntoma.** Rachas de decenas de fallos consecutivos con el ciclo cortado en
`LTE_WAIT_AT`, es decir esperando respuesta a un `AT` simple.

**Causa medida.** Siete episodios en un recorrido. **Seis de los siete empezaron en
`LTE_WAIT_HTTPACTION`** y el séptimo en `LTE_WAIT_HTTPTERM_POST`: el POST ya había salido,
la respuesta no llegó, el ciclo expiró a los 30 s, y a partir de ahí el SIM7600 dejó de
contestar por completo. El 83 % de las líneas de fallo (104 de 125) son el módem ya
atorado, no la causa.

Duración de los episodios: 81 s de mediana, 403 s el peor. Uno se recuperó solo tras siete
minutos; otro terminó únicamente cuando el nodo perdió energía.

**Corrección.** Reanimación por PWRKEY tras 10 esperas agotadas consecutivas en
`LTE_WAIT_AT`, con enfriamiento de 3 minutos, integrada en la máquina de estados para no
bloquear el lazo.

**Por qué no sirven `AT+CFUN=1,1` ni `AT+CRESET`:** viajan por el mismo puerto serie que
dejó de responder.

**Por qué la secuencia pregunta entre pulso y pulso:** PWRKEY conmuta. Dos pulsos
encadenados sólo funcionan si el módem estaba encendido pero sordo; si estuviera realmente
apagado, el primero lo encendería y el segundo lo volvería a apagar.

**Estado de la verificación.** El disparador y la secuencia se verificaron en banco con la
variante `R11_TEST_MODEM`. Que el pulso sea lo que cura al módem **no está demostrado**:
la tarjeta de adaptación lo vuelve a encender sola 28–30 s después de un apagado, y ese
tiempo enmascara el efecto del pulso. Queda como trabajo pendiente.

### Relleno en el campo de rumbo del estado

**Síntoma.** Tres entradas de 167 aparecían en la interfaz sin fuente y sin estado de IMU,
aunque llegaban completas al canal.

**Causa.** `String(txHeading_deg, 0)` formatea con `dtostrf` y un ancho mínimo de dos
caracteres, de modo que un rumbo de un solo dígito viajaba como `_HDG_ 4_`, con un espacio
delante. El patrón de la plataforma no coincidía y descartaba el bloque entero en vez de
sólo ese campo. Afectaba únicamente a rumbos de 0 a 9 grados.

**Corrección en dos capas.** El nodo redondea a entero y ya no emite el relleno; la
plataforma admite espacios alrededor del número, porque las entradas afectadas siguen en el
historial.

Es la misma familia que un defecto anterior de la plataforma, donde el patrón rechazaba el
`_HDG_-1` que el nodo envía mientras no aprende el norte: **un campo que no encaja descarta
el bloque entero en vez de sólo ese campo**.

### La bandera de calibración medía otra cosa

**Síntoma.** Los 22 arranques registrados decían `calibracionCargada=no`, lo que sugería
que la persistencia estaba rota.

**Causa.** La bandera se asignaba como `headingValid` de cualquiera de los dos sensores
inerciales, es decir reportaba si se había recuperado el **rumbo**, no si se había cargado
la calibración. El piso sí se restauraba en cada arranque. A eso se sumaba que el nodo no
había aprendido el norte hasta el tramo final del recorrido del 20 de septiembre, así que
en efecto no había rumbo que recuperar.

El archivo se verificó byte por byte y estaba íntegro: magic correcto, versión 2, tamaño
84, banderas coherentes y suma FNV-1a idéntica a la almacenada.

**Corrección.** La bandera se separó en `calibracionPiso` y `calibracionRumbo`.

### Pendiente: el guardado degrada la calibración

Si un sensor falta al arrancar, el rumbo persistido no se puede reconstruir y la rutina de
guardado reescribe `IMUCAL.BIN` sin él, perdiendo una referencia que costó una sesión de
campo obtener. Debería conservar lo persistido cuando no puede mejorarlo.

Se observó el 21 de septiembre: un arranque con el bus I2C degradado borró el rumbo de
74.60° con calidad 0.9989 que el nodo había aprendido el día anterior.
