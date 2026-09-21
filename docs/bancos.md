# Bancos de prueba y variantes

Todos **reemplazan** al firmware de operación cuando se cargan. Para devolver el nodo a
servicio hay que volver a cargar `firmware/operacion/`.

## R11_DIAG — diagnóstico completo

Replica pines, direcciones I2C, escalas y comandos AT del firmware de operación, pero no
navega: sirve para observar la cadena sensores → microSD → LTE → ThingSpeak desde la
consola USB.

Menú de una letra a 115200 baudios. `herramientas/diag.ps1 -Cmd <letra> -Seconds <n>` abre
el puerto, manda el comando y transcribe la respuesta.

| Comando | Qué hace |
|---|---|
| `1`–`8` | prueba de componente individual |
| `a` | todo el nivel 1 |
| `s` `w` `d` `e` | por subsistema |
| `n` | cadena completa |
| `c` | catálogo de archivos de vibración |
| `t <archivo>` | volcado de un archivo |
| `r` | informe |

## SD_TEST — banco dedicado a la microSD

Mismos pines que el firmware de operación. Nueve niveles, de la comprobación de pines a la
prueba de intermitencia. `herramientas/sd.ps1` lo maneja igual que `diag.ps1`.

| Nivel | Qué prueba |
|---|---|
| 1 | pines |
| 2 | sonda SPI cruda |
| 3 | montaje a varias velocidades |
| 4 | sistema de archivos |
| 5 | escritura y lectura |
| 6 | velocidad |
| 7 | raíz |
| 8 | sector 0 |
| 9 | consistencia de lectura |
| `r` | intermitencia |

**Dos trampas del banco, ya corregidas, que costaron corridas enteras:**

1. **Si MISO queda flotando** —tarjeta mal asentada o DO suelto— toda respuesta vale 0x00 y
   el programa creía que la tarjeta contestaba: CMD0 «responde», ACMD41 «queda lista», OCR
   en ceros. La prueba de bus en reposo del nivel 2 —ocho relojes con CS en alto, debe
   volver 0xFF— es la que lo distingue. Un OCR todo en ceros es imposible y ahora detiene
   el diagnóstico.
2. **Los niveles crudos desmontan el volumen** para tomar el bus. Si no se limpia la
   bandera de montaje, los niveles 5 a 7 abren archivos contra un sistema de archivos que
   ya no existe y fingen que la tarjeta no guarda nada.

**Hallazgo de referencia.** Con este banco se determinó que una microSD puede responder
correctamente todo el protocolo —CMD0, CMD8, ACMD41 y CMD58— y aun así estar muerta: su
CID venía en ceros y los sectores devolvían byte por byte el mismo patrón. Ni formatear ni
limpiar la partición la cambiaban; descartaba las escrituras. No se arregla, se reemplaza.

## R11_PROFILE — perfilado del lazo

Mide la duración de cada etapa por vuelta. Es la herramienta que identificó el defecto M4,
al mostrar que una sola etapa consumía 6151 ms de un lazo que debía correr a 20 Hz.

Es el primer banco que conviene cargar cuando algo «no responde» y no se sabe qué.

## R11_LEDTEST y MPU6500_TEST

Bancos auxiliares para los indicadores locales y para el acelerómetro. En
`herramientas/probe_*.py` hay sondas de barrido del bus I2C, de continuidad y de pull-ups.

## R11_DEMO_TS — demostración de cadencia

Variante del firmware de operación con el periodo forzado a 2 s y el piso de separación
desactivado. Sirve para demostrar que el límite de cadencia lo impone el servicio.

**Procedimiento.** Operar 10 minutos a cielo abierto, estacionario, y comparar dos cifras
del mismo intervalo:

- envíos correctos en `EVENTOS.LOG`: deberían rondar los 300
- entradas nuevas en el canal: deberían rondar las 40, que es 600 s entre 15

**La diferencia entre ambas cifras es la demostración.** Cada rechazo queda además marcado
en la bitácora con `AVISO ThingSpeak descarto la actualizacion`, porque el servicio
responde 200 con un cuerpo de un solo byte en vez de señalar un error HTTP.

Al terminar, recargar el firmware de operación.

## R11_TEST_MODEM — prueba de la reanimación

Variante que fuerza la condición de módem mudo para ejercitar la recuperación por PWRKEY.
Apaga el módem con `AT+CPOF` pasados 90 s y baja el umbral de disparo a 3 esperas agotadas.

**Limitación conocida.** La tarjeta de adaptación vuelve a encender el módem sola 28–30 s
después del apagado, y ese tiempo enmascara el efecto del pulso: la secuencia se verifica,
la cura no. Para aislarla habría que bajar el umbral a 1, de modo que el pulso caiga unos
6 s después del apagado, muy por delante del auto-encendido.
