# Mapa de continuidad: descubre que pin esta conectado con que pin.
#
# Convierte la Pico en un probador de cables. Recorre cada GPIO poniendolo en
# alto uno a la vez, y lee todos los demas con pull-down interno. Si otro pin
# sube a 1, hay continuidad electrica entre ambos.
#
# Para probar un cable dupont suelto: conectar sus dos extremos a dos GPIO
# cualesquiera (por ejemplo GP2 y GP3) y ejecutar. Si el cable sirve, aparece.
#
# Con el modulo MPU6500 conectado pero SIN alimentacion, es normal ver SDA y
# SCL enlazados entre si a traves de sus resistencias de pull-up. Esa senal
# tambien es util: confirma que los cables de datos si llegan al modulo.
import time
from machine import Pin

OMITIR = (23, 24, 25, 29)
PINES = [gp for gp in range(0, 29) if gp not in OMITIR]

enlaces = []

for emisor in PINES:
    # Todos en entrada con pull-down, menos el que emite.
    receptores = {}
    for gp in PINES:
        if gp != emisor:
            receptores[gp] = Pin(gp, Pin.IN, Pin.PULL_DOWN)

    salida = Pin(emisor, Pin.OUT)
    salida.value(1)
    time.sleep_ms(5)

    for gp, p in receptores.items():
        if p.value() == 1 and (gp, emisor) not in enlaces:
            enlaces.append((emisor, gp))

    salida.value(0)
    Pin(emisor, Pin.IN, None)

for gp in PINES:
    Pin(gp, Pin.IN, None)

print("")
if not enlaces:
    print("No se detecto continuidad entre ningun par de pines.")
    print("Si acabas de conectar un cable entre dos GPIO y no aparece aqui,")
    print("ese cable esta roto por dentro aunque se vea bien.")
else:
    print("Continuidad detectada entre:")
    for a, b in enlaces:
        print("   GP%-2d  <-->  GP%-2d" % (a, b))
print("")
