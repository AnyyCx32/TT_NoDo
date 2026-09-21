# Detecta en que pines hay resistencias de pull-up externas.
#
# Los modulos MPU6500 llevan pull-ups (2.2k a 10k) de SDA y SCL a VCC.
# El pull-down interno del RP2040 es de unos 50k a 80k, mucho mas debil.
# Entonces:
#   con pull-down interno y aun asi se lee 1  -> hay un pull-up externo,
#                                                es decir modulo conectado Y alimentado
#   con pull-down 0 y con pull-up 1           -> pin al aire, no hay nada
#   0 en ambos casos                          -> el pin esta forzado a GND
import time
from machine import Pin

OMITIR = (23, 24, 25, 29)   # de uso interno en la Pico

conPullup = []
alAire = []
aTierra = []

for gp in range(0, 29):
    if gp in OMITIR:
        continue
    try:
        p = Pin(gp, Pin.IN, Pin.PULL_DOWN)
        time.sleep_ms(3)
        conDown = p.value()

        p = Pin(gp, Pin.IN, Pin.PULL_UP)
        time.sleep_ms(3)
        conUp = p.value()

        Pin(gp, Pin.IN, None)   # se deja el pin en alta impedancia
    except Exception as e:
        print("GP%-2d error: %s" % (gp, e))
        continue

    if conDown == 1 and conUp == 1:
        conPullup.append(gp)
    elif conDown == 0 and conUp == 0:
        aTierra.append(gp)
    else:
        alAire.append(gp)

print("")
print("Pines con PULL-UP EXTERNO (algo conectado y alimentado):")
print("   ", conPullup if conPullup else "ninguno")
print("")
print("Pines forzados a GND:")
print("   ", aTierra if aTierra else "ninguno")
print("")
print("Pines al aire (sin nada conectado):")
print("   ", alAire if alAire else "ninguno")
print("")

if not conPullup:
    print("DIAGNOSTICO: no se detecta ningun pull-up externo en toda la placa.")
    print("El modulo no esta conectado a los pines de datos, o no tiene 3V3.")
    print("Si el modulo estuviera alimentado y cableado, SDA y SCL apareceran")
    print("necesariamente en la lista de pull-up, aunque el chip estuviera muerto.")
else:
    print("DIAGNOSTICO: esos pines tienen el bus alimentado y en reposo alto.")
    print("Ahi deberian estar SDA y SCL. Si el barrido I2C no encontro nada")
    print("en esa pareja, el bus esta bien pero el chip no contesta.")
