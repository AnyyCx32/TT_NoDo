# Barrido de todas las parejas SDA/SCL validas del RP2040 buscando el MPU6500.
import sys
from machine import Pin, I2C

print("MicroPython:", sys.implementation)
print("")

PARES = [(0, 0, 1), (0, 4, 5), (0, 8, 9), (0, 12, 13), (0, 16, 17), (0, 20, 21),
         (1, 2, 3), (1, 6, 7), (1, 10, 11), (1, 14, 15), (1, 18, 19), (1, 26, 27)]

NOMBRES = {0x70: "MPU6500", 0x71: "MPU9250", 0x73: "MPU9255",
           0x68: "MPU6050", 0x75: "MPU6515", 0xEA: "ICM-20948"}

encontrados = []

for idx, sda, scl in PARES:
    try:
        bus = I2C(idx, sda=Pin(sda), scl=Pin(scl), freq=100000)
        dispositivos = bus.scan()
    except Exception as e:
        print("I2C%d SDA=GP%-2d SCL=GP%-2d  error: %s" % (idx, sda, scl, e))
        continue

    if not dispositivos:
        continue

    for addr in dispositivos:
        try:
            who = bus.readfrom_mem(addr, 0x75, 1)[0]
            nombre = NOMBRES.get(who, "desconocido")
        except Exception:
            who = None
            nombre = "no se pudo leer WHO_AM_I"
        print("I2C%d SDA=GP%-2d SCL=GP%-2d  ->  0x%02X  WHO_AM_I=%s  %s"
              % (idx, sda, scl, addr,
                 ("0x%02X" % who) if who is not None else "n/d", nombre))
        encontrados.append((idx, sda, scl, addr, who))

print("")
if not encontrados:
    print("RESULTADO: ningun dispositivo respondio en ninguna pareja de pines.")
else:
    print("RESULTADO: %d respuesta(s). Detalle arriba." % len(encontrados))
