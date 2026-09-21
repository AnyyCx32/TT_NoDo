# Analisis del archivo VIB_YYYYMMDD.CSV generado por R11.
#
# Cubre los calculos que piden las fichas P-01, P-02, P-05, P-06, P-09, P-10 y
# P-20 del Capitulo 4, y ademas la vigilancia de alimentacion incorporada en la
# campana de agosto de 2026. No modifica el archivo original.
#
# Lee por flujo, sin cargar el archivo en memoria, de modo que soporta las
# sesiones largas: nueve horas a 20 Hz producen del orden de 170 MB.
#
# Reconoce dos formatos: el de 41 columnas y el de 43 con las columnas de
# alimentacion vsysMin_V y vsysMax_V.
#
# Uso:  python analizar_vib.py VIB_20260828.CSV
#       python analizar_vib.py VIB_20260828.CSV --tramo 2

import math
import os
import statistics as st
import sys
from collections import Counter


def titulo(texto):
    print()
    print("=" * 70)
    print(texto)
    print("=" * 70)


def abrir(ruta):
    return open(ruta, "r", encoding="utf-8", errors="replace")


def leer_encabezado(ruta):
    """Devuelve (comentarios, columnas) sin consumir el archivo completo."""
    comentarios = []
    with abrir(ruta) as f:
        for linea in f:
            linea = linea.rstrip("\n").rstrip("\r")
            if not linea:
                continue
            if linea.startswith("#"):
                comentarios.append(linea)
                continue
            return comentarios, linea.split(",")
    return comentarios, []


def detectar_tramos(ruta, idx_millis):
    """Localiza los reinicios: la columna millis vuelve a empezar en cada uno.

    Devuelve una lista de (linea_inicial, linea_final, muestras) en numero de
    filas de datos, para poder analizar una sesion concreta.
    """
    tramos = []
    inicio = 0
    anterior = -1
    n = 0
    with abrir(ruta) as f:
        for linea in f:
            if linea.startswith("#") or linea.startswith("date,"):
                continue
            partes = linea.split(",", 3)
            if len(partes) < 3:
                continue
            try:
                ms = int(partes[2])
            except ValueError:
                continue
            if anterior >= 0 and ms < anterior:
                tramos.append((inicio, n - 1, n - inicio))
                inicio = n
            anterior = ms
            n += 1
    if n > inicio:
        tramos.append((inicio, n - 1, n - inicio))
    return tramos


def recorrer(ruta, columnas, tramo=None):
    """Genera las filas de datos ya troceadas, opcionalmente de un solo tramo."""
    esperado = len(columnas)
    indice = 0
    with abrir(ruta) as f:
        for linea in f:
            if linea.startswith("#") or linea.startswith("date,"):
                continue
            linea = linea.rstrip("\n").rstrip("\r")
            if not linea:
                continue
            partes = linea.split(",")
            if len(partes) != esperado:
                yield indice, None      # fila defectuosa
                indice += 1
                continue
            if tramo is None or (tramo[0] <= indice <= tramo[1]):
                yield indice, partes
            indice += 1


def col(columnas, nombre):
    return columnas.index(nombre) if nombre in columnas else None


def analizar(ruta, tramo=None):
    comentarios, columnas = leer_encabezado(ruta)
    if not columnas:
        print("El archivo no contiene datos.")
        return

    tam = os.path.getsize(ruta)
    print("Archivo analizado:", ruta)
    print("Tamano: %.1f MB" % (tam / 1048576.0))
    for c in comentarios:
        print("Metadato:", c)

    i_ms = col(columnas, "millis")
    if i_ms is None:
        print("El archivo no tiene columna millis; formato no reconocido.")
        return

    tramos = detectar_tramos(ruta, i_ms)
    print("Sesiones detectadas (reinicios del nodo):", len(tramos))
    for k, (a, b, n) in enumerate(tramos, start=1):
        print("  tramo %d: filas %d a %d  (%d muestras)" % (k, a, b, n))
    if tramo is not None:
        if tramo < 1 or tramo > len(tramos):
            print("Tramo fuera de rango.")
            return
        seleccion = tramos[tramo - 1]
        print("Analizando solo el tramo", tramo)
    else:
        seleccion = None
        if len(tramos) > 1:
            print("AVISO: se analizan todos los tramos juntos. Use --tramo N")
            print("       para estudiar una sesion concreta.")

    # ---------------- Acumuladores de una sola pasada ----------------
    i_mag = col(columnas, "magLin_g")
    i_lat, i_lon = col(columnas, "lat"), col(columnas, "lon")
    i_sats, i_hacc = col(columnas, "sats"), col(columnas, "hAcc_m")
    i_usa = col(columnas, "gpsUsable")
    i_still, i_zupt = col(columnas, "still"), col(columnas, "zupt")
    i_icm, i_mpu = col(columnas, "icmMag_g"), col(columnas, "mpuMag_g")
    i_ve, i_vn = col(columnas, "velE_mps"), col(columnas, "velN_mps")
    i_vsmin, i_vsmax = col(columnas, "vsysMin_V"), col(columnas, "vsysMax_V")

    n = 0
    defectuosas = 0
    ms_primero = ms_ultimo = None
    ms_anterior = None
    intervalos = Counter()
    mayores = []

    mag_n = mag_sum = mag_sum2 = 0.0
    mag_max = 0.0
    still_n = zupt_n = 0
    icm_min, icm_max, icm_sum, icm_n = 9e9, -9e9, 0.0, 0
    mpu_min, mpu_max, mpu_sum, mpu_n = 9e9, -9e9, 0.0, 0
    vel_max = 0.0
    vsys_min, vsys_max, vsys_sum, vsys_n = 9e9, -9e9, 0.0, 0

    soluciones = []          # solo cambios de solucion GNSS: 1 de cada 4 filas
    ultima_clave = None
    sats_l, hacc_l = [], []
    validas = 0

    for indice, fila in recorrer(ruta, columnas, seleccion):
        if fila is None:
            defectuosas += 1
            continue
        n += 1

        try:
            ms = int(fila[i_ms])
        except ValueError:
            continue
        if ms_primero is None:
            ms_primero = ms
        ms_ultimo = ms
        if ms_anterior is not None:
            d = ms - ms_anterior
            if 0 < d < 3600000:
                intervalos[d] += 1
                if len(mayores) < 40:
                    mayores.append(d)
                    mayores.sort(reverse=True)
                elif d > mayores[-1]:
                    mayores[-1] = d
                    mayores.sort(reverse=True)
        ms_anterior = ms

        if i_mag is not None:
            try:
                v = float(fila[i_mag])
                mag_n += 1
                mag_sum += v
                mag_sum2 += v * v
                if v > mag_max:
                    mag_max = v
            except ValueError:
                pass

        if i_still is not None and fila[i_still] == "1":
            still_n += 1
        if i_zupt is not None and fila[i_zupt] == "1":
            zupt_n += 1

        for idx, acc in ((i_icm, "icm"), (i_mpu, "mpu")):
            if idx is None:
                continue
            try:
                v = float(fila[idx])
            except ValueError:
                continue
            if acc == "icm":
                icm_n += 1
                icm_sum += v
                icm_min = min(icm_min, v)
                icm_max = max(icm_max, v)
            else:
                mpu_n += 1
                mpu_sum += v
                mpu_min = min(mpu_min, v)
                mpu_max = max(mpu_max, v)

        if i_ve is not None and i_vn is not None:
            try:
                m = math.hypot(float(fila[i_ve]), float(fila[i_vn]))
                if m > vel_max:
                    vel_max = m
            except ValueError:
                pass

        if i_vsmin is not None:
            try:
                a = float(fila[i_vsmin])
                b = float(fila[i_vsmax])
                if 2.0 < a < 15.0:
                    vsys_min = min(vsys_min, a)
                    vsys_max = max(vsys_max, b)
                    vsys_sum += a
                    vsys_n += 1
            except ValueError:
                pass

        if i_lat is not None:
            try:
                lat = float(fila[i_lat])
                lon = float(fila[i_lon])
            except ValueError:
                continue
            if lat == 0.0 and lon == 0.0:
                continue
            clave = (lat, lon)
            if clave != ultima_clave:
                ultima_clave = clave
                soluciones.append((ms, lat, lon))
                if i_usa is not None and fila[i_usa] == "1":
                    validas += 1
                if i_sats is not None and fila[i_sats].isdigit():
                    sats_l.append(int(fila[i_sats]))
                if i_hacc is not None:
                    try:
                        hacc_l.append(float(fila[i_hacc]))
                    except ValueError:
                        pass

    # ---------------- P-09 ----------------
    titulo("P-09  Creacion y estructura del archivo principal")
    print("  columnas declaradas :", len(columnas))
    print("  formato             :",
          "43 columnas, con vigilancia de alimentacion" if i_vsmin is not None
          else "41 columnas, sin vigilancia de alimentacion")
    print("  filas completas     :", n)
    print("  filas defectuosas   :", defectuosas)
    print("  criterio: estructura legible y consistente ->",
          "CUMPLE" if defectuosas == 0 and n > 0 else "REVISAR")

    if n < 2 or ms_primero is None:
        print("\nMuestras insuficientes para el resto del analisis.")
        return

    # ---------------- P-10 y P-20 ----------------
    dur = (ms_ultimo - ms_primero) / 1000.0
    total_int = sum(intervalos.values())
    media_int = sum(k * v for k, v in intervalos.items()) / total_int if total_int else 0

    ordenados = sorted(intervalos.items())
    acumulado = 0
    mediana_int = 0
    for valor, veces in ordenados:
        acumulado += veces
        if acumulado >= total_int / 2:
            mediana_int = valor
            break

    titulo("P-10  Frecuencia efectiva de almacenamiento y continuidad")
    print("  muestras            :", n)
    print("  duracion            : %.1f s  (%.2f h)" % (dur, dur / 3600.0))
    print("  frecuencia efectiva : %.3f Hz   (nominal 20 Hz)" % (n / dur if dur else 0))
    print("  intervalo medio     : %.1f ms  (nominal 50 ms)" % media_int)
    print("  intervalo mediano   : %d ms" % mediana_int)
    print("  minimo / maximo     : %d / %d ms" % (ordenados[0][0], ordenados[-1][0]))
    for umbral in (100, 200, 500, 1000, 5000):
        c = sum(v for k, v in intervalos.items() if k > umbral)
        print("  huecos > %5d ms    : %d" % (umbral, c))
    frec = n / dur if dur else 0
    print("  criterio P-10 ->", "CUMPLE" if abs(frec - 20.0) < 2.0 else "REVISAR")

    titulo("P-20  Comportamiento no bloqueante")
    print("  veinte intervalos mayores:", mayores[:20], "ms")
    peor = ordenados[-1][0]
    print("  interrupcion sostenida (>1 s):",
          "ninguna" if peor <= 1000 else "%d ms" % peor)
    print("  criterio P-20 ->", "CUMPLE" if peor <= 1000 else "REVISAR")

    # ---------------- Alimentacion ----------------
    if vsys_n:
        titulo("Vigilancia de alimentacion (VSYS)")
        print("  muestras con lectura valida :", vsys_n)
        print("  minimo absoluto             : %.3f V" % vsys_min)
        print("  maximo absoluto             : %.3f V" % vsys_max)
        print("  media de los minimos        : %.3f V" % (vsys_sum / vsys_n))
        print("  umbral de alerta del firmware: 4.40 V")
        if vsys_min < 4.40:
            print("  HALLAZGO: hubo caidas por debajo del umbral. Buscar las")
            print("            lineas ALERTA en EVENTOS.LOG y comparar su hora.")
        else:
            print("  Sin caidas por debajo del umbral en todo el registro.")

    # ---------------- P-05 y P-06 ----------------
    if mag_n:
        titulo("P-05  Adquisicion de vibraciones")
        media = mag_sum / mag_n
        var = max(0.0, mag_sum2 / mag_n - media * media)
        print("  muestras          :", mag_n)
        print("  magnitud media    : %.5f g" % media)
        print("  desviacion tipica : %.5f g" % math.sqrt(var))
        print("  maximo            : %.5f g" % mag_max)
        print("  umbral de reposo del programa: 0.035 g")

    titulo("P-06  Deteccion de reposo y ZUPT")
    print("  con reposo activo : %d (%.1f %%)" % (still_n, 100.0 * still_n / n))
    print("  con ZUPT aplicado : %d (%.1f %%)" % (zupt_n, 100.0 * zupt_n / n))
    if icm_n:
        print("  icmMag_g  media %.4f  min %.4f  max %.4f"
              % (icm_sum / icm_n, icm_min, icm_max))
    if mpu_n:
        print("  mpuMag_g  media %.4f  min %.4f  max %.4f"
              % (mpu_sum / mpu_n, mpu_min, mpu_max))
    print("  velocidad estimada maxima: %.3f m/s" % vel_max)
    print("  criterio P-06 (parte estatica) ->",
          "CUMPLE" if zupt_n > 0 else "NO CUMPLE")

    # ---------------- P-01 y P-02 ----------------
    if len(soluciones) < 10:
        print("\nSoluciones GNSS insuficientes para P-01 y P-02.")
        return

    titulo("P-01  Estabilidad GNSS en condicion estacionaria")
    print("  soluciones GNSS distintas :", len(soluciones))
    print("  validas segun criterio R11: %d (%.1f %%)"
          % (validas, 100.0 * validas / len(soluciones)))

    lat_ref = st.median([s[1] for s in soluciones])
    lon_ref = st.median([s[2] for s in soluciones])
    print("  referencia (mediana): %.7f, %.7f" % (lat_ref, lon_ref))

    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat_ref))
    desv = sorted(math.hypot((s[2] - lon_ref) * mlon, (s[1] - lat_ref) * mlat)
                  for s in soluciones)

    def pct(p):
        return desv[min(len(desv) - 1, int(len(desv) * p))]

    print("  dispersion horizontal:")
    print("    media   : %.2f m" % (sum(desv) / len(desv)))
    print("    mediana : %.2f m" % desv[len(desv) // 2])
    print("    p95     : %.2f m" % pct(0.95))
    print("    maximo  : %.2f m" % desv[-1])
    if hacc_l:
        print("  hAcc del receptor: media %.2f m, min %.2f, max %.2f"
              % (sum(hacc_l) / len(hacc_l), min(hacc_l), max(hacc_l)))
    if sats_l:
        print("  satelites: media %.1f, min %d, max %d"
              % (sum(sats_l) / len(sats_l), min(sats_l), max(sats_l)))

    titulo("P-02  Frecuencia efectiva de actualizacion GNSS")
    dts = [soluciones[i + 1][0] - soluciones[i][0] for i in range(len(soluciones) - 1)]
    dts = [d for d in dts if 0 < d < 60000]
    if dts:
        dts_ord = sorted(dts)
        print("  intervalos analizados :", len(dts))
        print("  intervalo medio       : %.1f ms" % (sum(dts) / len(dts)))
        print("  intervalo mediano     : %d ms" % dts_ord[len(dts_ord) // 2])
        print("  minimo / maximo       : %d / %d ms" % (dts_ord[0], dts_ord[-1]))
        print("  frecuencia media      : %.2f Hz   (nominal 5 Hz)"
              % (1000.0 * len(dts) / sum(dts)))
        print("  intervalos > 400 ms   :", sum(1 for d in dts if d > 400))
    print("  NOTA: el CSV se muestrea a 20 Hz, luego la resolucion de este")
    print("        calculo es de 50 ms.")


def main():
    if len(sys.argv) < 2:
        print("Uso: python analizar_vib.py VIB_YYYYMMDD.CSV [--tramo N]")
        sys.exit(1)
    ruta = sys.argv[1]
    tramo = None
    if "--tramo" in sys.argv:
        tramo = int(sys.argv[sys.argv.index("--tramo") + 1])
    analizar(ruta, tramo)
    print()


if __name__ == "__main__":
    main()
