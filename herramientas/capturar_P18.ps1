# Captura con marca de tiempo por linea, para medir la periodicidad de
# transmision que pide la ficha P-18.
#
# La captura normal escribe el flujo crudo del puerto, sin tiempos, de modo que
# no permite calcular el intervalo entre envios. Aqui cada linea recibida se
# sella con la hora local en el momento de completarse.
#
# Uso:  powershell -File capturar_P18.ps1 -Minutos 12
param(
    [string]$Port = "COM3",
    [int]$Minutos = 12,
    [string]$Salida = "c:\Users\marcx\Downloads\TT Programas\Evidencias\P18_transmisiones.log"
)

$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$sp.ReadTimeout = 500
$sp.DtrEnable = $true

for ($i = 0; $i -lt 40; $i++) {
    try { $sp.Open(); break } catch { Start-Sleep -Milliseconds 250 }
}
if (-not $sp.IsOpen) { Write-Output "No se pudo abrir $Port"; exit 1 }

"# Captura con sello de tiempo, inicio $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff')" |
    Out-File $Salida -Encoding utf8 -Force
$sw = [System.IO.StreamWriter]::new($Salida, $true)

$parcial = ""
$fin = (Get-Date).AddMinutes($Minutos)
while ((Get-Date) -lt $fin) {
    try {
        $trozo = $sp.ReadExisting()
        if ($trozo.Length -gt 0) {
            $parcial += $trozo
            # Se emite una linea completa en cuanto llega su salto de linea.
            while ($parcial.Contains("`n")) {
                $corte = $parcial.IndexOf("`n")
                $linea = $parcial.Substring(0, $corte).TrimEnd("`r")
                $parcial = $parcial.Substring($corte + 1)
                $sw.WriteLine("$(Get-Date -Format 'HH:mm:ss.fff')`t$linea")
                $sw.Flush()
            }
        } else {
            Start-Sleep -Milliseconds 40
        }
    } catch { Start-Sleep -Milliseconds 40 }
}

$sw.Close(); $sp.Close()

# Resumen inmediato de la periodicidad observada.
$exitos = Get-Content $Salida | Select-String -Pattern "HTTP exitoso"
Write-Output "Envios exitosos: $($exitos.Count)"
if ($exitos.Count -ge 2) {
    $tiempos = $exitos | ForEach-Object { [datetime]::ParseExact($_.Line.Split("`t")[0], 'HH:mm:ss.fff', $null) }
    $deltas = for ($i = 1; $i -lt $tiempos.Count; $i++) { ($tiempos[$i] - $tiempos[$i-1]).TotalSeconds }
    $ordenados = $deltas | Sort-Object
    Write-Output ("Intervalo medio   : {0:N2} s" -f ($deltas | Measure-Object -Average).Average)
    Write-Output ("Intervalo mediano : {0:N2} s" -f $ordenados[[int]($ordenados.Count/2)])
    Write-Output ("Minimo / maximo   : {0:N2} / {1:N2} s" -f ($deltas | Measure-Object -Minimum).Minimum, ($deltas | Measure-Object -Maximum).Maximum)
    Write-Output "Periodo nominal del programa: 15 s mas la duracion del ciclo HTTP"
}
