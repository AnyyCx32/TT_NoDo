# Ejecuta un bloque de codigo en el REPL de MicroPython por puerto serie.
# Usa el modo pegar (Ctrl-E / Ctrl-D) para que la indentacion llegue intacta.
# Uso:  powershell -File mpy.ps1 -CodeFile probe.py -Seconds 15
param(
    [string]$Port = "COM7",
    [string]$CodeFile = "",
    [string]$Code = "",
    [int]$Seconds = 15
)

if ($CodeFile -ne "") {
    $Code = Get-Content -Path $CodeFile -Raw
}
if ($Code -eq "") {
    Write-Output "Falta el codigo a ejecutar."
    exit 1
}

$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$sp.ReadTimeout = 500
$sp.DtrEnable = $true
$sp.RtsEnable = $true

try {
    $sp.Open()
} catch {
    Write-Output "No se pudo abrir $Port : $($_.Exception.Message)"
    exit 1
}

Start-Sleep -Milliseconds 300

# Ctrl-C dos veces: interrumpe cualquier script en marcha y deja el prompt listo.
$sp.Write([char]3 + "")
Start-Sleep -Milliseconds 150
$sp.Write([char]3 + "")
Start-Sleep -Milliseconds 300
$sp.DiscardInBuffer()

# Ctrl-E entra en modo pegar; Ctrl-D lo ejecuta.
$sp.Write([char]5 + "")
Start-Sleep -Milliseconds 200
$sp.DiscardInBuffer()

$normalized = $Code -replace "`r`n", "`n"
$sp.Write($normalized)
Start-Sleep -Milliseconds 300
$sp.Write([char]4 + "")

$deadline = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $deadline) {
    try {
        $chunk = $sp.ReadExisting()
        if ($chunk.Length -gt 0) {
            Write-Host -NoNewline $chunk
        } else {
            Start-Sleep -Milliseconds 60
        }
    } catch {
        Start-Sleep -Milliseconds 60
    }
}

$sp.Close()
