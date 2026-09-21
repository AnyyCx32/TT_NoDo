# Consola de apoyo para R11-DIAG.
# Uso:  powershell -File diag.ps1 -Cmd 1 -Seconds 20
# Abre COM3 a 115200, envia el comando y transcribe todo lo que llega.
param(
    [string]$Port = "COM3",
    [string]$Cmd = "",
    [int]$Seconds = 20,
    [string]$LogFile = ""
)

$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$sp.ReadTimeout = 500
$sp.DtrEnable = $true
$sp.RtsEnable = $true
$sp.NewLine = "`n"

try {
    $sp.Open()
} catch {
    Write-Output "No se pudo abrir $Port : $($_.Exception.Message)"
    exit 1
}

Start-Sleep -Milliseconds 400
$sp.DiscardInBuffer()

if ($Cmd -ne "") {
    $sp.Write("$Cmd`r`n")
}

$deadline = (Get-Date).AddSeconds($Seconds)
$buffer = New-Object System.Text.StringBuilder
while ((Get-Date) -lt $deadline) {
    try {
        $chunk = $sp.ReadExisting()
        if ($chunk.Length -gt 0) {
            [void]$buffer.Append($chunk)
            Write-Host -NoNewline $chunk
        } else {
            Start-Sleep -Milliseconds 60
        }
    } catch {
        Start-Sleep -Milliseconds 60
    }
}

$sp.Close()

if ($LogFile -ne "") {
    $buffer.ToString() | Out-File -FilePath $LogFile -Encoding utf8
}
