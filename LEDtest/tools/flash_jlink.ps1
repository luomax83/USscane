$ErrorActionPreference = "Stop"

$workspace = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $workspace "build"
$elf = Join-Path $buildDir "LEDtest.elf"
$jrun = "C:/Program Files/SEGGER/JLink_V816/JRun.exe"
$device = "STM32F103ZE"
$probeSerial = "261003573"
$speed = 100
$logFile = Join-Path $env:TEMP "jrun_flash.log"

if (-not (Test-Path $elf)) {
    throw "Firmware not found: $elf"
}

if (-not (Test-Path $jrun)) {
    throw "J-Run not found: $jrun"
}

Remove-Item $logFile -Force -ErrorAction SilentlyContinue

Get-Process -Name "JRun", "JLink", "JLinkGUIServer", "JLinkGDBServerCL" -ErrorAction SilentlyContinue |
    Stop-Process -Force -ErrorAction SilentlyContinue

$arguments = @(
    "--usb", $probeSerial,
    "--device", $device,
    "--if", "SWD",
    "--speed", "$speed",
    "--nortt",
    "--nosemihost",
    "--log", $logFile,
    $elf
)

$process = Start-Process `
    -FilePath $jrun `
    -ArgumentList $arguments `
    -PassThru `
    -NoNewWindow

$started = $false
$errorDetected = $false

try {
    for ($attempt = 0; $attempt -lt 240; $attempt++) {
        Start-Sleep -Milliseconds 500

        if (Test-Path $logFile) {
            $log = Get-Content $logFile -Raw
            if ($log -match "Reading output from target until exit command\." -or $log -match "Start target application\.\.\.") {
                $started = $true
                break
            }

            if ($log -match "(?m)^\*JRUN .*ERROR" -or $log -match "Connecting to J-Link failed") {
                $errorDetected = $true
                break
            }
        }

        if ($process.HasExited) {
            break
        }
    }

    if (-not $started) {
        if (Test-Path $logFile) {
            Get-Content $logFile | Write-Output
        }

        if ($errorDetected) {
            throw "J-Run reported a flashing error."
        }

        if ($process.HasExited -and ($process.ExitCode -ne 0)) {
            throw "J-Run exited with code $($process.ExitCode)."
        }

        throw "Timed out while waiting for firmware download/start confirmation."
    }
}
finally {
    if (-not $process.HasExited) {
        Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
    }

    Get-Process -Name "JLinkGUIServer" -ErrorAction SilentlyContinue |
        Stop-Process -Force -ErrorAction SilentlyContinue

    if (Test-Path $logFile) {
        Get-Content $logFile | Write-Output
    }
}
