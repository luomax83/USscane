$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

$appName = "US360Host"
$venvPython = Join-Path $root ".venv\Scripts\python.exe"
$venvPyInstaller = Join-Path $root ".venv\Scripts\pyinstaller.exe"
$distDir = Join-Path $root "dist"
$buildDir = Join-Path $root "build"
$specFile = Join-Path $root "$appName.spec"
$releaseDir = Join-Path $root "release"
$portableDir = Join-Path $releaseDir "${appName}_Portable"
$zipFile = Join-Path $releaseDir "${appName}_Portable.zip"
$installerBundleDir = Join-Path $releaseDir "${appName}_Installer"
$installerBundleZip = Join-Path $releaseDir "${appName}_Installer.zip"
$stageDir = Join-Path $root "_installer_stage"
$payloadDir = Join-Path $stageDir "payload"
$installBat = Join-Path $stageDir "install.bat"
$sedFile = Join-Path $stageDir "${appName}_Installer.sed"
$installerExe = Join-Path $releaseDir "${appName}_Installer.exe"

function Remove-IfExists([string]$path) {
    if (Test-Path $path) {
        Remove-Item -Recurse -Force $path
    }
}

function Ensure-CommandSuccess($process, [string]$message) {
    if ($process.ExitCode -ne 0) {
        throw $message
    }
}

Write-Host "============================================"
Write-Host "  Build $appName Installer"
Write-Host "============================================"
Write-Host ""

if (-not (Test-Path $venvPython)) {
    throw "Virtual env Python not found: $venvPython"
}

Write-Host "[1/6] Check PyInstaller..."
$null = & $venvPython -m pip show pyinstaller 2>$null
if ($LASTEXITCODE -ne 0) {
    & $venvPython -m pip install pyinstaller
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to install pyinstaller"
    }
}

Write-Host "[2/6] Clean old artifacts..."
Remove-IfExists $distDir
Remove-IfExists $buildDir
Remove-IfExists $specFile
Remove-IfExists $releaseDir
Remove-IfExists $stageDir
New-Item -ItemType Directory -Force -Path $releaseDir | Out-Null
New-Item -ItemType Directory -Force -Path $payloadDir | Out-Null

Write-Host "[3/6] Build standalone app..."
& $venvPyInstaller `
    --noconfirm `
    --clean `
    --windowed `
    --name $appName `
    --paths $root `
    --add-data "$root\sonar_host;sonar_host" `
    "$root\main.py"
if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller build failed"
}

Write-Host "[4/6] Create portable package..."
Copy-Item -Recurse -Force (Join-Path $distDir $appName) $portableDir
if (Test-Path $zipFile) {
    Remove-Item -Force $zipFile
}
Compress-Archive -Path (Join-Path $portableDir "*") -DestinationPath $zipFile -Force

Write-Host "[5/6] Prepare installer payload..."
Copy-Item -Recurse -Force (Join-Path $distDir $appName) (Join-Path $payloadDir $appName)

$installBatContent = @'
@echo off
setlocal EnableExtensions
chcp 65001 > nul
set "SOURCE=%~dp0payload\US360Host"
set "TARGET=%LocalAppData%\US360Host"
if not "%~1"=="" set "TARGET=%~1"
echo Install path: %TARGET%
if exist "%TARGET%" rmdir /s /q "%TARGET%"
mkdir "%TARGET%" > nul 2>&1
xcopy /e /i /y "%SOURCE%" "%TARGET%" > nul
powershell -NoProfile -ExecutionPolicy Bypass -Command "$s=New-Object -ComObject WScript.Shell; $lnk=$s.CreateShortcut([System.IO.Path]::Combine([Environment]::GetFolderPath('Desktop'), 'US360Host.lnk')); $lnk.TargetPath=[System.IO.Path]::Combine('%TARGET%','US360Host.exe'); $lnk.WorkingDirectory='%TARGET%'; $lnk.Save()"
start "" "%TARGET%\US360Host.exe"
exit /b 0
'@
Set-Content -Path $installBat -Value $installBatContent -Encoding ASCII

New-Item -ItemType Directory -Force -Path $installerBundleDir | Out-Null
Copy-Item -Force $installBat (Join-Path $installerBundleDir "install.bat")
Copy-Item -Recurse -Force $payloadDir (Join-Path $installerBundleDir "payload")
if (Test-Path $installerBundleZip) {
    Remove-Item -Force $installerBundleZip
}
Compress-Archive -Path (Join-Path $installerBundleDir "*") -DestinationPath $installerBundleZip -Force

$files = Get-ChildItem -Path $stageDir -File -Recurse | Sort-Object FullName
$dirToGroup = [ordered]@{}
$groupIndex = 0
foreach ($file in $files) {
    $dir = Split-Path -Parent $file.FullName
    if (-not $dirToGroup.Contains($dir)) {
        $dirToGroup[$dir] = $groupIndex
        $groupIndex += 1
    }
}

$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine("[Version]")
[void]$sb.AppendLine("Class=IEXPRESS")
[void]$sb.AppendLine("SEDVersion=3")
[void]$sb.AppendLine("[Options]")
[void]$sb.AppendLine("PackagePurpose=InstallApp")
[void]$sb.AppendLine("ShowInstallProgramWindow=1")
[void]$sb.AppendLine("HideExtractAnimation=0")
[void]$sb.AppendLine("UseLongFileName=1")
[void]$sb.AppendLine("InsideCompressed=0")
[void]$sb.AppendLine("CAB_FixedSize=0")
[void]$sb.AppendLine("CAB_ResvCodeSigning=0")
[void]$sb.AppendLine("RebootMode=N")
[void]$sb.AppendLine("InstallPrompt=")
[void]$sb.AppendLine("DisplayLicense=")
[void]$sb.AppendLine("FinishMessage=US360Host installation completed.")
[void]$sb.AppendLine("TargetName=$installerExe")
[void]$sb.AppendLine("FriendlyName=US360Host Installer")
[void]$sb.AppendLine("AppLaunched=cmd /c install.bat")
[void]$sb.AppendLine("PostInstallCmd=<None>")
[void]$sb.AppendLine("AdminQuietInstCmd=cmd /c install.bat")
[void]$sb.AppendLine("UserQuietInstCmd=cmd /c install.bat")
[void]$sb.AppendLine("SourceFiles=SourceFiles")
[void]$sb.AppendLine("[Strings]")

$fileIndex = 0
foreach ($file in $files) {
    $relative = $file.FullName.Substring($stageDir.Length).TrimStart('\')
    [void]$sb.AppendLine(('FILE{0}="{1}"' -f $fileIndex, $relative.Replace('\', '\\')))
    $fileIndex += 1
}

[void]$sb.AppendLine("[SourceFiles]")
foreach ($dir in $dirToGroup.Keys) {
    [void]$sb.AppendLine(('SourceFiles{0}={1}' -f $dirToGroup[$dir], $dir))
}

$fileIndex = 0
foreach ($dir in $dirToGroup.Keys) {
    [void]$sb.AppendLine(("[SourceFiles{0}]" -f $dirToGroup[$dir]))
    foreach ($file in $files | Where-Object { (Split-Path -Parent $_.FullName) -eq $dir }) {
        [void]$sb.AppendLine(('%FILE{0}%=' -f $fileIndex))
        $fileIndex += 1
    }
}

Set-Content -Path $sedFile -Value $sb.ToString() -Encoding ASCII

Write-Host "[6/6] Build installer package..."
$iexpress = Join-Path $env:SystemRoot "System32\iexpress.exe"
if (Test-Path $installerExe) {
    Remove-Item -Force $installerExe
}
& $iexpress /N $sedFile
if (-not (Test-Path $installerExe)) {
    Write-Warning "IExpress did not produce EXE. Installer zip will be used instead: $installerBundleZip"
}

Write-Host ""
Write-Host "Build complete:"
Write-Host "  Portable folder : $portableDir"
Write-Host "  Portable zip    : $zipFile"
Write-Host "  Installer folder: $installerBundleDir"
Write-Host "  Installer zip   : $installerBundleZip"
if (Test-Path $installerExe) {
    Write-Host "  Installer exe   : $installerExe"
}
