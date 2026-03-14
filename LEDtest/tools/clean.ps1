$ErrorActionPreference = "Stop"

$workspace = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $workspace "build"

if (Test-Path $buildDir) {
    Remove-Item -Recurse -Force $buildDir
}
