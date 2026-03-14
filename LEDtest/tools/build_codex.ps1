$ErrorActionPreference = "Stop"

$workspace = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $workspace "build_codex"
$toolchain = "D:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin"

$cc = Join-Path $toolchain "arm-none-eabi-gcc.exe"
$objcopy = Join-Path $toolchain "arm-none-eabi-objcopy.exe"
$size = Join-Path $toolchain "arm-none-eabi-size.exe"

$target = "LEDtest"
$elf = Join-Path $buildDir "$target.elf"
$hex = Join-Path $buildDir "$target.hex"
$bin = Join-Path $buildDir "$target.bin"
$map = Join-Path $buildDir "$target.map"
$linkerScript = Join-Path $workspace "STM32F103XX_FLASH.ld"

$sources = @(
    "Core/Src/main.c",
    "Core/Src/gpio.c",
    "Core/Src/stm32f1xx_it.c",
    "Core/Src/stm32f1xx_hal_msp.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c",
    "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c",
    "Core/Src/system_stm32f1xx.c",
    "Core/Src/sysmem.c",
    "Core/Src/syscalls.c",
    "Core/Src/dma.c",
    "Core/Src/usart.c",
    "Core/Src/sonar_protocol.c",
    "Core/Src/sonar_app.c",
    "startup_stm32f103xe.s"
)

$includes = @(
    "Core/Inc",
    "Drivers/STM32F1xx_HAL_Driver/Inc/Legacy",
    "Drivers/STM32F1xx_HAL_Driver/Inc",
    "Drivers/CMSIS/Device/ST/STM32F1xx/Include",
    "Drivers/CMSIS/Include"
) | ForEach-Object { "-I$workspace/$_" }

$defines = @(
    "-DUSE_HAL_DRIVER",
    "-DSTM32F103xE"
)

$commonFlags = @(
    "-mcpu=cortex-m3",
    "-mthumb",
    "-Og",
    "-g",
    "-gdwarf-2",
    "-Wall",
    "-fdata-sections",
    "-ffunction-sections"
) + $defines + $includes

$linkFlags = @(
    "-mcpu=cortex-m3",
    "-mthumb",
    "-specs=nano.specs",
    "-T$linkerScript",
    "-Wl,-Map=$map,--cref",
    "-Wl,--gc-sections",
    "-lc",
    "-lm",
    "-lnosys"
)

if (Test-Path $buildDir) {
    Remove-Item $buildDir -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$objects = @()
foreach ($source in $sources) {
    $sourcePath = Join-Path $workspace $source
    $objectName = [System.IO.Path]::GetFileNameWithoutExtension($source) + ".o"
    $objectPath = Join-Path $buildDir $objectName
    $objects += $objectPath

    if ($source.ToLower().EndsWith(".s")) {
        & $cc "-c" "-x" "assembler-with-cpp" @commonFlags $sourcePath "-o" $objectPath
    } else {
        & $cc "-c" @commonFlags "-MMD" "-MP" $sourcePath "-o" $objectPath
    }

    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

& $cc @objects @linkFlags "-o" $elf
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $objcopy "-O" "ihex" $elf $hex
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $objcopy "-O" "binary" "-S" $elf $bin
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $size $elf
exit $LASTEXITCODE
