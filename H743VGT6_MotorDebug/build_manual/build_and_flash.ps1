$ErrorActionPreference = 'Stop'

$projectRoot = 'D:\US360Scan\H743VGT6_MotorDebug'
$buildDir = Join-Path $projectRoot 'build_manual'
$gcc = 'D:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe'
$objcopy = 'D:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-objcopy.exe'
$size = 'D:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-size.exe'
$pyocd = 'C:\Users\Kan\AppData\Local\Programs\Python\Python311\Scripts\pyocd.exe'

$includes = @(
    '-ICore/Inc',
    '-IDrivers/STM32H7xx_HAL_Driver/Inc',
    '-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy',
    '-IDrivers/CMSIS/Device/ST/STM32H7xx/Include',
    '-IDrivers/CMSIS/Include'
)

$defines = @(
    '-DUSE_PWR_LDO_SUPPLY',
    '-DUSE_HAL_DRIVER',
    '-DSTM32H743xx'
)

$commonFlags = @(
    '-mcpu=cortex-m7',
    '-mthumb',
    '-mfpu=fpv5-d16',
    '-mfloat-abi=hard'
) + $defines + $includes + @(
    '-Og',
    '-Wall',
    '-fdata-sections',
    '-ffunction-sections',
    '-g',
    '-gdwarf-2'
)

$sources = @(
    'Core/Src/main.c',
    'Core/Src/gpio.c',
    'Core/Src/adc.c',
    'Core/Src/dma.c',
    'Core/Src/tim.c',
    'Core/Src/usart.c',
    'Core/Src/motor_driver.c',
    'Core/Src/sonar_protocol.c',
    'Core/Src/sonar_app.c',
    'Core/Src/stm32h7xx_it.c',
    'Core/Src/stm32h7xx_hal_msp.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c',
    'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c',
    'Core/Src/system_stm32h7xx.c',
    'Core/Src/sysmem.c',
    'Core/Src/syscalls.c'
)

if (Test-Path $buildDir) {
    Get-ChildItem $buildDir -File | Where-Object { $_.Extension -in '.o', '.d', '.elf', '.hex', '.bin', '.map', '.lst' } | Remove-Item -Force
} else {
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

$objects = @()

Push-Location $projectRoot

foreach ($source in $sources) {
    $object = Join-Path $buildDir (([IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
    & $gcc -c @commonFlags $source -o $object
    if ($LASTEXITCODE -ne 0) {
        throw "compile failed: $source"
    }
    $objects += $object
}

$startupObject = Join-Path $buildDir 'startup_stm32h743xx.o'
& $gcc -x assembler-with-cpp -c @commonFlags 'startup_stm32h743xx.s' -o $startupObject
if ($LASTEXITCODE -ne 0) {
    throw 'startup compile failed'
}
$objects += $startupObject

$elf = Join-Path $buildDir 'MotorBridge.elf'
$hex = Join-Path $buildDir 'MotorBridge.hex'
$bin = Join-Path $buildDir 'MotorBridge.bin'

& $gcc @objects '-mcpu=cortex-m7' '-mthumb' '-mfpu=fpv5-d16' '-mfloat-abi=hard' '-specs=nano.specs' '-TSTM32H743XG_FLASH.ld' '-lc' '-lm' '-lnosys' '-Wl,-Map=build_manual/MotorBridge.map,--cref' '-Wl,--gc-sections' -o $elf
if ($LASTEXITCODE -ne 0) {
    throw 'link failed'
}

& $size $elf
& $objcopy -O ihex $elf $hex
& $objcopy -O binary -S $elf $bin

& $pyocd load -W -t stm32h743xx -f 1000k -M attach $hex
if ($LASTEXITCODE -ne 0) {
    throw 'flash failed'
}

Pop-Location
