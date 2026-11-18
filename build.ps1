# build.ps1 - 命令行编译小车工程（不打开 CCS 也能验证代码）
# 用法：右键“使用 PowerShell 运行”，或 powershell -ExecutionPolicy Bypass -File build.ps1

$ErrorActionPreference = "Stop"

# 让中文输出在控制台正常显示
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch { }
$OutputEncoding = New-Object System.Text.UTF8Encoding -ArgumentList $false

$root      = Split-Path -Parent $MyInvocation.MyCommand.Path
$sdk       = "C:\ti\mspm0_sdk_2_11_00_07"
$sysconfig = "C:\ti\sysconfig_1.26.2\sysconfig_cli.bat"
$compiler  = "C:\ti\ti_cgt_arm_llvm_4.0.2.LTS"
$cc        = "$compiler\bin\tiarmclang.exe"
$out       = Join-Path $root "build"

New-Item -ItemType Directory -Force -Path $out | Out-Null

Write-Host "[1/4] SysConfig 生成配置..."
& $sysconfig --compiler ticlang --product "$sdk\.metadata\product.json" --output $out "$root\car.syscfg"
if ($LASTEXITCODE -ne 0) { throw "SysConfig 失败，检查 car.syscfg" }

Write-Host "[2/4] 编译..."
$common = @(
    "-mcpu=cortex-m0plus", "-march=thumbv6m", "-mfloat-abi=soft", "-mthumb",
    "-O2", "-gdwarf-3", "-Wall",
    "-I$root", "-I$out", "-I$sdk\source",
    "-I$sdk\source\third_party\CMSIS\Core\Include",
    "@$out\device.opt"
)

$objects = @()
foreach ($src in @("main.c", "motor.c", "encoder.c", "pid.c", "control.c",
                   "car_uart.c", "line_sensor.c", "odom.c", "line_follow.c", "button.c",
                   "mission.c", "oled.c", "oled_font.c")) {
    $obj = Join-Path $out (([IO.Path]::GetFileNameWithoutExtension($src)) + ".obj")
    & $cc @common -c (Join-Path $root $src) -o $obj
    if ($LASTEXITCODE -ne 0) { throw "编译失败：$src" }
    $objects += $obj
}

$cfgObj = Join-Path $out "ti_msp_dl_config.obj"
& $cc @common -c (Join-Path $out "ti_msp_dl_config.c") -o $cfgObj
if ($LASTEXITCODE -ne 0) { throw "编译失败：ti_msp_dl_config.c" }
$objects += $cfgObj

$startup = Join-Path $out "startup.obj"
& $cc @common -c "$sdk\source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c" -o $startup
if ($LASTEXITCODE -ne 0) { throw "编译失败：startup" }
$objects += $startup

Write-Host "[3/4] 链接..."
$elf = Join-Path $out "wheeltec_car.out"
& $cc "-Wl,-u,_c_int00" @objects `
    "-L$out" "-ldevice.cmd.genlibs" "-L$sdk\source" `
    (Join-Path $out "device_linker.cmd") `
    "-Wl,-m,$out\wheeltec_car.map" "-Wl,--rom_model" "-Wl,--warn_sections" `
    "-L$compiler\lib" "-llibc.a" `
    -o $elf
if ($LASTEXITCODE -ne 0) { throw "链接失败" }

# 顺手导出 hex/bin，免得 build 目录里躺着上一次的旧固件被误刷
Write-Host "[4/4] 生成 hex/bin..."
$hexTool = Join-Path $compiler "bin\tiarmhex.exe"
& $hexTool --intel --memwidth=8 --romwidth=8 --fill=0xFF -o (Join-Path $out "wheeltec_car.hex") $elf | Out-Null
if ($LASTEXITCODE -ne 0) { throw "hex 生成失败" }
& $hexTool --binary --memwidth=8 --romwidth=8 --fill=0xFF -o (Join-Path $out "wheeltec_car.bin") $elf | Out-Null
if ($LASTEXITCODE -ne 0) { throw "bin 生成失败" }

$size = (Get-Item $elf).Length
Write-Host ""
Write-Host "编译成功：$elf  ($size bytes)"
Get-Content "$out\wheeltec_car.map" -ErrorAction SilentlyContinue |
    Select-String -Pattern "^(FLASH|SRAM)\s" | ForEach-Object { $_.Line }
