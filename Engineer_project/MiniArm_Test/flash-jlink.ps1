param(
    [string]$Elf = (Join-Path $PSScriptRoot 'build\Debug\MiniArm_Test.elf'),
    [string]$JLinkExe = '',
    [switch]$KeepServer
)

$ErrorActionPreference = 'Stop'
if (-not $JLinkExe) {
    $candidates = @(
        'C:\Program Files\SEGGER\JLink_V*\JLink.exe',
        'C:\Program Files (x86)\SEGGER\JLink_V*\JLink.exe',
        'C:\Program Files\SEGGER\JLink\JLink.exe'
    )
    $JLinkExe = $candidates | ForEach-Object { Get-Item $_ -ErrorAction SilentlyContinue } |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $JLinkExe -or -not (Test-Path -LiteralPath $JLinkExe)) { throw '未找到 SEGGER JLink.exe。请安装新版 J-Link Software Pack。' }
if (-not (Test-Path -LiteralPath $Elf)) { throw "未找到 ELF: $Elf" }

# J-Link/OpenOCD clients keep an exclusive USB handle. Release stale clients
# left by a failed CLion run before starting the command-line programmer.
if (-not $KeepServer) {
    Get-Process -Name JLink,JLinkGUIServer,JLinkGDBServer,JLinkGDBServerCL,openocd -ErrorAction SilentlyContinue |
        Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 300
}

$version = (Get-Item -LiteralPath $JLinkExe).VersionInfo.FileVersion
if ($version -and [version]$version -lt [version]'7.0.0') {
    throw "检测到 J-Link $version。STM32H723 需要 J-Link Software Pack 7.x 或更新版本，旧版会连接但无法正确烧录。"
}

$cmd = Join-Path $env:TEMP 'MiniArm_Test_jlink.jlink'
@(
    'device STM32H723VG',
    'si SWD',
    'speed 4000',
    'connect',
    'r',
    'h',
    "loadfile `"$Elf`"",
    'verify',
    'r',
    'g',
    'exit'
) | Set-Content -LiteralPath $cmd -Encoding ASCII

& $JLinkExe -CommandFile $cmd
if ($LASTEXITCODE -ne 0) { throw "J-Link 烧录失败，退出码 $LASTEXITCODE" }
