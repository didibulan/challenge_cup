param(
    [int]$Port = 2331,
    [int]$Speed = 4000,
    [string]$JLinkGDBServer = ''
)

$ErrorActionPreference = 'Stop'
if (-not $JLinkGDBServer) {
    $JLinkGDBServer = Get-ChildItem 'C:\Program Files\SEGGER\JLink_V*\JLinkGDBServerCL.exe',
        'C:\Program Files (x86)\SEGGER\JLink_V*\JLinkGDBServerCL.exe' -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $JLinkGDBServer -or -not (Test-Path -LiteralPath $JLinkGDBServer)) {
    throw '未找到 JLinkGDBServerCL.exe。请安装 SEGGER J-Link Software Pack。'
}

Get-Process -Name JLinkGDBServer,JLinkGDBServerCL -ErrorAction SilentlyContinue |
    Stop-Process -Force -ErrorAction SilentlyContinue

& $JLinkGDBServer `
    -device STM32H723VG `
    -if SWD `
    -speed $Speed `
    -port $Port `
    -swoport ($Port + 1) `
    -telnetport ($Port + 2) `
    -nogui `
    -singlerun
