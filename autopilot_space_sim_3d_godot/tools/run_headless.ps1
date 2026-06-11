# Fail-fast headless runner for Godot tool scripts (BatchEval, SelfTest).
#
# Why this exists: adding a class_name script without re-importing the project
# corrupts headless runs in the worst way -- the script fails to compile, the
# SceneTree never quits, and the process hangs silently, looking like a perf
# problem. This wrapper makes that class of failure loud and bounded:
#   1. Always re-import first (rebuilds .godot/global_script_class_cache.cfg).
#   2. Run the tool with a hard wall-clock timeout; kill + exit 124 on overrun.
#   3. Require the BOOT_OK sentinel within the boot window; kill + exit 125 if
#      it never appears (compile failure).
#   4. Propagate the tool's own exit code.
#
# Usage:
#   powershell -File tools\run_headless.ps1 -Script res://tools/BatchEval.gd `
#     -ToolArgs "--runs 60 --asteroids 26 --seed 0" [-TimeoutSec 1800] [-BootSec 60]

param(
    [Parameter(Mandatory = $true)][string]$Script,
    [string]$ToolArgs = "",
    [int]$TimeoutSec = 1800,
    [int]$BootSec = 60
)

$ErrorActionPreference = "Stop"
$godot = "D:\tools\godot\Godot_v4.5-stable_win64_console.exe"
$project = Split-Path -Parent $PSScriptRoot   # tools\ -> project root

if (-not (Test-Path $godot)) { Write-Host "FATAL: Godot not found at $godot"; exit 2 }

# --- 1. Import gate: rebuild the class cache so class_name scripts resolve ---
Write-Host "[run_headless] importing project..."
$import = Start-Process -FilePath $godot -ArgumentList @("--headless", "--path", "`"$project`"", "--import") `
    -NoNewWindow -PassThru -RedirectStandardOutput "$env:TEMP\godot_import_out.txt" -RedirectStandardError "$env:TEMP\godot_import_err.txt"
# PS 5.1: WaitForExit/ExitCode misbehave unless the handle is cached first.
$null = $import.Handle
if (-not $import.WaitForExit(180000)) { $import.Kill(); Write-Host "FATAL: import timed out"; exit 124 }
$importErrors = Select-String -Path "$env:TEMP\godot_import_out.txt", "$env:TEMP\godot_import_err.txt" `
    -Pattern "SCRIPT ERROR|Parse Error" -SimpleMatch:$false -ErrorAction SilentlyContinue
if ($importErrors) {
    Write-Host "FATAL: script errors at import:"
    $importErrors | ForEach-Object { Write-Host "  $($_.Line)" }
    exit 125
}
Write-Host "[run_headless] import clean."

# --- 2/3. Run the tool with boot sentinel + hard timeout ---
$outFile = Join-Path $env:TEMP ("godot_tool_out_{0}.txt" -f $PID)
$errFile = Join-Path $env:TEMP ("godot_tool_err_{0}.txt" -f $PID)
$argList = @("--headless", "--path", "`"$project`"", "--script", $Script)
if ($ToolArgs -ne "") { $argList += "--"; $argList += ($ToolArgs -split " ") }

Write-Host "[run_headless] running $Script $ToolArgs"
$proc = Start-Process -FilePath $godot -ArgumentList $argList -NoNewWindow -PassThru `
    -RedirectStandardOutput $outFile -RedirectStandardError $errFile
# PS 5.1: ExitCode reads as $null later unless the handle is cached now.
$null = $proc.Handle

$booted = $false
$deadline = (Get-Date).AddSeconds($TimeoutSec)
$bootDeadline = (Get-Date).AddSeconds($BootSec)
while (-not $proc.HasExited) {
    Start-Sleep -Milliseconds 500
    if (-not $booted) {
        if ((Test-Path $outFile) -and (Select-String -Path $outFile -Pattern "BOOT_OK" -Quiet -ErrorAction SilentlyContinue)) {
            $booted = $true
        } elseif ((Get-Date) -gt $bootDeadline) {
            $proc.Kill()
            Write-Host "FATAL: no BOOT_OK within ${BootSec}s -- compile failure or hang at startup."
            Get-Content $outFile, $errFile -ErrorAction SilentlyContinue | Select-Object -First 30 | ForEach-Object { Write-Host "  $_" }
            exit 125
        }
    }
    if ((Get-Date) -gt $deadline) {
        $proc.Kill()
        Write-Host "FATAL: tool exceeded ${TimeoutSec}s wall clock -- killed."
        exit 124
    }
}

# --- 4. Propagate output and exit code ---
Get-Content $outFile -ErrorAction SilentlyContinue | ForEach-Object { Write-Host $_ }
$errLines = Get-Content $errFile -ErrorAction SilentlyContinue | Where-Object { $_ -match "SCRIPT ERROR|ERROR|WARNING" }
if ($errLines) { $errLines | Select-Object -First 20 | ForEach-Object { Write-Host "stderr: $_" } }
# WaitForExit() is required even after HasExited: without it ExitCode can read
# as $null and `exit $null` silently becomes success -- which would swallow
# test failures, the one thing this wrapper exists to prevent.
$proc.WaitForExit()
$code = $proc.ExitCode
if ($null -eq $code) { Write-Host "FATAL: could not read exit code"; exit 3 }
exit $code
