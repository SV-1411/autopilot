# Verification matrix for the autopilot: runs the key benchmark cells
# sequentially through the fail-fast wrapper and writes one consolidated
# report. Exit code = number of failed cells.
#
#   powershell -File tools\run_matrix.ps1 [-Runs 60] [-OutFile eval_matrix.txt]

param(
    [int]$Runs = 60,
    [string]$OutFile = "D:\Auto-pilot\autopilot_space_sim_3d_godot\eval_matrix.txt"
)

$wrapper = Join-Path $PSScriptRoot "run_headless.ps1"
$failures = 0

# name | tool args | purpose
$cells = @(
    @{ name = "canary_union_26";  args = "--runs $Runs --asteroids 26 --seed 0 --planner union";
       note = "legacy union model MUST show ~7% NO_PATH (proves the matrix detects what time-indexing fixed)" },
    @{ name = "dense_34";         args = "--runs $Runs --asteroids 34 --seed 0 --gate-collisions 0";
       note = "dense-belt stress" },
    @{ name = "noise_no_unc";     args = "--runs $Runs --asteroids 26 --seed 0 --noise 0.5";
       note = "imperfect tracking, deterministic margins only" },
    @{ name = "noise_with_unc";   args = "--runs $Runs --asteroids 26 --seed 0 --noise 0.5 --uncertainty --gate-collisions 0";
       note = "imperfect tracking + chance-constrained 3-sigma shells" },
    @{ name = "sparse_18_x100";   args = "--runs 100 --asteroids 18 --seed 0 --gate-success 95 --gate-collisions 0 --gate-nopath 2";
       note = "comparable to the original 100-run baseline (92% / 1 collision / 7% NO_PATH)" }
)

"=== Autopilot verification matrix  $(Get-Date -Format 'yyyy-MM-dd HH:mm') ===" | Out-File $OutFile -Encoding utf8

foreach ($cell in $cells) {
    $banner = "`n########## CELL: $($cell.name) -- $($cell.note)`n"
    Write-Host $banner
    $banner | Out-File $OutFile -Append -Encoding utf8
    & $wrapper -Script "res://tools/BatchEval.gd" -ToolArgs $cell.args -TimeoutSec 2400 *>&1 |
        Tee-Object -Variable cellOut | Out-Null
    $code = $LASTEXITCODE
    $cellOut | Out-File $OutFile -Append -Encoding utf8
    "CELL_EXIT=$code" | Out-File $OutFile -Append -Encoding utf8
    Write-Host "CELL $($cell.name): exit $code"
    if ($code -ne 0 -and $cell.name -ne "canary_union_26") { $failures += 1 }
}

"`n=== Matrix complete: $failures gated cell(s) failed ===" | Out-File $OutFile -Append -Encoding utf8
Write-Host "Matrix complete: $failures gated cell(s) failed. Full report: $OutFile"
exit $failures
