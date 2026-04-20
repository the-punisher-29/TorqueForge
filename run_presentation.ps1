# TorqueForge — Unified Presentation Launcher (Windows / PowerShell)
# Usage:
#   .\run_presentation.ps1            # build then run all demos
#   .\run_presentation.ps1 -NoRebuild # skip build, just run
param([switch]$NoRebuild)

$ErrorActionPreference = "Continue"   # keep going if one demo crashes

$Root   = Split-Path -Parent $MyInvocation.MyCommand.Path
$Build  = Join-Path $Root "build"
$Demos  = Join-Path $Build "bin\demos"

# ── Optional rebuild ──────────────────────────────────────────────────────────
if (-not $NoRebuild) {
    Write-Host "==> Building TorqueForge (Release)..." -ForegroundColor Cyan

    # Try Ninja first (faster), fall back to default VS generator
    $ninjaAvail = (Get-Command ninja -ErrorAction SilentlyContinue) -ne $null
    if ($ninjaAvail) {
        cmake -S $Root -B $Build -DCMAKE_BUILD_TYPE=Release -G Ninja
    } else {
        cmake -S $Root -B $Build -DCMAKE_BUILD_TYPE=Release
    }
    cmake --build $Build --config Release --parallel
    Write-Host "==> Build complete." -ForegroundColor Green
}

# ── Helper: run a demo, wait for it to exit ───────────────────────────────────
function Run-Demo {
    param([string]$Label, [string]$Exe, [string]$Input = "")

    Write-Host ""
    Write-Host ("━" * 60) -ForegroundColor Yellow
    Write-Host "  DEMO: $Label" -ForegroundColor Yellow
    Write-Host "  (Close the window or press ESC to advance)" -ForegroundColor DarkGray
    Write-Host ("━" * 60) -ForegroundColor Yellow
    Start-Sleep -Seconds 1

    Set-Location $Root   # renderer config JSONs are resolved from CWD

    if ($Input -ne "") {
        # pipe a single choice to the demo (e.g. scene selector)
        $Input | & $Exe
    } else {
        & $Exe
    }
}

# ── Demo sequence ─────────────────────────────────────────────────────────────

Run-Demo "Wrecking Ball — Impulse Contact & Rigid Bodies" `
    "$Demos\demo_load_scene.exe"

Run-Demo "Scissorlift — Closed Kinematic Chain / Loop Closure" `
    "$Demos\demo_load_scene_articulated.exe" "0"

Run-Demo "Spring — Prismatic Joint & Spring-Damper" `
    "$Demos\demo_load_scene_articulated.exe" "1"

Run-Demo "Spherical Joint Chain — 3-DOF Joints" `
    "$Demos\demo_load_scene_articulated.exe" "2"

Run-Demo "Spring Scale — Torque Balance" `
    "$Demos\demo_spring_scale.exe"

Write-Host ""
Write-Host ("━" * 60) -ForegroundColor Yellow
Write-Host "  DEMO: Chain Pendulum — Scaling Benchmark (30 links)" -ForegroundColor Yellow
Write-Host "  (Close the window or press ESC to advance)" -ForegroundColor DarkGray
Write-Host ("━" * 60) -ForegroundColor Yellow
Start-Sleep -Seconds 1
Set-Location $Root
& "$Demos\demo_chain_pendulum.exe" 30

Write-Host ""
Write-Host "==> Presentation complete." -ForegroundColor Green
