#Requires -Version 5.0
# One-time setup for native Windows (cmd.exe or PowerShell, no Git Bash needed): installs the
# Pedro visualizer's dependencies and adds this scripts\ directory to your user PATH, so
# `visualizer` works as a bare command in every future terminal on this machine. Re-running this
# script is safe (idempotent).
$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VisualizerDir = (Resolve-Path (Join-Path $ScriptDir "..\tools\pedro-visualizer")).Path

if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
    Write-Error "npm was not found on PATH. Install Node.js (https://nodejs.org/) first, then re-run this script."
    exit 1
}

Write-Host "Installing Pedro visualizer dependencies..."
Push-Location $VisualizerDir
try {
    & npm ci
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} finally {
    Pop-Location
}

$currentPath = [Environment]::GetEnvironmentVariable("Path", "User")
$pathEntries = @()
if ($currentPath) { $pathEntries = $currentPath -split ";" }

if ($pathEntries -contains $ScriptDir) {
    Write-Host "Already on PATH: $ScriptDir"
} else {
    $newPath = if ([string]::IsNullOrEmpty($currentPath)) { $ScriptDir } else { "$currentPath;$ScriptDir" }
    [Environment]::SetEnvironmentVariable("Path", $newPath, "User")
    Write-Host "Added $ScriptDir to your user PATH."
}

Write-Host ""
Write-Host "Setup complete. Open a new terminal and type: visualizer"
