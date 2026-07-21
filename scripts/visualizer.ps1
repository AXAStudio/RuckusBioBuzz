#Requires -Version 5.0
# Native Windows counterpart to the `scripts/visualizer` bash script (used on macOS/Linux/Git
# Bash). Keep behavior in sync between the two: install deps if missing/stale, start the dev
# server, wait until it's actually responding, open the browser, then stay attached so closing
# this window / Ctrl+C stops the server.
$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VisualizerDir = (Resolve-Path (Join-Path $ScriptDir "..\tools\pedro-visualizer")).Path
$HostName = "127.0.0.1"
$Port = 5173
$Url = "http://${HostName}:${Port}/"

function Test-ServerUp {
    try {
        Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 2 | Out-Null
        return $true
    } catch {
        return $false
    }
}

function Open-VisualizerUrl {
    if ($env:VISUALIZER_NO_OPEN -eq "1") { return $false }
    try {
        Start-Process $Url | Out-Null
        return $true
    } catch {
        return $false
    }
}

if (Test-ServerUp) {
    Write-Host "Pedro visualizer is already running at $Url"
    Open-VisualizerUrl | Out-Null
    exit 0
}

Push-Location $VisualizerDir
try {
    $nodeModules = Join-Path $VisualizerDir "node_modules"
    $lockFile = Join-Path $VisualizerDir "package-lock.json"
    $needsInstall = -not (Test-Path $nodeModules)
    if (-not $needsInstall -and (Test-Path $lockFile)) {
        if ((Get-Item $lockFile).LastWriteTime -gt (Get-Item $nodeModules).LastWriteTime) {
            $needsInstall = $true
        }
    }

    if ($needsInstall) {
        Write-Host "Installing Pedro visualizer dependencies..."
        & npm ci
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }

    Write-Host "Starting Pedro visualizer at $Url"
    # Wrapped in cmd.exe so PowerShell's Start-Process reliably resolves npm.cmd for a
    # detached/background process (the plain call operator resolves it fine for foreground
    # calls like `npm ci` above, but Start-Process -FilePath npm is unreliable in some
    # PowerShell/.NET versions).
    $devProcess = Start-Process -FilePath "cmd.exe" `
        -ArgumentList "/c", "npm run dev -- --host $HostName --port $Port" `
        -WorkingDirectory $VisualizerDir -PassThru -NoNewWindow

    try {
        $ready = $false
        for ($i = 0; $i -lt 60; $i++) {
            if (Test-ServerUp) { $ready = $true; break }
            if ($devProcess.HasExited) { break }
            Start-Sleep -Milliseconds 250
        }

        if ($ready) {
            if (Open-VisualizerUrl) { Write-Host "Opened $Url" } else { Write-Host "Ready at $Url" }
        } else {
            Write-Host "Server did not respond yet; opening $Url anyway."
            Open-VisualizerUrl | Out-Null
        }

        Wait-Process -Id $devProcess.Id
        exit $devProcess.ExitCode
    } finally {
        if (-not $devProcess.HasExited) {
            # taskkill /T kills the whole process tree -- Stop-Process alone would only kill the
            # cmd.exe wrapper and leave the actual node dev server running.
            & taskkill /T /F /PID $devProcess.Id 2>$null | Out-Null
        }
    }
} finally {
    Pop-Location
}
