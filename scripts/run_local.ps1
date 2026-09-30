# Run QuadKen locally on Windows PC (with simulated ESPs and Rerun Visualizer)
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "   Starting QuadKen Dora v2 (Local PC Mode)      " -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan

# 1. Start ESP Simulator in background process
Write-Host "[1/2] Starting ESP1 & ESP2 Simulator..." -ForegroundColor Yellow
$espProcess = Start-Process -FilePath "uv" -ArgumentList "run", "python", "esp/esp_simulator.py" -PassThru -NoNewWindow

# Give simulator a moment to bind ports
Start-Sleep -Seconds 1

# 2. Run Dora Dataflow
Write-Host "[2/2] Launching Dora Dataflow..." -ForegroundColor Green
try {
    uv run dora run dataflow_local.yml --uv --log-filter "camera=error"
} finally {
    Write-Host "`nStopping ESP Simulator..." -ForegroundColor Yellow
    if ($espProcess -and -not $espProcess.HasExited) {
        Stop-Process -Id $espProcess.Id -Force -ErrorAction SilentlyContinue
    }
    Write-Host "Finished." -ForegroundColor Cyan
}
