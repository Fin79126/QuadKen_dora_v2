# Launch Distributed Dora Coordinator and PC Daemon
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "   Starting Dora Coordinator & Daemon on PC      " -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan

# 1. Bring up coordinator and local daemon
Write-Host "[1/3] Bringing up Dora Coordinator..." -ForegroundColor Yellow
uv run dora up

# 2. Status check
Write-Host "[2/3] Checking cluster status..." -ForegroundColor Yellow
Start-Sleep -Seconds 2
uv run dora status

Write-Host "`n[3/3] Ready to deploy dataflow." -ForegroundColor Green
Write-Host "Once Raspberry Pi daemon has connected, deploy the dataflow with:" -ForegroundColor Cyan
Write-Host "   uv run dora start dataflow.yml --name quadken" -ForegroundColor White
Write-Host "`nTo inspect traces:" -ForegroundColor Cyan
Write-Host "   uv run dora trace list" -ForegroundColor White
Write-Host "`nTo stop dataflow:" -ForegroundColor Cyan
Write-Host "   uv run dora stop quadken" -ForegroundColor White
Write-Host "   uv run dora down" -ForegroundColor White
