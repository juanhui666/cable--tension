param(
    [string]$BackendHost = "127.0.0.1",
    [int]$BackendPort = 8765,
    [string]$FrontendHost = "127.0.0.1",
    [int]$FrontendPort = 5173,
    [int]$HealthTimeoutSeconds = 120,
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$FrontendDir = Join-Path $Root "frontend"
$BackendUrl = "http://${BackendHost}:${BackendPort}"
$FrontendUrl = "http://${FrontendHost}:${FrontendPort}"
$BackendHealthUrl = "$BackendUrl/api/v1/health"
# Default health URL: http://127.0.0.1:8765/api/v1/health
# Default frontend URL: http://127.0.0.1:5173/
$RuntimeDir = Join-Path $Root ".run"
$BackendPidFile = Join-Path $RuntimeDir "backend.pid"
$FrontendPidFile = Join-Path $RuntimeDir "frontend.pid"
$BackendLogFile = Join-Path $RuntimeDir "backend.log"
$BackendErrFile = Join-Path $RuntimeDir "backend.err.log"
$FrontendLogFile = Join-Path $RuntimeDir "frontend.log"
$FrontendErrFile = Join-Path $RuntimeDir "frontend.err.log"

function Resolve-CommandPath {
    param([string[]]$Names)

    foreach ($Name in $Names) {
        $command = Get-Command $Name -ErrorAction SilentlyContinue
        if ($command) {
            return $command.Source
        }
    }

    throw "Command not found: $($Names -join ', '). Install it and run this script again."
}

function Test-HttpOk {
    param([string]$Url)
    try {
        $response = Invoke-WebRequest -UseBasicParsing -Uri $Url -TimeoutSec 2
        return ($response.StatusCode -ge 200 -and $response.StatusCode -lt 400)
    }
    catch {
        return $false
    }
}

function Wait-HttpOk {
    param(
        [string]$Url,
        [string]$Name,
        [int]$TimeoutSeconds,
        [string]$LogHint = ""
    )

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-HttpOk $Url) {
            Write-Host "${Name} ready: ${Url}"
            return
        }
        Start-Sleep -Milliseconds 700
    }
    if ($LogHint) {
        throw "${Name} startup timed out: ${Url}. Check logs: ${LogHint}"
    }
    throw "${Name} startup timed out: ${Url}"
}

function Stop-ProcessTree {
    param([int]$ProcessId)

    $children = Get-CimInstance Win32_Process -Filter "ParentProcessId = $ProcessId" -ErrorAction SilentlyContinue
    foreach ($child in $children) {
        Stop-ProcessTree -ProcessId $child.ProcessId
    }

    if (Get-Process -Id $ProcessId -ErrorAction SilentlyContinue) {
        Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
        Wait-Process -Id $ProcessId -Timeout 5 -ErrorAction SilentlyContinue
    }
}

function Stop-PidFileProcess {
    param([string]$Path)
    if (-not (Test-Path $Path)) {
        return
    }

    $processId = Get-Content $Path -ErrorAction SilentlyContinue
    if ($processId) {
        Stop-ProcessTree -ProcessId $processId
    }
    Remove-Item $Path -Force -ErrorAction SilentlyContinue
}

New-Item -ItemType Directory -Force -Path $RuntimeDir | Out-Null

$PythonCommand = Resolve-CommandPath -Names @("python.exe", "python")
$NodeCommand = Resolve-CommandPath -Names @("node.exe", "node")
$NpmCommand = Resolve-CommandPath -Names @("npm.cmd", "npm")
Write-Host "Using Python: $PythonCommand"
Write-Host "Using Node: $NodeCommand"
Write-Host "Using npm: $NpmCommand"

if (-not $SkipInstall) {
    Push-Location $FrontendDir
    try {
        if (-not (Test-Path (Join-Path $FrontendDir "node_modules"))) {
            Write-Host "Installing frontend dependencies..."
            & $NpmCommand ci
        }
        else {
            Write-Host "frontend/node_modules exists; skipping npm ci."
        }
    }
    finally {
        Pop-Location
    }
}

# Clean up processes created by a previous run of this script.
Stop-PidFileProcess $BackendPidFile
Stop-PidFileProcess $FrontendPidFile

if (-not (Test-HttpOk $BackendHealthUrl)) {
    $backendArgs = @(
        "backend/api/app.py",
        "--host", $BackendHost,
        "--port", $BackendPort
    )
    $backend = Start-Process -FilePath $PythonCommand -ArgumentList $backendArgs -WorkingDirectory $Root -PassThru -WindowStyle Hidden -RedirectStandardOutput $BackendLogFile -RedirectStandardError $BackendErrFile
    Set-Content -Path $BackendPidFile -Value $backend.Id -Encoding ASCII
}
else {
    Write-Host "Backend is already running: $BackendHealthUrl"
}

Wait-HttpOk -Url $BackendHealthUrl -Name "Backend" -TimeoutSeconds $HealthTimeoutSeconds -LogHint "$BackendLogFile, $BackendErrFile"

if (-not (Test-HttpOk $FrontendUrl)) {
    $frontendArgs = @("run", "dev", "--", "--host", $FrontendHost, "--port", "$FrontendPort")
    $frontend = Start-Process -FilePath $NpmCommand -ArgumentList $frontendArgs -WorkingDirectory $FrontendDir -PassThru -WindowStyle Hidden -RedirectStandardOutput $FrontendLogFile -RedirectStandardError $FrontendErrFile
    Set-Content -Path $FrontendPidFile -Value $frontend.Id -Encoding ASCII
}
else {
    Write-Host "Frontend is already running: $FrontendUrl"
}

Wait-HttpOk -Url $FrontendUrl -Name "Frontend" -TimeoutSeconds $HealthTimeoutSeconds -LogHint "$FrontendLogFile, $FrontendErrFile"

Write-Host "Workbench is ready: $FrontendUrl"
Start-Process $FrontendUrl
Write-Host "To stop services started by this script, run: .\stop-workbench.ps1"
