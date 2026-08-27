$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$RuntimeDir = Join-Path $Root ".run"
$FrontendDir = Join-Path $Root "frontend"
$BackendScriptPattern = "backend[/\\]api[/\\]app.py"
$RootPattern = [regex]::Escape($Root)
$FrontendPattern = [regex]::Escape($FrontendDir)
$PidFiles = @(
    (Join-Path $RuntimeDir "frontend.pid"),
    (Join-Path $RuntimeDir "backend.pid")
)

function Stop-ProcessTree {
    param([int]$ProcessId)

    $children = Get-CimInstance Win32_Process -Filter "ParentProcessId = $ProcessId" -ErrorAction SilentlyContinue
    foreach ($child in $children) {
        Stop-ProcessTree -ProcessId $child.ProcessId
    }

    $process = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    if ($process) {
        Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
        Wait-Process -Id $ProcessId -Timeout 5 -ErrorAction SilentlyContinue
        Write-Host "Stopped process $ProcessId"
    }
}

function Stop-WorkspaceProcesses {
    $processes = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue | Where-Object {
        $commandLine = $_.CommandLine
        if (-not $commandLine -or $_.ProcessId -eq $PID) {
            return $false
        }

        $isFrontend = ($commandLine -match $FrontendPattern -and $commandLine -match "vite")
        $isBackend = ($commandLine -match $RootPattern -and $commandLine -match $BackendScriptPattern)
        return ($isFrontend -or $isBackend)
    }

    foreach ($process in $processes) {
        Stop-ProcessTree -ProcessId $process.ProcessId
    }
}

function Remove-RuntimeDir {
    if (-not (Test-Path $RuntimeDir)) {
        return
    }

    for ($attempt = 0; $attempt -lt 20; $attempt++) {
        try {
            Remove-Item $RuntimeDir -Force -Recurse -ErrorAction Stop
            return
        }
        catch {
            Start-Sleep -Milliseconds 250
        }
    }

    if (Test-Path $RuntimeDir) {
        throw "Unable to remove runtime directory: $RuntimeDir"
    }
}

foreach ($pidFile in $PidFiles) {
    if (-not (Test-Path $pidFile)) {
        continue
    }

    $processId = Get-Content $pidFile -ErrorAction SilentlyContinue
    if ($processId) {
        Stop-ProcessTree -ProcessId $processId
    }
    Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
}

Stop-WorkspaceProcesses
Remove-RuntimeDir

Write-Host "Workbench services stopped."
