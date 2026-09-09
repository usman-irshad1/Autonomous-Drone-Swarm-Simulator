# ==============================================================================
# setup_daily_task.ps1 - Windows Task Scheduler Setup for Daily Pusher
# ==============================================================================
param (
    [string]$Time = "10:00",
    [switch]$Unregister
)

$TaskName = "AutonomousDroneSwarm_DailyPusher"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$TargetScript = Join-Path $ScriptDir "daily_push.ps1"

if ($Unregister) {
    Write-Output "Unregistering scheduled task: $TaskName..."
    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false -ErrorAction SilentlyContinue
    Write-Output "Task unregistered successfully."
    exit 0
}

Write-Output "Configuring scheduled task: $TaskName"
Write-Output "Execution Target: $TargetScript"
Write-Output "Daily Trigger Time: $Time"

$Action = New-ScheduledTaskAction -Execute "powershell.exe" -Argument "-ExecutionPolicy Bypass -WindowStyle Hidden -File `"$TargetScript`""
$Trigger = New-ScheduledTaskTrigger -Daily -At $Time
$Settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable -MultipleInstances IgnoreNew

try {
    # Unregister any existing instance first
    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false -ErrorAction SilentlyContinue

    # Register new task for the current user
    Register-ScheduledTask -TaskName $TaskName -Action $Action -Trigger $Trigger -Settings $Settings -Description "Automated daily single-file commit and push pipeline for Autonomous-Drone-Swarm-Simulator"
    Write-Output "SUCCESS: Windows Scheduled Task '$TaskName' registered successfully."
    Write-Output "It will execute daily at $Time automatically."
} catch {
    Write-Output "WARNING: PowerShell Register-ScheduledTask failed: $_"
    Write-Output "Attempting fallback via schtasks.exe..."

    $cmd = "powershell.exe -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$TargetScript`""
    $schOut = schtasks.exe /Create /SC DAILY /TN $TaskName /TR $cmd /ST $Time /F
    Write-Output $schOut
}
