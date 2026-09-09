# ==============================================================================
# daily_push.ps1 - Automated Progressive Daily File Deployment Engine
# ==============================================================================
param (
    [switch]$Status,
    [int]$Day = 0
)

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
Set-Location -Path $ScriptDir

$ManifestPath = Join-Path $ScriptDir "push_manifest.json"
$LogPath = Join-Path $ScriptDir "daily_push.log"

function Write-Log {
    param([string]$Message)
    $Timestamp = (Get-Date -Format "yyyy-MM-dd HH:mm:ss")
    $Entry = "[$Timestamp] $Message"
    Write-Output $Entry
    Add-Content -Path $LogPath -Value $Entry -ErrorAction SilentlyContinue
}

if (-not (Test-Path $ManifestPath)) {
    Write-Log "ERROR: push_manifest.json not found in $ScriptDir"
    exit 1
}

$Manifest = Get-Content -Raw -Path $ManifestPath | ConvertFrom-Json

# Handle Status Check Mode
if ($Status) {
    Write-Output "=== Daily Push Pipeline Status ==="
    foreach ($entry in $Manifest) {
        $mark = if ($entry.status -eq "pushed") { "[DONE]" } else { "[PEND]" }
        $timeStr = if ($entry.pushed_at) { " (Pushed: $($entry.pushed_at))" } else { "" }
        Write-Output "$mark Day $($entry.day): $($entry.commit_message)$timeStr"
        foreach ($f in $entry.files) {
            Write-Output "       -> $f"
        }
    }
    exit 0
}

# Determine target item
$TargetItem = $null
if ($Day -gt 0) {
    $TargetItem = $Manifest | Where-Object { $_.day -eq $Day }
    if (-not $TargetItem) {
        Write-Log "ERROR: Day $Day was not found in manifest."
        exit 1
    }
} else {
    $TargetItem = $Manifest | Where-Object { $_.status -eq "pending" } | Select-Object -First 1
}

if (-not $TargetItem) {
    Write-Log "INFO: All scheduled files have already been pushed to GitHub. Queue complete."
    exit 0
}

Write-Log "INFO: Processing Day $($TargetItem.day): $($TargetItem.commit_message)"

# Ensure git remote is active
$RemoteUrl = git config --get remote.origin.url
if (-not $RemoteUrl) {
    Write-Log "ERROR: Git remote 'origin' is not configured."
    exit 1
}

# Stage files for this specific day
$FilesStaged = 0
foreach ($f in $TargetItem.files) {
    $FullPath = Join-Path $ScriptDir $f
    if (Test-Path $FullPath) {
        git add $f
        Write-Log "STAGED: $f"
        $FilesStaged++
    } else {
        Write-Log "WARNING: File $f was not found on disk. Skipping."
    }
}

if ($FilesStaged -eq 0) {
    Write-Log "ERROR: No valid files were found to stage for Day $($TargetItem.day)."
    exit 1
}

# Also stage push_manifest.json so remote maintains up-to-date queue state
$TargetItem.status = "pushed"
$TargetItem.pushed_at = (Get-Date -Format "o")
$Manifest | ConvertTo-Json -Depth 5 | Set-Content -Path $ManifestPath -Encoding UTF8
git add push_manifest.json

# Commit staged changes
$CommitMsg = $TargetItem.commit_message
git commit -m "$CommitMsg"
if ($LASTEXITCODE -ne 0) {
    Write-Log "WARNING: Git commit returned non-zero code or nothing new to commit."
}

# Push to GitHub
Write-Log "INFO: Pushing commit to origin main..."
git push origin main
if ($LASTEXITCODE -eq 0) {
    Write-Log "SUCCESS: Day $($TargetItem.day) successfully committed and pushed to GitHub."
} else {
    Write-Log "ERROR: Git push failed with exit code $LASTEXITCODE. Rolling back manifest status."
    $TargetItem.status = "pending"
    $TargetItem.pushed_at = $null
    $Manifest | ConvertTo-Json -Depth 5 | Set-Content -Path $ManifestPath -Encoding UTF8
    exit 1
}
