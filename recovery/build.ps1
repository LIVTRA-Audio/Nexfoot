param([switch]$DebugUart)
$ErrorActionPreference = 'Stop'
$officialCommit = 'ff3b9299d1b94d834b995fd858b3f75e2faa9840'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$workspaceRoot = if ($env:NEXFOOT_WORKSPACE) {
    [System.IO.Path]::GetFullPath($env:NEXFOOT_WORKSPACE)
} else {
    Join-Path (Split-Path $repoRoot -Parent) ('.' + (Split-Path $repoRoot -Leaf) + '-work')
}
$vendorRoot = Join-Path $workspaceRoot 'cache\fw-Bootloader'
$makeExe = Join-Path $workspaceRoot 'cache\jl-sdk\tools\utils\make.exe'
$patchFile = Join-Path $PSScriptRoot 'recovery_main.patch'
$outputRoot = Join-Path $workspaceRoot 'build\recovery'
if (-not (Test-Path -LiteralPath $makeExe)) { throw 'Run python tools/nexfoot.py bootstrap first' }
if (-not (Test-Path -LiteralPath (Join-Path $vendorRoot '.git'))) {
    New-Item -ItemType Directory -Force -Path $vendorRoot | Out-Null
    git -C $vendorRoot init
    if ($LASTEXITCODE -ne 0) { throw 'Cannot initialize boot dependency' }
    git -C $vendorRoot remote add origin https://github.com/Jieli-Tech/fw-Bootloader.git
    if ($LASTEXITCODE -ne 0) { throw 'Cannot configure boot dependency' }
    git -C $vendorRoot fetch --depth 1 origin $officialCommit
    if ($LASTEXITCODE -ne 0) { throw 'Cannot fetch pinned boot dependency' }
    git -C $vendorRoot checkout --detach $officialCommit
    if ($LASTEXITCODE -ne 0) { throw 'Cannot check out pinned boot dependency' }
}
$actualCommit = (git -C $vendorRoot rev-parse HEAD).Trim()
if ($actualCommit -ne $officialCommit) { throw 'Unexpected boot dependency revision' }
$oldPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
git -C $vendorRoot apply --check $patchFile 2>$null
$applyResult = $LASTEXITCODE
$ErrorActionPreference = $oldPreference
if ($applyResult -eq 0) {
    git -C $vendorRoot apply $patchFile
    if ($LASTEXITCODE -ne 0) { throw 'Cannot apply recovery patch' }
} else {
    $ErrorActionPreference = 'Continue'
    git -C $vendorRoot apply --reverse --check $patchFile 2>$null
    $reverseResult = $LASTEXITCODE
    $ErrorActionPreference = $oldPreference
    if ($reverseResult -ne 0) { throw 'Recovery patch is neither applicable nor already applied' }
}
$bootRoot = Join-Path $vendorRoot 'user_boot'
Push-Location $bootRoot
try {
    & $makeExe clean_bd19_uboot
    if ($LASTEXITCODE -ne 0) { throw 'Failed to clean recovery boot' }
    if ($DebugUart) { & $makeExe bd19_uboot -j4 'EXT_CFLAGS=-D__DEBUG' }
    else { & $makeExe bd19_uboot -j4 }
    if ($LASTEXITCODE -ne 0) { throw 'Failed to build recovery boot' }
} finally { Pop-Location }
New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null
$outputName = if ($DebugUart) { 'uboot_debug.boot' } else { 'uboot.boot' }
$outputFile = Join-Path $outputRoot $outputName
Copy-Item (Join-Path $bootRoot 'cpu\bd19\output\uboot.boot') $outputFile -Force
$hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $outputFile).Hash.ToLowerInvariant()
Write-Output "Recovery candidate: $outputFile sha256=$hash"
Write-Output 'No approved artifact was replaced; no device flash was written.'
