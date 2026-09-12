param(
    [switch]$SelfCheck,
    [switch]$Tests,
    [switch]$Validate,
    [int]$Cases = 2,
    [int]$Seed = 20260912,
    [string]$RobotId = '',
    [string]$PythonPath = '',
    [string]$BaseUrl = 'http://127.0.0.1:2026'
)
$ErrorActionPreference = 'Stop'
if (([int]$SelfCheck.IsPresent + [int]$Tests.IsPresent + [int]$Validate.IsPresent) -gt 1) {
    throw 'Choose only one of SelfCheck, Tests, or Validate.'
}
if (-not $PythonPath) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    if (Test-Path -LiteralPath $bundledPython) {
        $PythonPath = $bundledPython
    } else {
        $PythonPath = (Get-Command python -ErrorAction Stop).Source
    }
}
$taskArgs = @('-B', (Join-Path $PSScriptRoot 'launch.py'))
if ($SelfCheck) {
    $taskArgs += '--self-check'
} elseif ($Tests) {
    $taskArgs += '--tests'
} elseif ($Validate) {
    $taskArgs += @('--validate', '--cases', "$Cases", '--seed', "$Seed")
} else {
    $taskArgs += @('--strategy', 'v5_dynamic', '--base-url', $BaseUrl)
    if ($RobotId) { $taskArgs += @('--robot-id', $RobotId) }
}
& $PythonPath @taskArgs
exit $LASTEXITCODE
