param(
    [string]$Compiler = 'g++',
    [string]$NinjaPath = ''
)

$ErrorActionPreference = 'Stop'
$testFirmwareDir = Split-Path -Parent $PSScriptRoot
$testCompiler = (Get-Command $Compiler -ErrorAction Stop).Source
if ($NinjaPath) {
    $testGenerator = @('-G', 'Ninja', "-DCMAKE_MAKE_PROGRAM=$((Get-Command $NinjaPath).Source)")
} elseif (Get-Command ninja -ErrorAction SilentlyContinue) {
    $testGenerator = @('-G', 'Ninja')
} else {
    $testMake = Join-Path (Split-Path -Parent $testCompiler) 'mingw32-make.exe'
    if (-not (Test-Path -LiteralPath $testMake)) { throw 'Install Ninja or provide -NinjaPath.' }
    $testGenerator = @('-G', 'MinGW Makefiles', "-DCMAKE_MAKE_PROGRAM=$testMake")
}
Push-Location $testFirmwareDir
try {
    foreach ($testConfiguration in @('Debug', 'Release')) {
        $testBuildDir = "build/host-$testConfiguration"
        & cmake -S tests -B $testBuildDir @testGenerator "-DCMAKE_CXX_COMPILER=$testCompiler" "-DCMAKE_BUILD_TYPE=$testConfiguration"
        if ($LASTEXITCODE -ne 0) { throw 'Host test configuration failed' }
        & cmake --build $testBuildDir --parallel 6
        if ($LASTEXITCODE -ne 0) { throw 'Host test build failed' }
        & ctest --test-dir $testBuildDir --output-on-failure
        if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    }
    # This compilation must fail with the intentional VQF diagnostic.
    $testGuardLog = 'build/host-Release/fast-math-rejection.log'
    & $testCompiler -std=c++23 -Ofast -I Component/HardWare/IMU -c Component/HardWare/IMU/vqf.cpp -o build/host-Release/forbidden-fast-math.o *> $testGuardLog
    if ($LASTEXITCODE -eq 0 -or -not (Select-String -Quiet -SimpleMatch 'VQF requires IEEE NaN semantics' $testGuardLog)) {
        throw 'The build did not reject unsafe VQF fast-math options as expected'
    }
    Write-Output 'PASS: Debug/Release host tests and unsafe VQF option rejection'
} finally {
    Pop-Location
}
