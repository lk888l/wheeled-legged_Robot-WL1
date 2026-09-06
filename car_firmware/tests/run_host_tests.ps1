param([string]$Compiler = 'g++')

$ErrorActionPreference = 'Stop'
$startupFirmwareDir = Split-Path -Parent $PSScriptRoot
Push-Location $startupFirmwareDir
try {
    New-Item -ItemType Directory -Force build/startup-tests | Out-Null
    foreach ($startupOptimization in @('-Og', '-O3')) {
        $startupFlags = @('-std=c++23', $startupOptimization, '-fno-fast-math', '-Wall', '-Wextra', '-Werror')
        & $Compiler @startupFlags -I tests/stubs -I Component/UserApp -I Component/HardWare/Motor tests/control_startup_test.cpp Component/HardWare/Motor/TB6612.cpp -o build/startup-tests/control.exe
        if ($LASTEXITCODE -ne 0) { throw 'Control test compilation failed' }
        & ./build/startup-tests/control.exe
        if ($LASTEXITCODE -ne 0) { throw 'Control startup test failed' }

        & $Compiler @startupFlags -I Component/HardWare/IMU tests/imu_startup_test.cpp Component/HardWare/IMU/vqf.cpp -o build/startup-tests/imu.exe
        if ($LASTEXITCODE -ne 0) { throw 'IMU test compilation failed' }
        & ./build/startup-tests/imu.exe
        if ($LASTEXITCODE -ne 0) { throw 'IMU startup test failed' }

        & $Compiler @startupFlags -I Component/UserApp tests/balance_compensation_test.cpp -o build/startup-tests/balance.exe
        if ($LASTEXITCODE -ne 0) { throw 'Balance test compilation failed' }
        & ./build/startup-tests/balance.exe
        if ($LASTEXITCODE -ne 0) { throw 'Balance calibration test failed' }
    }

    # This compilation must fail with the intentional VQF diagnostic.
    & $Compiler -std=c++23 -Ofast -I Component/HardWare/IMU -c Component/HardWare/IMU/vqf.cpp -o build/startup-tests/forbidden-fast-math.o *> build/startup-tests/fast-math-rejection.log
    if ($LASTEXITCODE -eq 0 -or -not (Select-String -Quiet -SimpleMatch 'VQF requires IEEE NaN semantics' build/startup-tests/fast-math-rejection.log)) {
        throw 'The build did not reject unsafe VQF fast-math options as expected'
    }
    Write-Output 'PASS: unsafe VQF compiler options are rejected'
} finally {
    Pop-Location
}
