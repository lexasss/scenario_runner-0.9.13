$carla = 'C:\Users\olequ\Downloads\CARLA 0.9.13'
$srunner = 'C:\Users\olequ\Downloads\CARLA 0.9.13\scenario_runner-0.9.13'

if (-Not (Test-Path -Path $carla)) {
    "CARLA's root folder specified in `.enter-env.ps1` is not valid."
    exit
}

if (-Not (Test-Path -Path $srunner)) {
    "SRunner's root folder specified in `.enter-env.ps1` is not valid."
    exit
}

$env:CARLA_ROOT = $carla
$env:SCENARIO_RUNNER_ROOT = $srunner
$env:PYTHONPATH = ";$env:CARLA_ROOT\PythonAPI\carla;$env:CARLA_ROOT\PythonAPI\carla\dist\carla-0.9.13-py3.7-win-amd64.egg"
