# This is the script to run ChangeLane scanario defined in the
#    srunner\tasks\change_lane.py and
#    srunner\examples\ChangeLane.xml files
#
# Simply start CARLA and run this script

function Get-Task
{
    [string[]]$tasks = 'Just drive', 'Safe lane change', 'Unsafe lane change'

    Write-Host "Available tasks:"
    Write-Host ""

    For ($i=0; $i -lt $tasks.Count; $i++)
    {
        Write-Host "$($i+1): $($tasks[$i])"
    }
    
    Write-Host ""

    [ValidateScript({$_ -ge 0 -and $_ -le $tasks.Length})]
    [int]$number = Read-Host "Press the number to select a task"
    
    Write-Host ""

    if($? -and $number -gt 0) {
        Write-Host "Starting the task '$($tasks[$number - 1])'"
        return $number
    }
    else {
        Write-Host "No task selected. Bye!"
    }

    return 0
}


Set-Location ..

. ".set-env.ps1"
. ".enter-env.ps1"

$task = Get-Task
if ($task -gt 0)
{
    Start-Process -FilePath "powershell" -ArgumentList "& .\.driving-ui.ps1"
    python scenario_runner.py --scenario ChangeLane_Linkoln --reloadWorld --params=$task

    ""
    "Done"
}

Set-Location scenarios
