# Clone of [ScenarioRunner for CARLA](https://github.com/carla-simulator/scenario_runner) adapted for own research needs.

The original `ScenarioRunner` was modified a bit:
- fixed the issue with wrong parameter for some function,
- `manual_contorl.py`:
    - the front camera moved into the cabin,
    - initial camera may be specified as a script argument with `--camera 0..4`
- `scenario_runner.py`:
    - the script takes an optional argument `--params <str>` that is passed to the scenario script:
    ```python
    class MyScenario(BasicScenario):
        def __init__(self, world, # skipped other parameters 
                    params=''):
            ...
    ```

## Requirements
- [CARLA 0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13)
- [Python 3.7.9](https://www.python.org/ftp/python/3.7.9/python-3.7.9-amd64.exe)
- [Git](https://git-scm.com/downloads)

## Downloading the project

```
git clone https://github.com/lexasss/scenario_runner-0.9.13.git
```

**Note**:
- After cloning the project, please specify correct pathes in `.set-env.ps1`,
- Virtual environment is created and all Python dependencies are installed into this environment once any scenario is launched for the first time.

## Scenarios

Scenarios are located in `scenarios` folder.
To start, simply run CARLA and then run a scenario in PowerShell.

**Note**:
You may need to allow PowerShell running scripts. If so, open PowerShell and run the following command:
``` ps
Set-ExecutionPolicy Unrestricted
```

**Tip**:
PowerShell scripts can be launched by mouse double-click if the default value of `HKEY_CLASSES_ROOT\Microsoft.PowerShellScript.1\Shell\Open\Command` in regedit is set to `powershell.exe -noLogo -file "%1"`. No that setting this value to `powershell.exe -noLogo -ExecutionPolicy unrestricted -file "%1"` removes the need to change the PowerShell policy as suggested in `Note` above.

### List of scenarios:

- ChangeLane:  
    Comes with 3 tasks:
    1. All cars move in autonomous mode,
    2. Ego car moves in authonomous mode. There a Tesla car moving a bit ahead and on another lane. As the ego car approaches, Tesla changes the lane, but this happens at a safe distance in front of the ego car. 
    3. Same as task #2, but Tesla changes the lane at an unsafe distance in front of the ego car, so the driver should slow down to avoid collision.
    
    **Note**: The original ChangeLane scenario was renamed into ChangeLaneOriginal