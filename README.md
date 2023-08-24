# Clone of [ScenarioRunner for CARLA](https://github.com/carla-simulator/scenario_runner) adapted for own research needs.

The ogirinal ScenarioRunner was modified a bit:
- fixed the issue with wrong parameter
- `manual_contorl.py`:
    - the front camera moved into the cabin,
    - initial camera may be specified as a script argument as `--camera 0..4`
- `scenario_runner.py`:
    - the script takes an optional arguments `--params <str>` that is passed to the scenario script as
    ```python
    class MyScenario(BasicScenario):
        def __init__(self, world, ego_vehicles, config,
                    randomize=False,
                    debug_mode=False,
                    criteria_enable=True,
                    timeout=600,
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
- After the project is cloned, specify correct pathes in `.set-env.ps1` before running for the first time!
- Virtual environment is created and all Python dependencies are installed into this environment once a scenario is launched for the first time.

## Scenarios

Scenarios are located in `scenarios` folder.
To start, simply run CARLA and then run a scenario.

**Note**:
You may need to allow PowerShell running scripts. If so, open PowerShell and run the following command:
``` ps
Set-ExecutionPolicy Unrestricted
```

### List of scenarios:

- ChangeLane:  
    Comes with 3 tasks:
    1. All cars move in autonomous mode,
    2. Ego car moves in authonomous mode. There a Tesla car moving a bit ahead and on another lane. As the ego car approaches, Tesla changes the lane, but this happens at a safe distance in front of the ego car. 
    3. Same as task #2, but Tesla changes the lane at an unsafe distance in front of the ego car, so the driver should slow down to avoid collision.
    
    **Note**: The original ChangeLane scenario was renamed into ChangeLaneOriginal