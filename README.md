# Clone of [ScenarioRunner for CARLA](https://github.com/carla-simulator/scenario_runner) adapted for own research needs.

Requires
- [CARLA 0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13)
- Python 3.7.9

**Note**:
- Specify correct pathes  `.enter-env.ps1` before running for the first time!
- All Python dependencies are installed into a virtual environment once a scenario is launched for the first time.

## Scenarios

Scenarios are located in `scenarios` folder.
To start, simply run CARLA and then run a scenario.

List of scanarios:

- ChangeLane:
    ego car moves in authonomous mode. There a Tesla car moving a bit ahead and on another lane. Suddenly, Tesla changes the lane passing by just in front of the ego car. 
