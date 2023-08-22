# This is the script to run ChangeLane procedure defined in the
#    srunner\scenarios\change_lane.py and
#    srunner\examples\ChangeLane.xml files
#
# Simply start CARLA and run this script
#

cd ..

. ".set-env.ps1"
. ".enter-env.ps1"

Start-Process -FilePath "powershell" -ArgumentList "& .\.driving-ui.ps1"
python scenario_runner.py --scenario ChangeLane_M --reloadWorld --waitForEgo

""
"Done"

cd scenarios