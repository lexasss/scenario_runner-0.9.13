if (-Not (Test-Path -Path "./.venv")) {
  "Creating the environment . . ."
  python -m venv .\.venv
  
  ""
  .\.venv\Scripts\activate
  
  ""
  "Upgrading PIP . . ."
  python -m pip install --upgrade pip

  ""
  "Installing dependencies. . ."
  pip install -r requirements.txt

  ""
  "Activating environment"

  ""
  "Starting the script"

  ""
}
else {
  .\.venv\Scripts\activate
}
