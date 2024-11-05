### Inspection Project for CSCEC - Backend

#### This project contains:
 - backend server
 - project manager
 - hardware manager
 - algorithms

#### How to use
1. change the config.py file to set the parameters for the inspection
2. run the main.py to start the server

#### Configuration via config.py
`PLC_WAIT_FOR_WALL` : Control whether PLC not to wait for wall in position signal.
`RUN_SIMULATION` : Control whether to connect to PLC and camera. False for real inspection.
`SIMULATION_DATA_DIR` : The folder path for simulation data.
`USE_FAKE_DATA` : Control whether to use fake data for demo purpose.

#### Deployment
1. Install the python packages in the requirements.txt
2. Git clone frontend project and `npm run dev`
3. change the `.bat` file to set the environment variables for python and node.js
4. create the shortcut for `python run.py` on the desktop
