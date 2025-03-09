"""
This file contains basic configuration options for the application to work normally.
Please change according to the systems' file sturcture.
"""


############################ Simulation configuration ############################
# Control whether PLC not to wait for wall in position signal.
PLC_WAIT_FOR_WALL = True
# Control whether to connect to PLC and camera.
RUN_SIMULATION = False
SIMULATION_DATA_DIR = r"D:\Inspection_data\20250309001"
USE_FAKE_DATA = False
GET_MODEL_FROM_PLC = False


############################ Extrinsics configuration ############################
# extrinsic between left and right camera AND extrinsic between each of two right capture position. 
CAM_EXT_PKL = r"Data\model_0308\left_right_ext.pkl"
TRAJ_EXT_PKL = r"Data\model_0308\cam_traj_ext.pkl"


############################ Capture Saving Options ############################
# root folder for saving captured data and database files.
# ROOT_FOLDER = r"U:\Inspection_Data"
ROOT_FOLDER = r"D:\Inspection_Data"

############################ PLC Options ############################
PLC_HOST = '192.168.111.3'
PLC_PORT = 502
