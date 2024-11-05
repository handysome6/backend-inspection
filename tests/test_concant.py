import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# from context import backend
from algorithms.calib_concant import combine_frames_extrinsic
from pathlib import Path
import open3d as o3d
from icecream import ic

combine_folder = Path(r"C:\Users\14904\wall_0929_1pm")
cam_ext_pkl = Path(r"Data\model_1012\left_right_ext.pkl")
traj_ext_pkl = Path(r"Data\model_1012\cam_traj_ext.pkl")

pcd_combined = combine_frames_extrinsic(combine_folder, cam_ext_pkl, traj_ext_pkl)

saving_folder = Path(r"C:\workspace\Data")
target_location = saving_folder / f"{combine_folder.name}.ply"
ic(target_location)
o3d.io.write_point_cloud(str(target_location), pcd_combined)
