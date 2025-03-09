import numpy as np
from icecream import ic
from typing import List
from pathlib import Path
from loguru import logger
import pickle
import open3d as o3d

from .my_pcd import MyPCD

TRANS1 = np.array([
    [0.992081165314, -0.000635988603, 0.125596761703, -0.332078039646],
    [-0.000635988603, 0.999948918819, 0.010087110102, -0.026670455933],
    [-0.125596761703, -0.010087110102, 0.992030084133, -0.037964105606],
    [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
])

TRANS2 = np.array([
    [0.999994099140, 0.000002645179, -0.003429675940, 0.008911013603],
    [0.000002645179, 0.999998807907, 0.001542518614, -0.004007816315],
    [0.003429675940, -0.001542518614, 0.999992907047, -0.001772880554],
    [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
])

TRANS3 = np.array([
    [0.992054939270, -0.000522068643, 0.125804364681, -0.331498384476],
    [-0.000522068643, 0.999965667725, 0.008266577497, -0.021782636642],
    [-0.125804364681, -0.008266577497, 0.992020606995, -0.051970720291],
    [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
])


def crop_static_bound(pcd):
    min_bound = np.array([-3.5, -1.5,  2.55 ])
    max_bound = np.array([    2,   7,  2.73  ])
    # ic(min_bound, max_bound)

    cropping_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    cropped_pcd = pcd.crop(cropping_box)

    return cropped_pcd

def resample_pcd(pcd, number_ratio=1/2):
    logger.info(f"resampling pcd with ratio {number_ratio}")
    pcd_down = pcd.random_down_sample(number_ratio)
    return pcd_down

def combine_frames_extrinsic(combine_folder, cam_ext_pkl, traj_ext_pkl):
    cam_ext = pickle.load(open(cam_ext_pkl, "rb"))
    traj_ext = pickle.load(open(traj_ext_pkl, "rb"))
    logger.info(f"loaded cam_ext and traj_ext from {cam_ext_pkl} and {traj_ext_pkl}")

    # glob all the folders
    frames_folder = [f for f in sorted(combine_folder.iterdir()) if f.is_dir()]
    ic(frames_folder)

    # Concatenate the point clouds
    pcd_combined = o3d.geometry.PointCloud()

    for frame_path in frames_folder:
        frame_path: Path
        idx = frame_path.name
        logger.info(f"Combining {idx} frame ...")
        rt_cam_traj= traj_ext[idx]
        rt_lr = cam_ext

        left_frame = MyPCD(frame_path / "left")
        right_frame = MyPCD(frame_path / "right")

        left_pcd = left_frame.pcd
        right_pcd = right_frame.pcd

        pcd_combined += left_pcd.transform(rt_cam_traj)
        pcd_combined += right_pcd.transform( rt_cam_traj @ rt_lr )

    # transform pcd_combined by TRANS2@TRANS1
    pcd_combined.transform(TRANS2@TRANS1)
    # rotate pcd_combined around z-axis by 180 degrees
    rt_180 = np.array([
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    pcd_combined.transform(rt_180)

    # crop static bound
    pcd_combined = crop_static_bound(pcd_combined)


    pcd_combined = resample_pcd(pcd_combined)
    logger.info(f"pcd_combined has {len(pcd_combined.points)} points")

    return pcd_combined



if __name__ == '__main__':

    combine_folder = Path(r"C:\Users\14904\Data")
    cam_ext_pkl = Path(r"C:\workspace\Data_test\0925_cam_ext\left_right_ext.pkl")
    traj_ext_pkl = Path(r"C:\workspace\Data_test\0925_traj_ext\cam_traj_ext.pkl")

    pcd_combined = combine_frames_extrinsic(combine_folder, cam_ext_pkl, traj_ext_pkl)

    o3d.io.write_point_cloud("combined_mic_0927_03pm.ply", pcd_combined)

    # crop(pcd_combined)
