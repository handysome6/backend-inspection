import numpy as np
from icecream import ic
from typing import List
from pathlib import Path
from loguru import logger
import pickle
import open3d as o3d

from .my_pcd import MyPCD

MAIN_CAM = "RIGHT"


def crop_static_bound(pcd):
    min_bound = np.array([-3.5, -1.5,  2.58 ])
    max_bound = np.array([    2,   7,  2.73  ])
    # ic(min_bound, max_bound)

    cropping_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    cropped_pcd = pcd.crop(cropping_box)

    return cropped_pcd

def resample_pcd(pcd, number_ratio=1/3):
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

        # left_pcd = left_frame.pcd
        # right_pcd = right_frame.pcd

        left_pcd = crop_static_bound(left_frame.pcd)
        right_pcd = crop_static_bound(right_frame.pcd)

        # ic (rt_cam_traj)
        pcd_combined += right_pcd.transform(rt_cam_traj)
        pcd_combined += left_pcd.transform( rt_cam_traj @ rt_lr )

    pcd_combined = resample_pcd(pcd_combined)
    logger.info(f"pcd_combined has {len(pcd_combined.points)} points")

    # 绕z轴旋转-90度
    # R = np.array([[0, 1, 0],
    #               [-1, 0, 0],
    #               [0, 0, 1]])
    # pcd_combined.rotate(R)
    return pcd_combined
        


if __name__ == '__main__':

    combine_folder = Path(r"C:\Users\14904\Data")
    cam_ext_pkl = Path(r"C:\workspace\Data_test\0925_cam_ext\left_right_ext.pkl")
    traj_ext_pkl = Path(r"C:\workspace\Data_test\0925_traj_ext\cam_traj_ext.pkl")

    pcd_combined = combine_frames_extrinsic(combine_folder, cam_ext_pkl, traj_ext_pkl)

    o3d.io.write_point_cloud("combined_mic_0927_03pm.ply", pcd_combined)

    # crop(pcd_combined)
