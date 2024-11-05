import numpy as np
from icecream import ic
from pathlib import Path
from loguru import logger
import open3d as o3d

def crop_static_bound(pcd):
    min_bound = np.array([-3.5, -1.5,  2.58 ])
    max_bound = np.array([    2,   7,  2.73  ])
    ic(min_bound, max_bound)

    cropping_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    cropped_pcd = pcd.crop(cropping_box)

    return cropped_pcd

def crop_using_current_bounding(pcd, z_cropping):
    max_bound = pcd.get_max_bound()
    min_bound = pcd.get_min_bound()
    ic(min_bound, max_bound)

    x_min, y_min, z_min = min_bound
    x_max, y_max, _ = max_bound
    z_max = z_min + z_cropping # 向下截取

    min_bound = np.array([x_min, y_min, z_min])
    max_bound = np.array([x_max, y_max, z_max])
    ic(min_bound, max_bound)
    cropping_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    cropped_pcd = pcd.crop(cropping_box)
    return cropped_pcd

def rotation_matrix_from_vectors(vec1, vec2):
    # Modified from https://segmentfault.com/q/1010000043308109
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def projet_by_fitting_palne(pcd):
    # 检测平面
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    R_plane = rotation_matrix_from_vectors(plane_model[:3], np.array([0,0,1]))

    # 提取平面点云
    # plane_cloud = pcd.select_by_index(inliers)
    # ic(plane_cloud.get_center())
    
    T_plane = np.eye(4)
    T_plane[:3, :3] = R_plane
    ic(T_plane)
    orthogonal_plane_pcd = pcd.transform(T_plane)

    return orthogonal_plane_pcd

def get_top_surface(pcd):
    """
    """
    ortho_pcd = projet_by_fitting_palne(pcd)
    # o3d.io.write_point_cloud("ortho_mic.ply", ortho_pcd)

    croppped_top = crop_using_current_bounding(ortho_pcd, 0.05)
    # o3d.io.write_point_cloud("croppped_top.ply", croppped_top)
    return croppped_top


if __name__ == "__main__":
    pcd_path = Path(r"C:\Inspection_Data\20241012003\pcd_combined.ply")
    print("Load a ply point cloud")
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    pcd_surface = get_top_surface(pcd)