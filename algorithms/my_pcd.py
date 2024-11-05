import open3d as o3d
import numpy as np
import copy
import cv2
from pathlib import Path
from icecream import ic
from typing_extensions import Self
from loguru import logger
from PIL import Image
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from .utils import bilinear_interpolation, timeit, filter_avg
from .CCTDecoder.cct_decode import CCT_extract


CCT_N = 12


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.2459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

def draw_registration_result(source, target, transformation, color=False):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    if not color:
        source_temp.paint_uniform_color([1, 1, 0])
        target_temp.paint_uniform_color([0, 1, 1])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

@timeit
def colored_icp_registration(source, target, voxel_size, current_transformation):
    print("Colored ICP registration")
    voxel_radius = [5*voxel_size, 3*voxel_size, voxel_size]
    max_iter = [60, 35, 20]
    for scale in range(3):
        max_it = max_iter[scale]
        radius = voxel_radius[scale]
        print("scale_level = {0}, voxel_size = {1}, max_iter = {2}".format(scale, radius, max_it))
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=20))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=20))
        result = o3d.pipelines.registration.registration_icp(
            source_down, 
            target_down, 
            radius, 
            current_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=max_it))
        current_transformation = result.transformation
        print(result)
    current_transformation = np.array(current_transformation)
    print(current_transformation)
    # draw_registration_result(source, target, current_transformation, color=False)
    return current_transformation


class MyPCD():

    def __init__(self, folder_path):
        self.folder_path = Path(folder_path)
        logger.info(f"Loading pointcloud from {self.folder_path}...")
        pcd_path = self.folder_path / 'PointCloud.ply'
        self.pcd = o3d.io.read_point_cloud(str(pcd_path))

        self.image = cv2.imread(str(self.folder_path / 'Image.png'))
        if self.image is None:
            logger.error(f"Failed to load pointcloud folder, please check path.")
        self.h, self.w = self.image.shape[:2]

    def _get_3d_point_int(self, x: int, y: int):
        l = list(self.pcd.points[y * self.w + x])
        return np.array(l)

    def get_3dcoord_bilinear(self, x:float, y:float):
        x0, y0 = int(x), int(y)
        x1, y1 = x0 + 1, y0 + 1

        q00 = self._get_3d_point_int(x0, y0)
        q01 = self._get_3d_point_int(x0, y1)
        q10 = self._get_3d_point_int(x1, y0)
        q11 = self._get_3d_point_int(x1, y1)

        d3_coord = bilinear_interpolation(x, y, x0, y0, x1, y1, q00, q01, q10, q11)
        if np.isnan(d3_coord).any():
            return None
        return d3_coord
    
    def refine_RT_icp(self, pcd2: Self, T_mat_opencv):
        # source = self.remove_plane()
        # target = pcd2.remove_plane()
        source = self.pcd
        target = pcd2.pcd
        threshold = 2
        trans_init = T_mat_opencv

        # downsample the point cloud
        # source = source.voxel_down_sample(voxel_size=0.001)
        # target = target.voxel_down_sample(voxel_size=0.001)

        print("Initial alignment")
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, threshold, trans_init)
        print(evaluation)
        print(trans_init)
        print()
        # visualize the registration result
        # draw_registration_result(source, target, trans_init)


        # print("Recompute the normal of the downsampled point cloud")
        # source.estimate_normals(
        #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # target.estimate_normals(
        #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        print("Apply point-to-point ICP")
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint())
            # criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        print()
        print()
        # visualize the registration result
        # draw_registration_result(source, target, reg_p2p.transformation)
        return reg_p2p.transformation

    # use CCT decoder 
    def estimate_RT_CCT_optimize(self, pcd2: Self):
        """
        Estimate the transformation matrix between two frames using CCT markers
        R, T optimization using scipy minimize
        """
        frame0 = self
        frame1 = pcd2
        img0 = frame0.image
        img1 = frame1.image

        # extract CCT id and coords
        cct0, _ = CCT_extract(img0, CCT_N)
        cct1, _ = CCT_extract(img1, CCT_N)

        ic(cct0.keys())
        ic(cct1.keys())

        # get common cct ids
        common_ids = set(cct0.keys()).intersection(set(cct1.keys()))
        # ic(len(common_ids), common_ids)

        src_pts = [cct0[id] for id in common_ids]
        dst_pts = [cct1[id] for id in common_ids]
        src_pts, dst_pts = filter_avg(src_pts, dst_pts)

        if __name__ == "__main__":
            # show matches in image
            concant_img = np.concatenate((img0, img1), axis=0)
            # draw matches on the concant image, using line and circle
            for src, dst in zip(src_pts, dst_pts):
                x0, y0 = src
                x1, y1 = dst
                cv2.circle(concant_img, np.int16([x0, y0]), 5, (0, 0, 255), -1)
                cv2.circle(concant_img, np.int16([x1, y1+frame0.h]), 5, (0, 0, 255), -1)
                cv2.line(concant_img, np.int16([x0, y0]), np.int16([x1, y1+frame0.h]), (0, 255, 0), 1)
                # cv2.line(concant_img, np.int16([x0, y0]), np.int16([x1+frame0.w, y1]), (0, 255, 0), 1)
            # show using pil
            Image.fromarray(concant_img).show()

        # get 3d points from cct ids
        world_points0 = []
        world_points1 = []
        for src, dst in zip(src_pts, dst_pts):
            x0, y0 = src
            x1, y1 = dst
            d3_coord0 = frame0.get_3dcoord_bilinear(*src)
            d3_coord1 = frame1.get_3dcoord_bilinear(*dst)
            if d3_coord0 is not None and d3_coord1 is not None:
                world_points0.append(d3_coord0)
                world_points1.append(d3_coord1)
            else:
                if d3_coord0 is None:
                    print(f"Failed to get 3d coord for id {id} in {frame0.folder_path}")
                if d3_coord1 is None:
                    print(f"Failed to get 3d coord for id {id} in {frame1.folder_path}")

        # normalize 3d points
        u0 = np.mean(np.array(world_points0), axis=0)
        u1 = np.mean(np.array(world_points1), axis=0)
        world_points0 = world_points0 - u0
        world_points1 = world_points1 - u1

        # scipy optimize
        def loss(x):
            rot = R.from_euler('xyz', x[:3]).as_matrix()
            T = x[3:]
            error = 0
            for i, j in zip(world_points0, world_points1):
                error += np.linalg.norm(rot @ i + T - j)
            return error

        # initial guesses
        # r =  R.from_matrix(rotMat)
        # rot_eular = r.as_euler("xyz",degrees=False)
        # x0 = np.concatenate((rot_eular, T_mat))
        x0 = np.zeros(6)
        # x0 = np.random.rand(6)
        # x0 = np.array([1, 1, 1, 0, 0, 0])
        res = minimize(loss, x0, method='Nelder-Mead')
        # print(res)
        # print final error
        logger.info(f"Final error: {loss(res.x)}")
        rot = R.from_euler('xyz', res.x[:3]).as_matrix()
        T = res.x[3:] + u1 - rot @ u0
        transmat01 = np.eye(4)
        transmat01[:3, :3] = rot
        transmat01[:3, 3] = T
        return transmat01

    def check_cct_similarity(self, pcd2: Self):
        frame0 = self
        frame1 = pcd2
        img0 = frame0.image
        img1 = frame1.image

        # extract CCT id and coords
        cct0, _ = CCT_extract(img0, CCT_N)
        cct1, _ = CCT_extract(img1, CCT_N)

        # get common cct ids
        common_ids = set(cct0.keys()).intersection(set(cct1.keys()))
        # ic(len(common_ids), common_ids)

        common_ids = sorted(common_ids)

        src_pts = [cct0[id] for id in common_ids]
        dst_pts = [cct1[id] for id in common_ids]
        src_pts, dst_pts = filter_avg(src_pts, dst_pts)

        # pixel error
        errors = src_pts - dst_pts
        err_distance = np.linalg.norm(errors, axis=1)
        ic(err_distance)
        mean = np.mean(err_distance)
        ic(mean)

        # d3 coord error
        # get 3d points from cct ids
        world_points0 = []
        world_points1 = []
        for src, dst in zip(src_pts, dst_pts):
            x0, y0 = src
            x1, y1 = dst
            d3_coord0 = frame0.get_3dcoord_bilinear(*src)
            d3_coord1 = frame1.get_3dcoord_bilinear(*dst)
            if d3_coord0 is not None and d3_coord1 is not None:
                world_points0.append(d3_coord0)
                world_points1.append(d3_coord1)
            else:
                if d3_coord0 is None:
                    print(f"Failed to get 3d coord for id {id} in {frame0.folder_path}")
                if d3_coord1 is None:
                    print(f"Failed to get 3d coord for id {id} in {frame1.folder_path}")

        errors = np.array(world_points0) - np.array(world_points1)
        err_distance = np.linalg.norm(errors, axis=1)

        mean = np.mean(err_distance)
        std = np.std(err_distance)
        upper_bound = mean + 1 * std
        lower_bound = mean - 1 * std

        x = np.argwhere((err_distance>lower_bound) & (err_distance<upper_bound))    # shape: (n, 1, 2)
        x = x.flatten()     # shape: (n, 2)

        err_distance = err_distance[x]
        mean = np.mean(err_distance)
        ic(err_distance)
        ic(mean)

if __name__ == "__main__":
    pcd1 = MyPCD(r"C:\workspace\Data_test\Data_0925\7")
    pcd0 = MyPCD(r"C:\workspace\Data_test\Data_0925\8")

    # rt = pcd0.estimate_RT_CCT_optimize(pcd1)
    # print(rt)

    pcd0.check_cct_similarity(pcd1)

    # # visual
    # draw_registration_result(pcd0.pcd, pcd1.pcd, rt)

    # # Concatenate the point clouds
    # pcd_combined = o3d.geometry.PointCloud()
    # pcd_combined += pcd0.pcd.transform(rt)
    # pcd_combined += pcd1.pcd
    # o3d.io.write_point_cloud("combined_pointcloud.ply", pcd_combined)
