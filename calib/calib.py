import numpy as np
import cv2
from icecream import ic
from loguru import logger
from PIL import Image
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from algorithms.utils import filter_avg
from algorithms.CCTDecoder.cct_decode import CCT_extract
from algorithms.my_pcd import MyPCD


CCT_N = 12
MANUAL_CONSTRAINT_DISTANCE = 0.640


def get_cct_world_coord(pcd1: MyPCD, pcd2: MyPCD):
        """
        Get common cct ids and world coordinates of two frames
        """
        frame0 = pcd1
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

        return common_ids, world_points0, world_points1


def optimize_R_T(world_points0, world_points1):
        """
        Optimize the transformation matrix between two frames using given world coordinates
        """
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

        x0 = np.zeros(6)
        res = minimize(loss, x0, method='Nelder-Mead')
        # print final error
        logger.info(f"Final error: {loss(res.x)}")
        rot = R.from_euler('xyz', res.x[:3]).as_matrix()
        T = res.x[3:] + u1 - rot @ u0
        transmat01 = np.eye(4)
        transmat01[:3, :3] = rot
        transmat01[:3, 3] = T
        return transmat01


def optimize_R_T_with_constraint(world_points0, world_points1):
        """
        Optimize the transformation matrix between two frames using given world coordinates
        """
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
        
        def constraint(x):
            return np.linalg.norm(x[3:]) - MANUAL_CONSTRAINT_DISTANCE

        x0 = np.zeros(6)
        res = minimize(loss, x0, constraints={'type': 'eq', 'fun': constraint})
        # print final error
        logger.info(f"Final error: {loss(res.x)}")
        rot = R.from_euler('xyz', res.x[:3]).as_matrix()
        T = res.x[3:] + u1 - rot @ u0
        transmat01 = np.eye(4)
        transmat01[:3, :3] = rot
        transmat01[:3, 3] = T
        return transmat01


def optimize_T_only_with_constraint(world_points0, world_points1):
        """
        Optimize the translation vector between two frames using given world coordinates
        """
        # normalize 3d points
        u0 = np.mean(np.array(world_points0), axis=0)
        u1 = np.mean(np.array(world_points1), axis=0)
        world_points0 = world_points0 - u0
        world_points1 = world_points1 - u1

        # scipy optimize
        def loss(x):
            T = x
            error = 0
            for i, j in zip(world_points0, world_points1):
                error += np.linalg.norm(i + T - j)
            return error
        
        def constraint(x):
            return np.linalg.norm(x) - MANUAL_CONSTRAINT_DISTANCE

        x0 = np.zeros(3)
        res = minimize(loss, x0, constraints={'type': 'eq', 'fun': constraint})
        # print final error
        logger.info(f"Final error: {loss(res.x)}")
        T = res.x + u1 - u0
        transmat01 = np.eye(4)
        transmat01[:3, 3] = T
        return transmat01


def calib_two_frames(pcd1: MyPCD, pcd2: MyPCD):
    common_ids, world_points0, world_points1 = get_cct_world_coord(pcd1, pcd2)
    transmat01 = optimize_R_T_with_constraint(world_points0, world_points1)
    return transmat01
