import cloudComPy as cc
import numpy as np
from icecream import ic

from .utils import timer

cc.initCC()
np.set_printoptions(precision=6, suppress=True)

@timer
def rotate_to_xy_plane(cloud: cc.ccPointCloud) -> cc.ccPointCloud:
    """
    将点云旋转到xy平面
    """
    polygon3 = cc.ccFacet.Create(cloud, maxEdgeLength=0) # 创建平面
    eq3 = polygon3.getPlaneEquation() # 获取平面方程    

    def get_quaternion_from_plane(plane_eq):
        """
        从平面方程计算四元数
        """
        # 提取并归一化法向量
        normal = np.array(plane_eq[:3])
        normal = normal / np.linalg.norm(normal)
        
        # 计算从法向量到z轴(0,0,1)的旋转
        z_axis = np.array([0, 0, 1])
        
        # 计算旋转轴（法向量与z轴的叉积）
        rotation_axis = np.cross(normal, z_axis)  # 注意：这里交换了叉积的顺序
        
        # 如果旋转轴接近零向量，说明法向量与z轴平行
        if np.allclose(rotation_axis, 0):
            if normal[2] > 0:  # 如果法向量已经指向z轴正方向
                return np.array([1, 0, 0, 0])  # 单位四元数，不需要旋转
            else:  # 如果法向量指向z轴负方向
                return np.array([0, 1, 0, 0])  # 绕x轴旋转180度
        
        # 归一化旋转轴
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        
        # 计算旋转角度（法向量与z轴的夹角）
        cos_theta = np.dot(normal, z_axis)
        theta = np.arccos(cos_theta)
        
        # 构建四元数 [w, x, y, z]
        w = np.cos(theta / 2)
        xyz = rotation_axis * np.sin(theta / 2)
        quaternion = np.array([w, xyz[0], xyz[1], xyz[2]])
        
        return quaternion

    quaternion = get_quaternion_from_plane(eq3) # 计算四元数
    tr = cc.ccGLMatrix.FromQuaternionAndTranslation(quaternion) # 创建变换矩阵
    cloud.applyRigidTransformation(tr) # 应用变换
    return cloud
   

if __name__ == "__main__":
    cloud = cc.loadPointCloud("rotated_180_y.ply")
    cloud = rotate_to_xy_plane(cloud)
    
    cal_z_range(cloud)
    # ic(extracted_points)
    # cal_z_range(extracted_points)
    # cc.SavePointCloud(extracted_cloud, "new.ply")
    # cc.SaveEntities([cloud], "new.ply")