import cloudComPy as cc
cc.initCC()
cc.initCloudCompare()
from icecream import ic
import numpy as np
import trimesh

from .main_fit_bottom_line import extract_line, extract_hole_line, cal_angle
from .main_rotate_to_xy_plane import rotate_to_xy_plane
from .main_rotateToGetMinBB import get_bb_dim, rotate_and_get_bb_dim
from .main_extractFourCorners import extract_four_points
from .utils import timer


def rotate_cloud(cloud, degree, axis):
    # ic(degree)
    transform1 = cc.ccGLMatrix()
    if axis == 'x':
        transform1.initFromParameters(degree * np.pi/180, (1, 0, 0),(0, 0, 0))
    if axis == 'y':
        transform1.initFromParameters(degree * np.pi/180, (0, 1, 0), (0, 0, 0))
    if axis == 'z':
        transform1.initFromParameters(degree * np.pi/180, (0, 0, 1), (0, 0, 0))
    cloud.applyRigidTransformation(transform1)
    return cloud

def cal_z_range(cloud_np):
    max_z = np.max(cloud_np[:, 2])        
    min_z = np.min(cloud_np[:, 2])   
    ic(min_z)   
    ic(max_z)   
    top_layer = max_z
    prev_points_count  = 0
    while True:
        points_per_layer = cloud_np[np.logical_and(cloud_np[:, 2] >= top_layer - 0.01, cloud_np[:, 2] <= top_layer)]
        ic(top_layer)  
        current_points_count = len(points_per_layer)
        diff = abs(current_points_count - prev_points_count)
        ic(diff / len(cloud_np))
        if diff / len(cloud_np) >= 0.1:
            break
        prev_points_count = current_points_count
        top_layer -= 0.01
    z_start = top_layer-0.04
    z_end = top_layer
    ic(z_start)   
    ic(z_end)
    return z_start, z_end

def cal_y_range(cloud_np):
    max_y = np.max(cloud_np[:, 1])        
    min_y = np.min(cloud_np[:, 1])   
    ic(min_y)
    ic(max_y)   
    top_y = max_y
    stop_iteration = False
    while not stop_iteration:
        points_per_layer = cloud_np[np.logical_and(cloud_np[:, 1] >= top_y - 0.01, cloud_np[:, 1] <= top_y)]
        # ic(len(points_per_layer))
        if (len(points_per_layer))  >= 1000:
            ic(top_y)
            x_coords = points_per_layer[:, 0]
            for coord in x_coords:
                local_points = cloud_np[
                    np.logical_and(
                        np.logical_and(cloud_np[:, 1] >= top_y - 0.01, cloud_np[:, 1] <= top_y),
                        np.logical_and(cloud_np[:, 0] >= coord, cloud_np[:, 0] <= coord + 0.1)
                    )
                ]
                ic(len(local_points))
                if (len(local_points))  >= 300:
                    stop_iteration = True
                    ic(len(local_points))
                    break
                
        # if (len(points_per_layer)/len(cloud_np))*100  >= 0.01:
        #     ic(len(points_per_layer))
        #     break
        y_start = min_y
        y_end = top_y
        top_y -= 0.01
    
    ic(y_start)   
    ic(y_end)
    return y_start, y_end    

def crop_cloud(cloud, start, end, axis):    
    cloud_np = cloud.toNpArray()
    color = cloud.colorsToNpArray()    
    
    cropped_cloud_points = cloud_np[np.logical_and(cloud_np[:, axis] >=start, cloud_np[:, axis] <= end)]
    cropped_color = color[np.logical_and(cloud_np[:, axis] >= start, cloud_np[:, axis] <= end)]
        
    # 创建一个trimesh点云对象
    # cropped_cloud_path = "cropped_cloud_one_color.ply" # 原
    # if axis == 2:
    #     one_color_cloud = trimesh.points.PointCloud(vertices=cropped_cloud_points[:, :3], colors=np.tile([1.0, 0.0, 0.0], (len(cropped_cloud_points), 1)))   # 原
    #     one_color_cloud.export(cropped_cloud_path) # 原
    
    original_color_cloud = trimesh.points.PointCloud(vertices=cropped_cloud_points, colors=cropped_color)

    vertices = original_color_cloud.vertices
    vertices_float32 = vertices.astype(np.float32)
    cropped_cloud = cc.ccPointCloud()
    cropped_cloud.coordsFromNPArray_copy(vertices_float32)
    colors = original_color_cloud.colors
    colors_normalized = colors / 255.0
    colors_uint8 = (colors_normalized * 255).astype(np.uint8)
    cropped_cloud.colorsFromNPArray_copy(colors_uint8)    

    return cropped_cloud

def extract_main_cc(cloud, octreeLevel):
    print("cloud name: %s"%cloud.getName())

    # extract the main cc
    n_clouds, components, residual = cc.ExtractConnectedComponents(clouds=[cloud], octreeLevel=octreeLevel, 
                                        minComponentSize = 350000,
                                        randomColors=False) # minComponentSize = 350000
    print("Number of components: %d"%len(components))
    main_cc = components[0]
    return main_cc

def find_points_in_directions(all_points_np, start, step, axis):
    x_c = np.mean(all_points_np[:, 0])
    y_c = np.mean(all_points_np[:, 1])
    z_c = np.mean(all_points_np[:, 2])
    center_point = (x_c, y_c, z_c)
    points = all_points_np[np.abs(all_points_np[:, axis] - center_point[axis]) <= 0.01]
    len_before = len(points)
    while True:
        points = all_points_np[np.abs(all_points_np[:, axis] - start) <= 0.01]
        len_after = len(points)
        diff = len_after - len_before
        len_before = len_after        
        
        if (diff) >= 30000: # 0.0014
            # ic(diff)
            # ic(len(points))
            return points, start
        start += step  # 增加或减少指定轴的阈值

@timer
def preprocess(cloud):

    # 翻转至正面，底部朝下
    rotated_90_z = rotate_cloud(cloud, 90, 'z')
    rotated_180_y = rotate_cloud(rotated_90_z, 180, 'y') 
    # 先提取最大的连通件
    main_cc = extract_main_cc(rotated_180_y, 9)    
    # 旋转至与xy平面平行
    rotate_to_xy_cloud = rotate_to_xy_plane(main_cc)
    # cc.SavePointCloud(rotate_to_xy_cloud, "rotate_to_xy_cloud.ply")
    # z轴方向切割点云
    z_start, z_end = cal_z_range(rotate_to_xy_cloud.toNpArray())
    cropped_z_cloud = crop_cloud(rotate_to_xy_cloud, z_start, z_end, 2)
    # cc.SavePointCloud(cropped_z_cloud, "cropped_z_cloud.ply")
    
    # 旋转至底部与x轴平行
    background_color = (255, 255, 255)
    rotated_degree_1 = cal_angle(cropped_z_cloud, background_color)
    rotated_cloud_1 = rotate_cloud(cropped_z_cloud, rotated_degree_1, 'z')
    y_start, y_end = cal_y_range(rotated_cloud_1.toNpArray())
    cropped_y_cloud = crop_cloud(rotated_cloud_1, y_start, y_end, 1)
    rotated_degree_2 = cal_angle(cropped_y_cloud, background_color)
    new_cloud = rotate_cloud(cropped_y_cloud, rotated_degree_2, 'z')
    dia_diff, k_diff = extract_line(new_cloud, background_color)

    # cc.SavePointCloud(new_cloud, "new_cloud.ply")
    # extract_four_points(new_cloud)

    # # 旋转原点云以作对比
    # original_cloud_xy = rotate_to_xy_plane(rotated_180_y)
    # original_cloud = rotate_cloud(rotate_to_xy_cloud, (-bottom_to_x), 'z')    
    # cc.SavePointCloud(original_cloud, "original_cloud.ply")

    return  new_cloud, dia_diff, k_diff

if __name__ == "__main__":
    cloud_path = r"C:\workspace\Test\Project_Hebei\ply\test_9J4.ply"
    cloud = cc.loadPointCloud(cloud_path)   

    # 翻转至正面，底部朝下
    rotated_90_z = rotate_cloud(cloud, -90, 'z')
    rotated_180_y = rotate_cloud(rotated_90_z, 180, 'y') 
    # cc.SavePointCloud(rotated_180_y, "rotated_180_y.ply")  
    # 先提取最大的连通件
    main_cc = extract_main_cc(rotated_180_y, 9)    
    # cc.SavePointCloud(main_cc, "main_cc.ply")  
    # 旋转至与xy平面平行
    rotate_to_xy_cloud = rotate_to_xy_plane(main_cc)
    # z轴方向切割点云
    z_start, z_end = cal_z_range(rotate_to_xy_cloud.toNpArray())
    cropped_z_cloud = crop_cloud(rotate_to_xy_cloud, z_start, z_end, 2)
    # cc.SavePointCloud(cropped_z_cloud, "cropped_z_cloud.ply")
    
    # 旋转至底部与x轴平行
    background_color = (255, 255, 255)
    bottom_to_x, left_to_y = extract_line(cropped_z_cloud, background_color)
    rotate_to_x_cloud = rotate_cloud(cropped_z_cloud, (-bottom_to_x), 'z')
    # y轴方向切割点云
    y_start, y_end = cal_y_range(rotate_to_x_cloud.toNpArray())
    new_cloud = crop_cloud(rotate_to_x_cloud, y_start, y_end, 1)

    extract_four_points(new_cloud)
    extract_hole_line(new_cloud, background_color)

    cc.SavePointCloud(new_cloud, "new_cloud_test_9J4calib_concant.ply")
    
    # x, y, z = get_bb_dim(new_cloud.getOwnBB())
    # ic(x, y, z)   

    rotate_and_get_bb_dim(new_cloud, -1, 1, 0.1)

      

    # # 旋转原点云以作对比
    # original_cloud_xy = rotate_to_xy_plane(rotated_180_y)
    # original_cloud = rotate_cloud(original_cloud_xy, (-bottom_to_x), 'z')    
    # cc.SavePointCloud(original_cloud, "original_cloud.ply")

    