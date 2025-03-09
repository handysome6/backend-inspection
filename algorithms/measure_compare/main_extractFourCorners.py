import cloudComPy as cc
from icecream import ic
import numpy as np
cc.initCC()

def extract_main_cc(pcd_path):
    # load the pcd
    cloud = cc.loadPointCloud(pcd_path)
    print("cloud name: %s"%cloud.getName())

    # extract the main0102 cc
    n_clouds, components, residual = cc.ExtractConnectedComponents(clouds=[cloud], octreeLevel=10, 
                                        minComponentSize = 350000,
                                        randomColors=True) # minComponentSize = 350000
    print("Number of components: %d"%len(components))
    main_cc = components[0]

    # Save the main0102 cc
    cc.SavePointCloud(main_cc, "main_cc.ply")
    return main_cc

def extract_four_points(cloud):
    
    cloud_np = cloud.toNpArray()

    bottom_left_idx = np.argmin(cloud_np[:, 0] + cloud_np[:, 1])
    bottom_left = cloud_np[bottom_left_idx]  
    ic("左下角点坐标:", bottom_left)

    bottom_right_idx = np.argmin(-cloud_np[:, 0] + cloud_np[:, 1])
    bottom_right = cloud_np[bottom_right_idx]
    ic("右下角点坐标:", bottom_right)

    # 找到左下角点 (最小x和最大y)
    top_left_idx = np.argmin(cloud_np[:, 0] - cloud_np[:, 1])
    top_left = cloud_np[top_left_idx]
    ic("左上角点坐标:", top_left)

    # 找到右下角点 (最大x和最大y)
    top_right_idx = np.argmax(cloud_np[:, 0] + cloud_np[:, 1])
    top_right = cloud_np[top_right_idx]
    ic("右上角点坐标:", top_right)

    top_len = np.linalg.norm(np.array(top_left) - np.array(top_right))*1000
    bottom_len = np.linalg.norm(np.array(bottom_left) - np.array(bottom_right))*1000
    left_len = np.linalg.norm(np.array(top_left) - np.array(bottom_left))*1000
    right_len = np.linalg.norm(np.array(top_right) - np.array(bottom_right))*1000
    ic(top_len, bottom_len, left_len, right_len)


if __name__ == "__main__":
    main_cc = extract_main_cc("main_cc.ply")
    extract_four_points("min_bb_y.ply")