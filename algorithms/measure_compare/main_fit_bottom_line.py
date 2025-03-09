"""
This is for extracting bottom points from a surface pointcloud.
Please install the following dependencies before use:
    pip install scikit-image icecream matplotlib numpy

Please also install cloudComPy binaries.
"""

import numpy as np
import tkinter as tk
import cloudComPy as cc
from icecream import ic
from PIL import Image, ImageTk
from skimage.measure import LineModelND, ransac

cc.initCC()
# set np print options
np.set_printoptions(precision=4, suppress=True)

def show_image_in_popup(img):
    root = tk.Tk()
    root.title("Image Popup")

    # 将 NumPy 数组转换为 PIL 图像
    img_pil = Image.fromarray(img)

    # 将 PIL 图像转换为 Tkinter PhotoImage
    img_tk = ImageTk.PhotoImage(img_pil)

    # 创建标签并在窗口中显示图像
    label = tk.Label(root, image=img_tk)
    label.pack()

    root.mainloop()

# 转换点云到2d图像（CloudCompare version） background_color=(48, 40, 33)
def convert_pcd_to_2d_image(pcd:cc.ccPointCloud, background_color=(48, 40, 33)) -> tuple[np.ndarray, np.ndarray]:
    """
    convert open3d point cloud to 2d image
    ---------------------------------------------------------------------------
    parameters:
    ----------
        pcd: open3d point cloud
        background_color: background color of the image

    returns:
    --------
        2d image and transform matrix
    """
    print("convert_pcd_to_2d_image")
    # get all points change z-coord of points to 1
    points = np.asarray(pcd.toNpArray())
    print("points shape: ", points.shape)
    points[:, 2] = 1

    # [0, 1] to uint8 [0, 255]
    colors = (np.asarray(pcd.colorsToNpArray())).astype(np.uint8)
    print("colors shape: ", colors.shape)
    colors = colors[:,:3]

    bb = pcd.getOwnBB()
    x_min, y_min, _ = bb.minCorner()
    x_max, y_max, _ = bb.maxCorner()
    x_range = x_max - x_min
    y_range = y_max - y_min

    img_x_range = 1200
    img_y_range = int(y_range * img_x_range / x_range)

    img_width = img_x_range + 300
    img_height = img_y_range + 300

    # default background is BGR(48, 40, 33)
    # current background is white(255, 255, 255)
    img = np.full((img_height, img_width, 3), background_color, dtype=np.uint8)
    # img = np.zeros((img_y_range+300, img_x_range+300, 3))

    # 计算缩放因子
    scale_x = img_x_range / x_range
    scale_y = img_y_range / y_range

    # 创建变换矩阵
    transform_matrix = np.array([
        [scale_x,      0,        0,  -x_min * scale_x + 150],
        [0,            scale_y,  0,  -y_min * scale_y + 150],
        [0,            0,        1,  0                     ],
        [0,            0,        0,  1                     ]
    ])

    # 将所有点转换为齐次坐标
    homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))

    # 使用矩阵乘法一次性转换所有点
    transformed_points = np.dot(homogeneous_points, transform_matrix.T)

    # 提取转换后的x和y坐标，并转换为整数类型
    x_pixels = transformed_points[:, 0].astype(int)
    y_pixels = transformed_points[:, 1].astype(int)

    # 创建有效点的掩码
    valid_mask = (x_pixels >= 0) & (x_pixels < img_width) & (y_pixels >= 0) & (y_pixels < img_height)
    # 使用掩码筛选有效点
    valid_x = x_pixels[valid_mask]
    valid_y = y_pixels[valid_mask]
    valid_colors = colors[valid_mask]

    # 更新图像
    img[valid_y, valid_x] = valid_colors

    # 打印一些统计信息
    print(f"Convertion finished, 总点数: {len(points)}")
    print(f"有效点数: {np.sum(valid_mask)}")
    print(f"无效点数: {len(points) - np.sum(valid_mask)}")
    # show_image_in_popup(img)
    return img, transform_matrix

def extract_hole_horizontal_points(points, img, background_color, w_end, w_dir, h_end, h_dir, neighbor):
    height, width = img.shape[:2]
    center_x = width // 2
    center_y = height // 2    

    end_point = False
    for i in range(center_x, w_end, w_dir):
        for j in range(center_y, h_end, h_dir):
            if tuple(img[j, i]) != background_color:
                points.append((j, i))
                if tuple(img[j + neighbor, i]) != background_color:
                    end_point = True
                break
        if end_point:
            break

    print(f"hole points number: {len(points)}")
    return points

def extract_hole_vertical_points(points, img, background_color, w_end, w_dir, h_end, h_dir, neighbor):
    height, width = img.shape[:2]
    center_x = width // 2
    center_y = height // 2
    
    end_point = False
    for i in range(center_y, h_end, h_dir):
        for j in range(center_x, w_end, w_dir):
            if tuple(img[i, j]) != background_color:
                points.append((i, j))
                if tuple(img[i, j + neighbor]) != background_color:
                    end_point = True
                break
        if end_point:
            break

    print(f"hole points number: {len(points)}")
    return points

def extract_bottom_points(img, background_color):
    height, width = img.shape[:2]
    # ic(height, width)
    bottom_points = []
    for i in range(width):
        for j in range(height):
            if tuple(img[j, i]) != background_color:
                bottom_points.append((j, i))
                break
    print(f"bottom points number: {len(bottom_points)}")
    return bottom_points

def extract_right_points(img, background_color):
    height, width = img.shape[:2]
    right_points = []
    for i in range(height):
        for j in range(width-1,-1,-1):
            if tuple(img[i, j]) != background_color:
                right_points.append((i, j))
                break
    print(f"right points number: {len(right_points)}")
    return right_points

def extract_top_points(img, background_color):
    height, width = img.shape[:2]
    # ic(height, width)
    top_points = []
    for i in range(width):
        for j in range(height-1,-1,-1):
            if tuple(img[j, i]) != background_color:
                top_points.append((j, i))
                break     
    print(f"top points number: {len(top_points)}")
    return top_points

def extract_left_points(img, background_color):
    height, width = img.shape[:2]
    left_points = []
    for i in range(height):
        for j in range(width):
            if tuple(img[i, j]) != background_color:
                left_points.append((i, j))
                break
            
    print(f"left points number: {len(left_points)}")
    return left_points

# 估计2d线参数
def estimate_line(points):
    """
    estimate line using all data and robustly fit line only using inlier data with RANSAC algorithm
    return: model, model_robust, inliers(mask), outliers(mask)
    """
    x = np.array([p[1] for p in points])
    y = np.array([p[0] for p in points])
    data = np.column_stack([x, y])

    # fit line using all data
    model = LineModelND()
    model.estimate(data)

    # robustly fit line only using inlier data with RANSAC algorithm
    model_robust, inliers = ransac(data, LineModelND, min_samples=2,
                                    residual_threshold=5, max_trials=1000)
    outliers = inliers == False
    print("outliers number: ", sum(outliers))

    origin, direction = model_robust.params
    print("robust model: ", origin, direction)
    return model, model_robust, inliers, outliers

# 线段拟合的可视化
def visualize_fitting_result(model, model_robust, inliers, outliers, x, y, direction):
    # predict points on the line using model and model_robust
    
    if direction == 'vertical':
        line_y = np.arange(int(y.min()), int(y.max()))
        line_x = model.predict_x(line_y)
        line_robust = model_robust.predict_x(line_y)
        # ax.plot(line_robust, line_y, '-g', label='Robust line model')

    if direction == 'horizontal':
        line_x = np.arange(int(x.min()), int(x.max()))
        line_y = model.predict_y(line_x)
        line_robust = model_robust.predict_y(line_x)
        # ax.plot(line_x, line_robust, '-g', label='Robust line model')
        # ic(line_robust)
    #     fig, ax = plt.subplots()
    #     ax.plot(x[inliers], y[inliers], '.b', alpha=0.6, label='Inlier data')
    #     ax.plot(x[outliers], y[outliers], '.r', alpha=0.6, label='Outlier data')
    #     ax.plot(line_x, line_y, '-k', label='Line model from all data')
    #     ax.plot(line_x, line_robust, '-g', label='Robust line model')
    
    # # set legend position
    #     ax.legend(loc='upper right')
    #     plt.gca().set_aspect('equal', adjustable='box')
    #     plt.show()

    return line_x, line_y, line_robust

# 转2d拟合结果到3d
def transform_d2_points_to_3d(d2_points, transform_matrix_inv):
    # transform d2 points to 3d
    d2_points = np.column_stack([d2_points, np.ones(len(d2_points)), np.ones(len(d2_points))])
    d2_points_3d = np.dot(d2_points, transform_matrix_inv.T)
    # show the first few points
    return d2_points_3d

# 验证转换到3d的线方程是正确的
def d3_line_calculation(origin, direction, lambda_min, lambda_max):
    """
    calculate 3d points on a line:
        X = origin + lambda * direction
    origin: 3d origin point
    direction: 3d direction vector
    lambda_min: minimum lambda value
    lambda_max: maximum lambda value
    return: 3d points
    """
    lambda_range = np.arange(lambda_min, lambda_max, 0.01)
    points = [origin + i * direction for i in lambda_range]
    return np.array(points)
def reverse_calculate_lambda(point, origin, direction):
    """
    reverse calculate lambda from a d2 or d3 point on a line
    """
    residual = point[:2] - origin[:2]
    return [
        (residual[0]) / (direction[0]),
        (residual[1]) / (direction[1]),
    ]

def cal_origin_and_direction(points, transform_matrix):
    model, model_robust, inliers, outliers = estimate_line(points)
    origin, direction = model_robust.params

    def transform_line_to_3d(origin, direction,transform_matrix):
        # inverse transform_matrix
        transform_matrix_inv = np.linalg.inv(transform_matrix)

        # transform origin and direction to 3d
        origin_3d = np.array([origin[0], origin[1], 1, 1]) @ transform_matrix_inv.T
        direction_3d = np.array([direction[0], direction[1], 0]) @ transform_matrix_inv.T[:3, :3]
        origin_3d = origin_3d[:3]
        direction_3d = direction_3d[:3]
        return origin_3d, direction_3d

    origin_3d, direction_3d = transform_line_to_3d(origin, direction, transform_matrix)
    return origin_3d, direction_3d

def extract_hole_line(cloud, background_color):
    img, transform_matrix = convert_pcd_to_2d_image(cloud, background_color=background_color)
    
    height, width = img.shape[:2]

    bottom_points = []
    bottom_points = extract_hole_horizontal_points(bottom_points, img, background_color, width, 1, 0, -1, 1)
    bottom_points = extract_hole_horizontal_points(bottom_points, img, background_color, 0, -1, 0, -1, 1)
    origin_3d_b, direction_3d_b = cal_origin_and_direction(bottom_points, transform_matrix)

    top_points = []
    top_points = extract_hole_horizontal_points(top_points, img, background_color, width, 1, height, 1, -1)
    top_points = extract_hole_horizontal_points(top_points, img, background_color, 0, -1, height, 1, -1)   
    origin_3d_t, direction_3d_t = cal_origin_and_direction(top_points, transform_matrix)  

    left_points = []
    left_points = extract_hole_vertical_points(left_points, img, background_color, 0, -1, 0, -1, 1)
    left_points = extract_hole_vertical_points(left_points, img, background_color, 0, -1, height, 1, 1)
    origin_3d_l, direction_3d_l = cal_origin_and_direction(left_points, transform_matrix)  

    right_points = []
    right_points = extract_hole_vertical_points(right_points, img, background_color, width, 1, 0, -1, -1)
    right_points = extract_hole_vertical_points(right_points, img, background_color, width, 1, height, 1, -1) 
    origin_3d_r, direction_3d_r = cal_origin_and_direction(right_points, transform_matrix)  

    hole_left_top = find_intersect(origin_3d_l, direction_3d_l, origin_3d_t, direction_3d_t)
    hole_right_top = find_intersect(origin_3d_r, direction_3d_r, origin_3d_t, direction_3d_t)
    hole_left_bottom = find_intersect(origin_3d_l, direction_3d_l, origin_3d_b, direction_3d_b)
    hole_right_bottom = find_intersect(origin_3d_r, direction_3d_r, origin_3d_b, direction_3d_b)

    hole_top_len = np.linalg.norm(np.array(hole_left_top) - np.array(hole_right_top))*1000
    hole_bottom_len = np.linalg.norm(np.array(hole_left_bottom) - np.array(hole_right_bottom))*1000
    hole_left_len = np.linalg.norm(np.array(hole_left_top) - np.array(hole_left_bottom))*1000
    hole_right_len = np.linalg.norm(np.array(hole_right_bottom) - np.array(hole_right_top))*1000

    hole_dia1 = np.linalg.norm(np.array(hole_right_bottom) - np.array(hole_left_top))*1000
    hole_dia2 = np.linalg.norm(np.array(hole_left_bottom) - np.array(hole_right_top))*1000
    hole_dia_diff = abs(hole_dia1 - hole_dia2)
    # ic(hole_top_len, hole_bottom_len, hole_left_len, hole_right_len)

    hole_sizes = {
        'hole_top_len': hole_top_len,
        'hole_bottom_len': hole_bottom_len,
        'hole_left_len': hole_left_len,
        'hole_right_len': hole_right_len,
        'hole_dia_diff': hole_dia_diff
    }

    # poly3dl = get_poly3d(cloud, left_points, 'vertical')
    # poly3dr = get_poly3d(cloud, right_points, 'vertical')
    # poly3db = get_poly3d(cloud, bottom_points, 'horizontal')
    # poly3dt = get_poly3d(cloud, top_points, 'horizontal')
    # ret = cc.SaveEntities([cloud, poly3dl, poly3dr, poly3db, poly3dt], "hole.bin")

    return hole_sizes

def cal_text_position(img, background_color):
    bottom_points = extract_bottom_points(img, background_color)
    left_points = extract_left_points(img, background_color)
    right_points = extract_right_points(img, background_color)
    top_points = extract_top_points(img, background_color)

    height, width = img.shape[:2]

    hole_bottom_points = []
    hole_bottom_points = extract_hole_horizontal_points(hole_bottom_points, img, background_color, width, 1, 0, -1, 1)
    hole_bottom_points = extract_hole_horizontal_points(hole_bottom_points, img, background_color, 0, -1, 0, -1, 1)

    hole_top_points = []
    hole_top_points = extract_hole_horizontal_points(hole_top_points, img, background_color, width, 1, height, 1, -1)
    hole_top_points = extract_hole_horizontal_points(hole_top_points, img, background_color, 0, -1, height, 1, -1)

    hole_left_points = []
    hole_left_points = extract_hole_vertical_points(hole_left_points, img, background_color, 0, -1, 0, -1, 1)
    hole_left_points = extract_hole_vertical_points(hole_left_points, img, background_color, 0, -1, height, 1, 1)

    hole_right_points = []
    hole_right_points = extract_hole_vertical_points(hole_right_points, img, background_color, width, 1, 0, -1, -1)
    hole_right_points = extract_hole_vertical_points(hole_right_points, img, background_color, width, 1, height, 1, -1)

    def midpoint(points):
        if not points:
            raise ValueError("输入的点列表为空")
        total_x = sum(y for x, y in points)
        total_y = sum(x for x, y in points)
        mid_x = total_x / len(points)
        mid_y = total_y / len(points)
        return (mid_x, mid_y)
    b_mid_x, b_mid_y = midpoint(bottom_points)
    l_mid_x, l_mid_y = midpoint(left_points)
    r_mid_x, r_mid_y = midpoint(right_points)
    t_mid_x, t_mid_y = midpoint(top_points)

    hb_mid_x, hb_mid_y,ht_mid_x, ht_mid_y,hl_mid_x, hl_mid_y,hr_mid_x, hr_mid_y=[None]*8

    if len(hole_bottom_points)>100:
        hb_mid_x, hb_mid_y = midpoint(hole_bottom_points)
        ht_mid_x, ht_mid_y = midpoint(hole_top_points)
        hl_mid_x, hl_mid_y = midpoint(hole_left_points)
        hr_mid_x, hr_mid_y = midpoint(hole_right_points)

    text_position={
        'b_mid_x':b_mid_x,
        'b_mid_y':b_mid_y,
        'l_mid_x':l_mid_x,
        'l_mid_y':l_mid_y,
        'r_mid_x':r_mid_x,
        'r_mid_y':r_mid_y,
        't_mid_x':t_mid_x,
        't_mid_y':t_mid_y,
        'hb_mid_x':hb_mid_x,
        'hb_mid_y':hb_mid_y,
        'ht_mid_x':ht_mid_x,
        'ht_mid_y':ht_mid_y,
        'hl_mid_x':hl_mid_x,
        'hl_mid_y':hl_mid_y,
        'hr_mid_x':hr_mid_x,
        'hr_mid_y':hr_mid_y
    }
    return text_position

def find_intersect(origin_3d_1, direction_3d_1, origin_3d_2, direction_3d_2):
        # 求解参数
        A = np.vstack((direction_3d_1, -direction_3d_2)).T
        b = origin_3d_2 - origin_3d_1
        x, y = np.linalg.lstsq(A, b, rcond=None)[0]
        # 计算交点
        intersect = origin_3d_1 + x * direction_3d_1
        return intersect
    
def get_poly3d(cloud, points, direction):
    x = np.array([p[1] for p in points])
    y = np.array([p[0] for p in points])
    model, model_robust, inliers, outliers = estimate_line(points) 
    line_x, line_y, line_robust = visualize_fitting_result(model, model_robust, inliers, outliers, x, y, direction)
    if direction == 'vertical':
        x_2d, y_2d = line_robust, line_y
    if direction == 'horizontal':
        x_2d, y_2d = line_x, line_robust
    background_color = (255, 255, 255)
    img, transform_matrix = convert_pcd_to_2d_image(cloud, background_color=background_color)
    d2_points_3d = transform_d2_points_to_3d(np.column_stack([x_2d, y_2d]), np.linalg.inv(transform_matrix))
    poly3d = cc.ccPolyline(d2_points_3d[:, :3].tolist(), False)
    poly3d.setName("poly3d")
    cloud3d = poly3d.getAssociatedCloud()
    poly3d.addChild(cloud3d)
    return poly3d

def cal_degree_with_axis(dependent, independent):
    k = (dependent[-1] - dependent[0]) / (independent[-1] - independent[0])
    radian = np.arctan(k)
    degree = np.degrees(radian)
    return degree, k

def cal_angle(cloud, background_color):
    img, transform_matrix = convert_pcd_to_2d_image(cloud, background_color)
    bottom_points = extract_bottom_points(img, background_color)
    left_points = extract_left_points(img, background_color)
    right_points = extract_right_points(img, background_color)
    model_b, model_robust_b, inliers_b, outliers_b = estimate_line(bottom_points)    
    model_l, model_robust_l, inliers_l, outliers_l = estimate_line(left_points)
    model_r, model_robust_r, inliers_r, outliers_r = estimate_line(right_points)
    x_b = np.array([p[1] for p in bottom_points])
    y_b = np.array([p[0] for p in bottom_points])
    x_l = np.array([p[1] for p in left_points])
    y_l = np.array([p[0] for p in left_points])
    x_r = np.array([p[1] for p in right_points])
    y_r = np.array([p[0] for p in right_points])

    line_x_b, line_y_b, line_robust_b = visualize_fitting_result(model_b, model_robust_b, inliers_b, outliers_b, x_b, y_b, 'horizontal')
    line_x_l, line_y_l, line_robust_l = visualize_fitting_result(model_l, model_robust_l, inliers_l, outliers_l, x_l, y_l, 'vertical')
    line_x_r, line_y_r, line_robust_r = visualize_fitting_result(model_r, model_robust_r, inliers_r, outliers_r, x_r, y_r, 'vertical')
    
    def cal_degree_with_axis_old(direction, axis):
        axis = np.array(axis)
        # Calculate the angle between the two vectors
        radian = np.arccos(np.dot(direction, axis) / (np.linalg.norm(direction) * np.linalg.norm(axis)))
        # Convert radians to degrees
        degree = np.degrees(radian)
        return degree

    bottom_to_x, _ = cal_degree_with_axis(line_robust_b, line_x_b)
    left_to_y, _ = cal_degree_with_axis(line_robust_l, line_y_l)
    right_to_y, _ = cal_degree_with_axis(line_robust_r, line_y_r)

    fitting_lv_b = (sum(1 for item in inliers_b if item))/len(inliers_b)
    fitting_lv_l = (sum(1 for item in inliers_l if item))/len(inliers_l)
    fitting_lv_r = (sum(1 for item in inliers_r if item))/len(inliers_r)
    rotated_degrees = [(bottom_to_x, fitting_lv_b), (left_to_y, fitting_lv_l), (right_to_y, fitting_lv_r)]
    ic(rotated_degrees)
    fls = [item[1] for item in rotated_degrees]
    rds = [item[0] for item in rotated_degrees]
    argmax_fls = np.argmax(fls)
    rd_argmax_fls = rds[argmax_fls]    
    if argmax_fls == 0:
        rotated_degree = -rd_argmax_fls
    else:
        rotated_degree = rd_argmax_fls

    ic(argmax_fls, rd_argmax_fls)
    ic(rotated_degree)
    # save_entities(cloud, left_points, 'vertical', line_robust_l, line_y_l, "ll.bin")
    return rotated_degree

def extract_line(cloud, background_color):
    img, transform_matrix = convert_pcd_to_2d_image(cloud, background_color)
    bottom_points = extract_bottom_points(img, background_color)
    left_points = extract_left_points(img, background_color)
    right_points = extract_right_points(img, background_color)
    model_b, model_robust_b, inliers_b, outliers_b = estimate_line(bottom_points)
    
    model_l, model_robust_l, inliers_l, outliers_l = estimate_line(left_points)
    model_r, model_robust_r, inliers_r, outliers_r = estimate_line(right_points)
    x_b = np.array([p[1] for p in bottom_points])
    y_b = np.array([p[0] for p in bottom_points])
    x_l = np.array([p[1] for p in left_points])
    y_l = np.array([p[0] for p in left_points])
    x_r = np.array([p[1] for p in right_points])
    y_r = np.array([p[0] for p in right_points])

    line_x_b, line_y_b, line_robust_b = visualize_fitting_result(model_b, model_robust_b, inliers_b, outliers_b, x_b, y_b, 'horizontal')
    line_x_l, line_y_l, line_robust_l = visualize_fitting_result(model_l, model_robust_l, inliers_l, outliers_l, x_l, y_l, 'vertical')
    line_x_r, line_y_r, line_robust_r = visualize_fitting_result(model_r, model_robust_r, inliers_r, outliers_r, x_r, y_r, 'vertical')

    origin_3d_b, direction_3d_b = cal_origin_and_direction(bottom_points, transform_matrix)
    origin_3d_l, direction_3d_l = cal_origin_and_direction(left_points, transform_matrix)  
    origin_3d_r, direction_3d_r = cal_origin_and_direction(right_points, transform_matrix)  

    d2_points_3d_l = transform_d2_points_to_3d(np.column_stack([line_robust_l, line_y_l]), np.linalg.inv(transform_matrix))
    d2_points_3d_r = transform_d2_points_to_3d(np.column_stack([line_robust_r, line_y_r]), np.linalg.inv(transform_matrix))

    left_bottom = find_intersect(origin_3d_l, direction_3d_l, origin_3d_b, direction_3d_b)
    ic(left_bottom)
    right_bottom = find_intersect(origin_3d_r, direction_3d_r, origin_3d_b, direction_3d_b)
    bottom_len = np.linalg.norm(np.array(left_bottom) - np.array(right_bottom))*1000
    ic(bottom_len)

    left_len = np.linalg.norm(d2_points_3d_l[-1, :3] - left_bottom)*1000
    ic(left_len)

    right_len = np.linalg.norm(d2_points_3d_r[-1, :3] - right_bottom)*1000
    ic(right_len)

    dia_1 = np.linalg.norm(d2_points_3d_l[-1, :3] - right_bottom)*1000
    dia_2 = np.linalg.norm(d2_points_3d_r[-1, :3] - left_bottom)*1000
    dia_diff = abs(dia_1 - dia_2)    

    left_to_y, k_left_to_y = cal_degree_with_axis(line_robust_l, line_y_l)
    ic(k_left_to_y)
    right_to_y, k_right_to_y = cal_degree_with_axis(line_robust_r, line_y_r)
    ic(k_right_to_y)
    k_diff = abs(k_left_to_y-k_right_to_y)
    
    ic(k_diff)  
    ic(left_to_y)
    # save_entities(cloud, left_points, 'vertical', line_robust_l, line_y_l, "ll.bin")
    # save_entities(cloud, right_points, 'vertical', line_robust_r, line_y_r, "rr.bin")
    # save_entities(cloud, bottom_points, 'horizontal', line_x_b, line_robust_b, "bb.bin")

    return dia_diff, k_diff

if __name__ == "__main__":

    # 加载cc点云
    main_cc = cc.loadPointCloud("cropped_cloud.ply")
    if not main_cc:
        raise ValueError("无法读取点云文件")


    # ic(img.shape)
    # plt.imshow(img)
    # plt.show()
    
    
    
    
    
    # convert 2d line to 3d line
    origin_b, direction_b = model_robust_b.params
    origin_l, direction_l = model_robust_l.params

    origin_3d_b, direction_3d_b = transform_line_to_3d(origin_b, direction_b)
    ic(direction_3d_b)
    origin_3d_l, direction_3d_l = transform_line_to_3d(origin_l, direction_l)
    ic(direction_3d_l)

    d2_points_3d_b = transform_d2_points_to_3d(np.column_stack([line_x_b, line_robust_b]), np.linalg.inv(transform_matrix))
    d2_points_3d_l = transform_d2_points_to_3d(np.column_stack([line_robust_l, line_y_l]), np.linalg.inv(transform_matrix))
    ic(d2_points_3d_b)
    ic(d2_points_3d_l)
    # 保存3d线段和原始点云
    poly3d_b = cc.ccPolyline(d2_points_3d_b[:, :3].tolist(), False)
    poly3d_l = cc.ccPolyline(d2_points_3d_l[:, :3].tolist(), False)
    poly3d_b.setName("poly3d_b")
    poly3d_l.setName("poly3d_l")
    cloud3d_b = poly3d_b.getAssociatedCloud()
    cloud3d_l = poly3d_l.getAssociatedCloud()
    poly3d_b.addChild(cloud3d_b)
    poly3d_l.addChild(cloud3d_l)
    # save entities
    ret_b = cc.SaveEntities([main_cc, poly3d_b], "mixedEntities_b.bin")
    ret_l = cc.SaveEntities([main_cc, poly3d_l], "mixedEntities_l.bin")


    points = d3_line_calculation(origin_3d, direction_3d, 520, 530)
    ic(points.shape)
    ic(points[:5, :])

    lamdba_x, lambda_y = reverse_calculate_lambda(d2_points_3d[0], origin_3d, direction_3d)
    ic(lamdba_x, lambda_y)
