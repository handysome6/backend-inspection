
import cv2
import numpy as np
import open3d as o3d
import math
from icecream import ic


def convert_pcd_to_2d_image(pcd) -> tuple[np.ndarray, np.ndarray]:
    """
    convert open3d point cloud to 2d image
    ---------------------------------------------------------------------------
    parameters:
    ----------
        pcd: open3d point cloud

    returns:
    --------
        2d image and transform matrix
    """
    # get all points change z-coord of points to 1
    points = np.asarray(pcd.points)
    points[:, 2] = 1
    
    # [0, 1] to uint8 [0, 255]
    colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
    print(colors.shape)

    x_min, y_min, _ = pcd.get_min_bound()
    x_max, y_max, _ = pcd.get_max_bound()
    x_range = x_max - x_min
    y_range = y_max - y_min

    img_x_range = 2000
    img_y_range = int(y_range * img_x_range / x_range)

    img_width = img_x_range + 300
    img_height = img_y_range + 300

    # default background is BGR(48, 40, 33)
    # current background is white(255, 255, 255)
    img = np.full((img_height, img_width, 3), (48, 40, 33), dtype=np.uint8)
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
    print(f"总点数: {len(points)}")
    print(f"有效点数: {np.sum(valid_mask)}")
    print(f"无效点数: {len(points) - np.sum(valid_mask)}")

    return img, transform_matrix


def plot_skeleton_on_image(img, transform_matrix, lines) -> np.ndarray:
    """
    plot skeleton on 2d image
    ---------------------------------------------------------------------------
    parameters:
    ----------
        img: 2d image
        transform_matrix: transform matrix from point cloud to image
        lines: [[start, end], [start, end], ...]

    returns:
    --------
        2d image with skeleton
    """
    for line in lines:
        start, end = line
        
        # 将端点转换为齐次坐标
        start_homogeneous = np.append(start, [1, 1])
        end_homogeneous = np.append(end, [1, 1])
        # from icecream import ic
        # ic(start_homogeneous)
        # ic(end_homogeneous)
        # ic(transform_matrix)
        
        # 使用变换矩阵转换端点
        start_transformed = np.dot(transform_matrix, start_homogeneous)
        end_transformed = np.dot(transform_matrix, end_homogeneous)
        
        # 提取转换后的x和y坐标，并转换为整数类型
        start_pixel = start_transformed[:2].astype(int)
        end_pixel = end_transformed[:2].astype(int)
        
        # 检查端点是否在图像范围内
        if (0 <= start_pixel[0] < img.shape[1] and 0 <= start_pixel[1] < img.shape[0] and
            0 <= end_pixel[0] < img.shape[1] and 0 <= end_pixel[1] < img.shape[0]):
            # 在图像上绘制线段
            cv2.line(img, tuple(start_pixel), tuple(end_pixel), (0, 255, 0), 2)
            # draw the start and end points
            cv2.circle(img, tuple(start_pixel), 3, (0, 0, 255), -1)
            cv2.circle(img, tuple(end_pixel), 3, (0, 0, 255), -1)
            # draw the length of the line in the middle of the line with larger font size
            length = np.linalg.norm(start - end)*1e3
            cv2.putText(img, f"{int(np.around(length))}", ((start_pixel[0]+end_pixel[0])//2, (start_pixel[1]+end_pixel[1])//2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
    
    # 保存绘制了骨架的图像
    # cv2.imwrite("skeleton_on_image.png", img)
    
    return img


def draw_cad_style_line(img, start, end, color=(0, 255, 0), thickness=2, short_line_length=100):
    # 绘制主线
    # cv2.line(img, start, end, color, thickness)

    # radian to degree
    line_angle = np.arctan2(end[1] - start[1], end[0] - start[0]) * 180 / np.pi
    
    # 计算主线的方向向量
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = int(np.around(math.sqrt(dx**2 + dy**2)))
    
    # 计算垂直于主线的单位向量
    perpendicular_x = -dy / length
    perpendicular_y = dx / length
    

    # the short line's dragging direction should be from center to outer region
    
    # 计算短线的端点
    # 判断线段是否在图像的左上方或右下方
    image_center_x = img.shape[1] // 2
    image_center_y = img.shape[0] // 2
    
    # 计算线段中点
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2

    # line function that connect top-right and bottom-left
    line_function = lambda x: (x - image_center_x) * (image_center_y - image_center_x) / (image_center_x - image_center_y) + image_center_y
    
    if (mid_y < line_function(mid_x)):
        print("left and up")
        # 左上方
        start_short = np.array([int(start[0] - short_line_length * perpendicular_x),
                       int(start[1] - short_line_length * perpendicular_y)])
        end_short = np.array([int(end[0] - short_line_length * perpendicular_x),
                     int(end[1] - short_line_length * perpendicular_y)])
    else:
        print("right and down")
        # right and down
        start_short = np.array([int(start[0] + short_line_length * perpendicular_x),
                       int(start[1] + short_line_length * perpendicular_y)])
        end_short = np.array([int(end[0] + short_line_length * perpendicular_x),
                     int(end[1] + short_line_length * perpendicular_y)])


    # 红色虚线
    cv2.line(img, start, start_short, (0, 0, 255), thickness, cv2.LINE_AA)
    cv2.line(img, end, end_short, (0, 0, 255), thickness, cv2.LINE_AA)
    # 绘制连接短线端点的虚线
    dashes = np.linspace(0, 1, 20).reshape(-1, 2)
    for dash_start, dash_end in dashes:
        pt1 = (int(start_short[0] + (end_short[0] - start_short[0]) * dash_start),
               int(start_short[1] + (end_short[1] - start_short[1]) * dash_start))
        pt2 = (int(start_short[0] + (end_short[0] - start_short[0]) * dash_end),
               int(start_short[1] + (end_short[1] - start_short[1]) * dash_end))
        cv2.line(img, pt1, pt2, (0, 0, 255), thickness, cv2.LINE_AA)

    angle = line_angle - 90
    # 绘制连接短线端点的线段长度
    from test_pil import draw_rotated_text
    # text center should locate in the middle of the short line
    txt_center = (start_short[0] + (end_short[0] - start_short[0]) // 2, start_short[1] + (end_short[1] - start_short[1]) // 2)
    draw_rotated_text(img, -line_angle, txt_center, f"{length:.2f}", (0, 0, 255))
    return img


def plot_skeleton_cad_style_on_image(img, transform_matrix, lines) -> np.ndarray:
    """
    plot skeleton on 2d image in CAD style
    """
    for line in lines:
        start, end = line
        
        # 将端点转换为齐次坐标
        start_homogeneous = np.append(start, [1, 1])
        end_homogeneous = np.append(end, [1, 1])
        
        # 使用变换矩阵转换端点
        start_transformed = np.dot(transform_matrix, start_homogeneous)
        end_transformed = np.dot(transform_matrix, end_homogeneous)
        
        # 提取转换后的x和y坐标，并转换为整数类型
        start_pixel = start_transformed[:2].astype(int)
        end_pixel = end_transformed[:2].astype(int)
        
        # 检查端点是否在图像范围内
        if (0 <= start_pixel[0] < img.shape[1] and 0 <= start_pixel[1] < img.shape[0] and
            0 <= end_pixel[0] < img.shape[1] and 0 <= end_pixel[1] < img.shape[0]):
            # 在图像上绘制线段cad style
            draw_cad_style_line(img, start_pixel, end_pixel)
            # draw the start and end points
            cv2.circle(img, tuple(start_pixel), 3, (0, 0, 255), -1)
            cv2.circle(img, tuple(end_pixel), 3, (0, 0, 255), -1)

    # 保存绘制了骨架的图像
    cv2.imwrite("skeleton_on_image_cad_style.png", img)
    return img


if __name__ == "__main__":
    # read point cloud
    pcd = o3d.io.read_point_cloud(r"C:\Users\14904\Downloads\mic_measure_portable\skeleton_sampled.ply")
    img, transform_matrix = convert_pcd_to_2d_image(pcd)
    lines = np.array([
                    [[-3.01558614,  4.08597059],
                    [-0.11401578,  4.04239056]],
                    [[-3.05156016,  0.72506085],
                    [-0.15674022,  0.68655884]],

                    [[-3.01558614,  4.08597059],
                    [-3.05156016,  0.72506085]],
                    [[-0.11401578,  4.04239056],
                    [-0.15674022,  0.68655884]],

                    [[-3.01558614,  4.08597059],
                    [-0.15674022,  0.68655884]],
                    [[-3.05156016,  0.72506085],
                    [-0.11401578,  4.04239056]],


                    
                    [[-2.51052451,  1.52939695],
                    [-0.69158679,  1.50310343]],
                    [[-0.69311361,  1.51119602],
                    [-0.66991269,  3.44534588]],
                    [[-2.48275375,  3.46892097],
                    [-0.67501831,  3.44295628]],
                    [[-2.51725961,  1.5309999 ],
                    [-2.49024551,  3.46091294]],
                    # 添加对角线
                    [[-2.51052451,  1.52939695],
                    [-0.67501831,  3.44295628]],
                    [[-0.69158679,  1.50310343],
                    [-2.48275375,  3.46892097]],


    ])

    plotted_img = plot_skeleton_cad_style_on_image(img, transform_matrix, lines)
    cv2.imshow("plotted_img", plotted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

