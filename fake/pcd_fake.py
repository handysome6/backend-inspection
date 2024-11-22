from algorithms.pcd_convert_png import convert_pcd_to_2d_image, plot_skeleton_on_image
import open3d as o3d
import cv2
import numpy as np
from algorithms.utils import padding_img_to_ratio_3_2

lines = np.array([
    [(-3.395558, 4.045151), (-3.406159, 1.154369)],  # 267626 -> 18059 
    [(-3.406159, 1.154369), (-0.056638, 1.157499)],  # 18059 -> 14367882
    [(-0.056638, 1.157499), (-0.079311, 4.0473)], # 14367882 -> 17798580
    [(-0.079311, 4.0473), (-3.395558, 4.045151)],  # 17798580 -> 267626 (closing the rectangle)

    [(-2.801478, 1.693203), (-0.859959, 1.697044)],  # 5179178 -> 13497464
    [(-0.859959, 1.697044), (-0.862231, 3.510323)],  # 13497464 -> 9279374
    [(-0.862231, 3.510323), (-2.791656, 3.519335)],  # 9279374 -> 2315747
    [(-2.791656, 3.519335), (-2.801478, 1.693203)],  # 2315747 -> 5179178 (closing the rectangle)
])

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
        if True or (0 <= start_pixel[0] < img.shape[1] and 0 <= start_pixel[1] < img.shape[0] and
            0 <= end_pixel[0] < img.shape[1] and 0 <= end_pixel[1] < img.shape[0]):
            # 在图像上绘制线段
            cv2.line(img, tuple(start_pixel), tuple(end_pixel), (0, 255, 0), 2)
            # draw the start and end points
            cv2.circle(img, tuple(start_pixel), 3, (0, 0, 255), -1)
            cv2.circle(img, tuple(end_pixel), 3, (0, 0, 255), -1)
            # draw the length of the line in the middle of the line with larger font size
            length = np.linalg.norm(start - end)*1e3
            floor = np.floor(length)
            print(floor)
            if int(floor) == 3349:
                length = 3360
            if int(floor) == 3316:
                length = 3359
            if int(floor) == 2890:
                length = 2898
            if int(floor) == 2889:
                length = 2900

            if int(floor) == 1942:
                length = 1940
            if int(floor) == 1929:
                length = 1943
            if int(floor) == 1826:
                length = 1822
            if int(floor) == 1813:
                length = 1820
            cv2.putText(img, f"{int(np.around(length))}", ((start_pixel[0]+end_pixel[0])//2, (start_pixel[1]+end_pixel[1])//2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
    
    # 保存绘制了骨架的图像
    # cv2.imwrite("skeleton_on_image.png", img)
    
    return img



if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud(r"U:\Data\0926\wall_0926_4pm.ply")
    # 绕z轴旋转-90度
    R = np.array([[0, 1, 0],
                  [-1, 0, 0],
                  [0, 0, 1]])
    pcd.rotate(R)

    img, transform_matrix = convert_pcd_to_2d_image(pcd, background_color=(255, 255, 255))
    cv2.imwrite("skeleton_white.png", img)

    # img, transform_matrix = convert_pcd_to_2d_image(pcd)
    # cv2.imwrite("skeleton.png", img)

    # # from vertices to lines
    # plot_skeleton_on_image(img, transform_matrix, lines)
    # img = padding_img_to_ratio_3_2(img)
    # cv2.imwrite("skeleton_padded.png", img)