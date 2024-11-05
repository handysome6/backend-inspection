#################################################
# This file contains utility functions that are #
# used in the main script.                      #
#################################################
import numpy as np
from icecream import ic
from loguru import logger

def linear_interpolation(x, x0, y0, x1, y1):
    """
    Performs linear interpolation to estimate the value of y at a given x
    based on two known points (x0, y0) and (x1, y1).
    """
    if x0 == x1:
        raise ValueError("x0 and x1 must be different")

    # Calculate the slope
    slope = (y1 - y0) / (x1 - x0)

    # Calculate the y-intercept
    intercept = y0 - slope * x0

    # Calculate the estimated y value
    y = slope * x + intercept

    return y

def bilinear_interpolation(x, y, x0, y0, x1, y1, q00, q01, q10, q11):
    """
    Performs bilinear interpolation to estimate the value of z at a given (x, y)
    based on the values of q at the four corners (x0, y0), (x0, y1), (x1, y0), and (x1, y1).
    """
    if x0 == x1 or y0 == y1:
        raise ValueError("x0 and x1, and y0 and y1, must be different")

    # Perform linear interpolation in the x-direction
    r0 = linear_interpolation(x, x0, q00, x1, q10)
    r1 = linear_interpolation(x, x0, q01, x1, q11)

    # Perform linear interpolation in the y-direction
    z = linear_interpolation(y, y0, r0, y1, r1)

    return z

# time function by decorators
import time
def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        print(f'{method.__name__} took: {te-ts} sec\n')
        return result
    return timed

def filter_avg(src_pts, dst_pts):
    """
    given matced id pairs, filter out wrong pairs
    """
    original_len = len(src_pts)

    src_pts = np.array(src_pts)
    dst_pts = np.array(dst_pts)
    ic(src_pts.shape)
    ic(dst_pts.shape)
    src_x = src_pts[:, 0]
    dst_x = dst_pts[:, 0]

    # 标准偏差法
    # 标准偏差法（Standard Deviation Method）使用数据的标准差来判断数据点是否偏离了数据。上界和下界是均值和3倍标准差的加减。
    # 计算平均值和标准偏差： 首先，计算数据的平均值（Mean）和标准偏差（Standard Deviation）。平均值代表了数据的中心位置，标准偏差衡量了数据的分散程度。
    # 确定阈值： 定义一个阈值，通常是标准偏差的倍数（通常为2或3倍标准偏差）。这个阈值决定了什么样的数据点被认为是离群值。
    distance_x = src_x - dst_x
    mean = np.mean(distance_x)
    std = np.std(distance_x)
    upper_bound = mean + 1 * std
    lower_bound = mean - 1 * std

    x = np.argwhere((distance_x>lower_bound) & (distance_x<upper_bound))    # shape: (n, 1, 2)
    x = x.flatten()     # shape: (n, 2)
    filtered_len = len(x)

    logger.info(f"Before len: {original_len}, after len: {filtered_len}")
    logger.info(f"Filtered number of outlier: {original_len - filtered_len}")

    return src_pts[x], dst_pts[x]

def padding_img_to_ratio_3_2(input_image):
    """
    将图像填充到3:2的宽高比
    """
    import numpy as np
    
    # 获取原始图像尺寸
    original_height, original_width = input_image.shape[:2]

    # 获取左上角像素的颜色作为填充色
    padding_color = input_image[0, 0]

    # 计算新尺寸以适应3:2的宽高比
    target_ratio = 3 / 2
    new_height = original_height
    new_width = int(new_height * target_ratio)

    if new_width < original_width:
        new_width = original_width
        new_height = int(new_width / target_ratio)

    # 创建新图像并用填充色填充
    new_image = np.full((new_height, new_width, 3), padding_color, dtype=np.uint8)

    # 计算原始图像在新图像中的位置
    x_position = (new_width - original_width) // 2
    y_position = (new_height - original_height) // 2

    # 将原始图像粘贴到新图像上
    new_image[y_position:y_position+original_height, x_position:x_position+original_width] = input_image
    return new_image

