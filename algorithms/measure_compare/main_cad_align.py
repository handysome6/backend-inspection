import cloudComPy as cc
cc.initCC()
import cv2
import copy
import ezdxf
import numpy as np
from icecream import ic

from .main_fit_bottom_line import cal_text_position
from .main_fit_bottom_line import convert_pcd_to_2d_image
from .main_dxf_wall_points import get_wall_points
from .dxf_analyze import dxf_analyze

# 提取四个角点
def extract_four_points(main_cc) -> np.ndarray:
    """
    return:
        four_points: 2D array, shape (4, 2) 
                    in the order of bottom_left, bottom_right, top_left, top_right
    """
    # main_cc = cc.loadPointCloud(main_cc)
    main_cc_np = main_cc.toNpArray()
    main_cc_np_xy = main_cc_np[:, :2]

    corners = {}
    bottom_left_idx = np.argmin(main_cc_np_xy[:, 0] + main_cc_np_xy[:, 1])
    bottom_left = main_cc_np_xy[bottom_left_idx]
    # ic("左下角点坐标:", bottom_left)

    bottom_right_idx = np.argmin(-main_cc_np_xy[:, 0] + main_cc_np_xy[:, 1])
    bottom_right = main_cc_np_xy[bottom_right_idx]
    # ic("右下角点坐标:", bottom_right)

    top_left_idx = np.argmin(main_cc_np_xy[:, 0] - main_cc_np_xy[:, 1])
    top_left = main_cc_np_xy[top_left_idx]
    # ic("左上角点坐标:", top_left)

    top_right_idx = np.argmax(main_cc_np_xy[:, 0] + main_cc_np_xy[:, 1])
    top_right = main_cc_np_xy[top_right_idx]
    # ic("右上角点坐标:", top_right)

    return np.array([bottom_left, bottom_right, top_left, top_right])


def load_all_points_from_dxf(dxf_file) -> np.ndarray:
    """
    return:
        all_points: 2D array, shape (N, 2)
    """
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()
    all_points = []
    for entity in msp.query('LINE'):
        all_points.append((entity.dxf.start.x, entity.dxf.start.y))
        all_points.append((entity.dxf.end.x, entity.dxf.end.y))
    if not all_points:
        raise ValueError("没有找到任何点")

    return np.array(all_points)


def sort_four_points(four_points) -> np.ndarray:
    """
    Sort four points in the order of bottom_left, bottom_right, top_left, top_right
    return:
        sorted_four_points: 2D array, shape (4, 2)
    """
    bottom_left_idx = np.argmin(four_points[:, 0] + four_points[:, 1])
    cad_bottom_left = four_points[bottom_left_idx]
    bottom_right_idx = np.argmin(-four_points[:, 0] + four_points[:, 1])
    cad_bottom_right = four_points[bottom_right_idx]
    top_left_idx = np.argmax(four_points[:, 1] - four_points[:, 0])
    cad_top_left = four_points[top_left_idx]
    top_right_idx = np.argmax(four_points[:, 0] + four_points[:, 1])
    cad_top_right = four_points[top_right_idx]

    return np.array([
        [cad_bottom_left[0], cad_bottom_left[1]],
        [cad_bottom_right[0], cad_bottom_right[1]],
        [cad_top_left[0], cad_top_left[1]],
        [cad_top_right[0], cad_top_right[1]]
    ])


def draw_pc(point_cloud, background_color=(255, 255, 255)):
    """
    PLEASE ASSURE THE POINT CLOUD IS IN THE SAME SCALE/DIMENSION/LOCATION AS THE DXF FILE
    """
    # 将点云转换为2D图像
    img, transformation_matrix = convert_pcd_to_2d_image(point_cloud, background_color=background_color)
    # ic(transformation_matrix)

    return img


def draw_cad_on_img(img,cad_all_points, transformation_matrix, background_color=(255, 255, 255)):

# 将dxf端点转换为2D点
    cad_all_points_3d = np.hstack((cad_all_points, np.ones((cad_all_points.shape[0], 2))))
    cad_all_points_2d = np.dot(cad_all_points_3d, transformation_matrix.T)
    cad_all_points_2d = cad_all_points_2d[:, :2]

    # 在图像上绘制dxf线条
    for i in range(0, len(cad_all_points_2d), 2):
        cv2.line(img, (int(cad_all_points_2d[i, 0]), int(cad_all_points_2d[i, 1])), 
                 (int(cad_all_points_2d[i + 1, 0]), int(cad_all_points_2d[i + 1, 1])), 
                 (0, 255, 0), 2)

    return img

def cad_align(point_cloud, dimensions, cad_model_path, cad_img) -> tuple[np.ndarray, np.ndarray]:
    """
    parameters:
        point_cloud: point cloud with unit of mm, SCALE(1000, 1000, 1000) before input
        cad_model_path: dxf file path
        wall_sizes: dictionary containing wall sizes
        window_sizes: dictionary containing window sizes
        door_sizes: dictionary containing door sizes
        hole_sizes: dictionary containing hole sizes
        vis: whether to visualize the image
    return:
        img_grey_bg: 2D array, shape (H, W, 3)
        img_white_bg: 2D array, shape (H, W, 3)
    """
    # STEP 1: 处理点云
    pcd_four_points = extract_four_points(point_cloud)
    point_cloud_np = point_cloud.toNpArray()[:, :2]
    # ic(pcd_four_points)

    # STEP 2: 处理dxf
    dxf_four_points = get_wall_points(cad_model_path)
    cad_all_points = load_all_points_from_dxf(cad_model_path)
    dimensions, value_from_CAD = dxf_analyze(cad_model_path)

    # STEP 3: 对dxf四点排序并且平移归零
    dxf_four_points = sort_four_points(dxf_four_points)
    base_point = dxf_four_points[0].copy()
    dxf_four_points -= base_point
    cad_all_points -= base_point
    value_from_CAD = sort_and_zero_value_from_CAD(value_from_CAD, base_point)
    # ic("排序并归零后的:", value_from_CAD)
    # ic(dxf_four_points)

    # STEP 4: 计算平移向量
    translated_vectors = dxf_four_points - pcd_four_points
    average_translation = np.mean(translated_vectors, axis=0)
    point_cloud_np += average_translation
    # ic("平移向量: ", average_translation)
    # ic(pcd_four_points + average_translation)

    # STEP 5: 将点云转换为2D图像
    img_white_bg, _ = convert_pcd_to_2d_image(point_cloud, background_color=(255, 255, 255))
    img_grey_bg, transformation_matrix = convert_pcd_to_2d_image(point_cloud, background_color=(48, 40, 33))

    if cad_img == True:
        # STEP 6: 将dxf端点绘制在图像上
        img_white_bg = draw_cad_on_img(img_white_bg, cad_all_points, transformation_matrix, background_color=(255, 255, 255))
        img_grey_bg = draw_cad_on_img(img_grey_bg, cad_all_points, transformation_matrix, background_color=(48, 40, 33))

        # STEP 7：绘制文本 
        value_from_CAD_copy = copy.deepcopy(value_from_CAD)
        img_grey_bg = draw_CADtext_on_img(img_grey_bg, value_from_CAD_copy, dimensions, transformation_matrix)
        # value_from_CAD_copy = copy.deepcopy(value_from_CAD)
        # img_white_bg = draw_CADtext_on_img(img_white_bg, value_from_CAD_copy,  transformation_matrix)
    
    # if vis:
    #     cv2.namedWindow("img_grey_bg", cv2.WINDOW_NORMAL)
    #     cv2.imshow("img_grey_bg", img_grey_bg)
    #     cv2.namedWindow("img_white_bg", cv2.WINDOW_NORMAL)
    #     cv2.imshow("img_white_bg", img_white_bg)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()

    return img_grey_bg, img_white_bg

def sort_and_zero_value_from_CAD(value_from_CAD, base_point):
    """
    Sort and zero the four points of value_from_CAD.
    对value_from_CAD进行排序并归零。
    """
    for layer, entities in value_from_CAD.items():
        for entity in entities:
            entity['start'] -= base_point
            entity['end'] -= base_point
    # ic("排序并归零后的:", value_from_CAD)
    return value_from_CAD


import random
def draw_CADtext_on_img(img, value_from_CAD, dimensions, transformation_matrix):
    """
    Draw lines and text on the image.
    在图像上绘制线条和文本。
    """
    all_mid_points = []

    # 将value_from_CAD转换为2D点
    for layer, entities in value_from_CAD.items():
        for entity in entities:
            start = np.array([entity['start'][0], entity['start'][1], 1, 1])
            end = np.array([entity['end'][0], entity['end'][1], 1, 1])
            start_2d = np.dot(start, transformation_matrix.T)[:2]
            end_2d = np.dot(end, transformation_matrix.T)[:2]
            entity['start'] = start_2d
            entity['end'] = end_2d

            # 计算每个线段的中心点
            mid_point = ((start_2d[0] + end_2d[0]) / 2, (start_2d[1] + end_2d[1]) / 2)
            all_mid_points.append(mid_point)

    # 计算所有线段的整体中心点
    overall_mid_point = np.mean(all_mid_points, axis=0)
    # ic("整体中心点坐标:", overall_mid_point)
    # cv2.circle(img, (int(overall_mid_point[0]), int(overall_mid_point[1])), 5, (0, 0, 255), -1) # 在图像上绘制整体中心点
    ic(dimensions)
    # 绘制垂直线和水平线的文本
    for layer, entities in value_from_CAD.items():
        for entity in entities:
            start = entity['start']
            end = entity['end']
            length = entity['length']

            if abs(start[0] - end[0]) > 0.3:
                # 垂直线
                mid_point = (int(start[0]), int((start[1] + end[1]) // 2))
                text = f"{length:.2f}mm"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                #如果是 wall 图层
                if layer == 'wall':
                    text = f"{length:.2f}mm"
                    # 判断文本的位置
                    if start[0] < overall_mid_point[0]:                    
                        text_pos = (int(mid_point[0]) - 60, int(overall_mid_point[1])) 
                        if 'wall_height_left' in dimensions and dimensions['wall_height_left'] != 0:
                            img = create_transparent_text_image(img,"{:.2f}mm".format(dimensions['wall_height_left']),text_pos[0],text_pos[1],direction='COUNTERCLOCKWISE')
                        # rotation_matrix = cv2.getRotationMatrix2D((text_pos[0], text_pos[1]), 90, 1) #左
                    else:
                        text_pos = (int(mid_point[0]) - 30, int(overall_mid_point[1] - 150))
                        if 'wall_height_right' in dimensions and dimensions['wall_height_right'] != 0:
                            img = create_transparent_text_image(img,"{:.2f}mm".format(dimensions['wall_height_right']),text_pos[0],text_pos[1],direction='CLOCKWISE')
                        # rotation_matrix = cv2.getRotationMatrix2D((text_pos[0], text_pos[1]), -90, 1) #右
                else:
                    if start[0] < overall_mid_point[0]:
                        text_pos = (int(mid_point[0]) - 20, int(overall_mid_point[1])) 
                        if 'window_height_left' in dimensions and dimensions['window_height_left'] != 0:
                            img = create_transparent_text_image(img,"{:.2f}mm".format(dimensions['window_height_left']),text_pos[0],text_pos[1],direction='COUNTERCLOCKWISE')
                        elif 'door_height_left' in dimensions and dimensions['door_height_left'] != 0:
                            img = create_transparent_text_image(img,"{:.2f}mm".format(dimensions['door_height_left']),text_pos[0],text_pos[1],direction='COUNTERCLOCKWISE')
                        # rotation_matrix = cv2.getRotationMatrix2D((text_pos[0], text_pos[1]), 90, 1)  #左
                    else:
                        text_pos = (int(mid_point[0]) - 80, int(overall_mid_point[1] - 150))
                        if 'window_height_right' in dimensions and dimensions['window_height_right'] != 0:
                            img = create_transparent_text_image(img,"{:.2f}mm".format(dimensions['window_height_right']),text_pos[0],text_pos[1],direction='CLOCKWISE')
                        elif 'door_height_right' in dimensions and dimensions['door_height_right'] != 0:
                            img = create_transparent_text_image(img,"{:.2f}mm".format(dimensions['door_height_right']),text_pos[0],text_pos[1],direction='CLOCKWISE')
                        # rotation_matrix = cv2.getRotationMatrix2D((text_pos[0], text_pos[1]), -90, 1) #右

                # text_img = np.zeros_like(img)
                # cv2.putText(text_img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                # text_img = cv2.warpAffine(text_img, rotation_matrix, (img.shape[1], img.shape[0]))
                # img = cv2.add(img, text_img)

            if abs(start[1] - end[1]) <= 0.3:
                # 水平线
                mid_point = (int((start[0] + end[0]) / 2), int(start[1]))
                text = f"{length:.2f}mm"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                # 判断文本的位置
                if layer == 'wall': # wall 图层
                    if start[1] < overall_mid_point[1]:
                        text_pos = (int(overall_mid_point[0]), int(mid_point[1]) - text_size[1] + 10)
                    else:
                        text_pos = (int(overall_mid_point[0]), int(mid_point[1]) + text_size[1] - 30)
                else:
                    if start[1] > overall_mid_point[1]:
                        text_pos = (int(overall_mid_point[0]), int(mid_point[1]) - text_size[1] - 5)
                    else:
                        text_pos = (int(overall_mid_point[0]), int(mid_point[1]) + text_size[1] + 5)

                cv2.putText(img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    return img

def create_transparent_text_image(img_bg,text,x,y,direction='CLOCKWISE'):
    
    img_text = np.zeros((100, 200, 4), dtype=np.uint8)
    cv2.putText(img_text, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255, 255), 2)

    # 将图像逆时针旋转90度
    if direction == 'CLOCKWISE':
        img_text_rotated = cv2.rotate(img_text, cv2.ROTATE_90_CLOCKWISE)
    elif direction == 'COUNTERCLOCKWISE':
        img_text_rotated = cv2.rotate(img_text, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # 提取文本图像中的 alpha 通道
    alpha_channel = img_text_rotated[:, :, 3] / 255.0

    # 将文本图像合并到背景图像中
    for c in range(3):
        img_bg[y:y + img_text_rotated.shape[0], x:x + img_text_rotated.shape[1], c] = \
            img_text_rotated[:, :, c] * alpha_channel + img_bg[y:y + img_text_rotated.shape[0], x:x + img_text_rotated.shape[1], c] * (1 - alpha_channel)
    return img_bg

def draw_text_on_img(img1, img2, wall_sizes, window_sizes, door_sizes):
    background_color = (255, 255, 255)
    text_position = cal_text_position(img1, background_color)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 0, 255)
    thickness = 2

    def input_img(img):
        height, width = img.shape[:2]
        # ic(height, width)
        if wall_sizes['bottom_width']:
            cv2.putText(img, "{:.2f}mm".format(wall_sizes['bottom_width']), (int(text_position['b_mid_x']), int(text_position['b_mid_y']-20)), font, font_scale, font_color, thickness)
        if wall_sizes['left_height']:
            # img = create_transparent_text_image(img,"{:.2f}".format(wall_sizes['left_height']),int(text_position['l_mid_x']-50), int(text_position['l_mid_y']),direction='COUNTERCLOCKWISE')
            img = create_transparent_text_image(img,"{:.2f}mm".format(wall_sizes['left_height']),width//16, int(text_position['l_mid_y']),direction='COUNTERCLOCKWISE')
        if wall_sizes['right_height']:
            # img = create_transparent_text_image(img,"{:.2f}".format(wall_sizes['right_height']),int(text_position['r_mid_x']+50), int(text_position['r_mid_y']),direction='CLOCKWISE')
            img = create_transparent_text_image(img,"{:.2f}mm".format(wall_sizes['right_height']),(width//17)*15, int(text_position['r_mid_y']),direction='CLOCKWISE')
        if wall_sizes['top_width']:
            cv2.putText(img, "{:.2f}mm".format(wall_sizes['top_width']), (int(text_position['t_mid_x']), int(text_position['t_mid_y']+20)), font, font_scale, font_color, thickness)
        
        if window_sizes['window_width']:
            cv2.putText(img, "{:.2f}mm".format(window_sizes['window_width']), (int(text_position['hb_mid_x']), int(text_position['hb_mid_y']+30)), font, font_scale, font_color, thickness)
        if window_sizes['window_height']:
            img = create_transparent_text_image(img,"{:.2f}mm".format(window_sizes['window_height']),int(text_position['hl_mid_x']), int(text_position['hl_mid_y']),direction='COUNTERCLOCKWISE')
            # (int(text_position['hr_mid_x']-20), int(text_position['hr_mid_y']))
            # (int(text_position['ht_mid_x']), int(text_position['ht_mid_y']-20))
        
        if door_sizes['door_width']:
            cv2.putText(img, "{:.2f}mm".format(door_sizes['door_width']), (int(text_position['hb_mid_x']), int(text_position['hb_mid_y']+30)), font, font_scale, font_color, thickness)
        if door_sizes['door_height']:
            img = create_transparent_text_image(img,"{:.2f}mm".format(door_sizes['door_height']),int(text_position['hl_mid_x']), int(text_position['hl_mid_y']),direction='COUNTERCLOCKWISE')
            # (int(text_position['hr_mid_x']-20), int(text_position['hr_mid_y']))
            # (int(text_position['ht_mid_x']), int(text_position['ht_mid_y']-20))

        return img

    img_white_bg = input_img(img1)
    img_grey_bg = input_img(img2)
    return img_white_bg, img_grey_bg  



# def draw_text_on_img(img, wall_sizes, window_sizes, door_sizes, hole_sizes, dimensions):
#     text = []

#     def append_size_info(text, label, size):
#         """Helper function to append size information."""
#         if size is not None:
#             text.append(f"{label}: {size:.2f}mm")
#         else:
#             text.append(f"{label}: N/A")

#     # 检查 wall_sizes 是否为有效字典
#     if wall_sizes is not None and len(wall_sizes) > 0:
#         append_size_info(text, "wall width (bottom)", wall_sizes.get('bottom_width'))
#         append_size_info(text, "wall width (top)", wall_sizes.get('top_width'))
#         append_size_info(text, "wall height (left)", wall_sizes.get('left_height'))
#         append_size_info(text, "wall height (right)", wall_sizes.get('right_height'))
#         append_size_info(text, "wall diagonal difference", wall_sizes.get('dia_diff'))

#     # 检查 dimensions 是否包含窗户相关的键
#     if any('window' in key for key in dimensions):
#         append_size_info(text, "windows width", hole_sizes.get('hole_bottom_len'))
#         append_size_info(text, "window height", hole_sizes.get('hole_left_len'))
#         append_size_info(text, "window diagonal difference", hole_sizes.get('hole_dia_diff'))

#     # 检查 dimensions 是否包含门相关的键
#     if any('door' in key for key in dimensions):
#         append_size_info(text, "door width", hole_sizes.get('hole_bottom_len'))
#         append_size_info(text, "door height", hole_sizes.get('hole_left_len'))
#         append_size_info(text, "door diagonal difference", hole_sizes.get('hole_dia_diff'))
#     ic(text)

#     # Define text parameters
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     font_scale = 0.5
#     font_color = (226, 155, 11)
#     thickness = 1
#     line_height = 20
#     start_x = 10
#     start_y = 30

#     # Add each line of text to the image
#     # for i, t in enumerate(text):
#     #     text_y = start_y + i * line_height
#     #     cv2.putText(img, t, (start_x, text_y), font, font_scale, font_color, thickness)

#     return img



if __name__ == "__main__":
    point_cloud_path = r"data\20241210.ply"
    cad_model_path = r"data\20241210.dxf"
    point_cloud = cc.loadPointCloud(point_cloud_path)
    # 将点云缩放1000倍，单位从m变为mm
    point_cloud.scale(1000, 1000, 1000)
    cad_align(point_cloud, cad_model_path)