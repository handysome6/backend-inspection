
from algorithms.pcl_pybind import BoundaryFitting

import cv2
import math
import ezdxf
import shutil
import openpyxl
import numpy as np
import open3d as o3d
from icecream import ic
from pathlib import Path
from openpyxl.styles import Alignment, Font
from matplotlib import pyplot as plt

from .utils import padding_img_to_ratio_3_2
from .dxf_analyze import process_dxf_file, plot_cad, value_from_CAD
from .pcd_convert_png import convert_pcd_to_2d_image, plot_skeleton_on_image

def rotation_matrix_from_vectors(vec1, vec2):
    # Modified from https://segmentfault.com/q/1010000043308109
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def get_plane(pcd, vis=False):
    # 检测平面
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    
    R_plane = rotation_matrix_from_vectors(plane_model[:3], np.array([0,0,1]))

    orthogonal_plane_pcd = o3d.geometry.PointCloud()
    tmp_points = np.array(pcd.points)
    tmp_points = tmp_points @ R_plane.T
    tmp_points[:,2] = 0
    orthogonal_plane_pcd.points = o3d.utility.Vector3dVector(tmp_points)

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=4, origin=[0, 0, 0])
    if vis:
        o3d.visualization.draw_geometries([orthogonal_plane_pcd, axis_pcd],
                                        window_name="Plane")
    return orthogonal_plane_pcd


# ============= 旋转点云 ============
def rotate_point_cloud(points, angle):
    # 将角度转换为弧度
    theta = np.radians(angle)  # 90° 转换为弧度
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # 旋转点云，只旋转 x 和 y 坐标
    rotated_points = np.dot(points[:, :2], rotation_matrix)
    return np.hstack((rotated_points, points[:, 2:]))  # 保持 z 坐标不变

# ============= 按水平和垂直方向分类线段 ============
def calssify_lines(lines_func, edges):
    angles = []
    line_lengths = []
    labels = []
    line_info = {}    

    for idx, (line, edge) in enumerate(zip(lines_func, edges)):
        slope = -line[0] / line[1]  # 计算斜率
        angle = math.degrees(math.atan(slope))
        angles.append(angle)
        length = np.linalg.norm(edge[1] - edge[0])  # 计算长度
        line_lengths.append(length)
        labels.append(idx)
        line_info[idx] = {'label': idx, 'angle': angle, 'length': length, 'edge': edge}

    for idx, info in line_info.items():
        print(f"Line {info['label']}: Angle = {info['angle']:.2f}, Length = {info['length']:.2f}, Edge = {info['edge']}")
    # 分出水平和垂直的线
    horizontal_lines = {idx: info for idx, info in line_info.items() if abs(info['angle']) <= 10}
    vertical_lines = {idx: info for idx, info in line_info.items() if abs(info['angle']) >= 80}
    # 根据y坐标排序水平线
    sort_H_byYcoord = sorted(horizontal_lines.values(), key=lambda x: x['edge'][0][1], reverse=False)
    # 根据x坐标排序垂直线
    sort_V_byXcoord = sorted(vertical_lines.values(), key=lambda x: x['edge'][0][0], reverse=False)
    return horizontal_lines, vertical_lines, sort_V_byXcoord, sort_H_byYcoord

# ============= 判断平行 ============
def determine_parallel_lines(sort_V_byXcoord):            
        # 计算水平方向墙板的角度差
        angle_diff = abs(sort_V_byXcoord[0]['angle'] - sort_V_byXcoord[3]['angle'])
        ic(angle_diff)
        # 差值小于0.03即判断为平行线
        para_result = "是" if angle_diff <= 1 else "否"
        ic(para_result)
        # ic(sort_H_byYcoord[0]['label'])
        # ic(sort_H_byYcoord[3]['label'])
        return para_result

# ============= 计算交点 ============
def cal_intersections(lines_func):
    intersections = []
    # 计算直线的交点
    for i in range(len(lines_func)):
        for j in range(i + 1, len(lines_func)):
            A = np.array([lines_func[i][:2], lines_func[j][:2]])
            b = np.array([lines_func[i][2], lines_func[j][2]])
            intersection = np.linalg.solve(A, b)
            if abs(intersection[0]) < 10 and abs(intersection[1]) < 10:
                intersections.append(intersection)
    sort_byXcoord = sorted(intersections, key=lambda x: x[0], reverse=False)
    sort_byYcoord = sorted(intersections, key=lambda x: x[1], reverse=False)

    pointA = next((point for point in sort_byXcoord[:4] if tuple(point) in map(tuple, sort_byYcoord[:4])), None)
    pointB = next((point for point in sort_byXcoord[:4] if tuple(point) in map(tuple, sort_byYcoord[-4:])), None)
    pointC = next((point for point in sort_byXcoord[-4:] if tuple(point) in map(tuple, sort_byYcoord[-4:])), None)
    pointD = next((point for point in sort_byXcoord[-4:] if tuple(point) in map(tuple, sort_byYcoord[:4])), None)

    pointE = next((point for point in sort_byXcoord[4:8] if tuple(point) in map(tuple, sort_byYcoord[4:8])), None)
    pointF = next((point for point in sort_byXcoord[4:8] if tuple(point) in map(tuple, sort_byYcoord[-8:-4])), None)
    pointG = next((point for point in sort_byXcoord[-8:-4] if tuple(point) in map(tuple, sort_byYcoord[-8:-4])), None)
    pointH = next((point for point in sort_byXcoord[-8:-4] if tuple(point) in map(tuple, sort_byYcoord[4:8])), None)
    vertices = [pointA, pointB, pointC, pointD, pointE, pointF, pointG, pointH]
    return vertices

# ============= 计算对角线差值 ============
# 计算对角线
def cal_diagonal_diff(vertices, bottom_left_point, bottom_right_point):    
    diagonal1 = np.array([vertices[1], bottom_right_point])  # B-br
    diagonal2 = np.array([vertices[2], bottom_left_point])  # C-bl
    diagonal3 = np.array([vertices[4], vertices[6]])  # E-G
    diagonal4 = np.array([vertices[5], vertices[7]])  # F-H

    # 计算对角线长度
    length_diag1 = np.linalg.norm(vertices[1] - bottom_right_point)
    # length_diag1 = np.linalg.norm(diagonal1[0] - diagonal1[1])
    length_diag2 = np.linalg.norm(vertices[2] - bottom_left_point)
    # length_diag2 = np.linalg.norm(diagonal2[0] - diagonal2[1])
    length_diag3 = np.linalg.norm(diagonal3[0] - diagonal3[1])
    length_diag4 = np.linalg.norm(diagonal4[0] - diagonal4[1])

    # 计算对角线差值
    wall_diagonal_diff = round(abs(length_diag1 - length_diag2)*1000, 2)
    window_diagonal_diff = round(abs(length_diag3 - length_diag4)*1000, 2)

    return wall_diagonal_diff, window_diagonal_diff

# ============= 点云识别线段显示 ============
def plot_PointCloud_line(vertices, bottom_left_point, bottom_right_point):
    # 创建四边形的顶点数组
    quadrilateral1 = np.array([bottom_left_point, vertices[1], vertices[2], bottom_right_point, bottom_left_point])  # 第一个四边形
    quadrilateral2 = np.array([vertices[4], vertices[5], vertices[6], vertices[7], vertices[4]])  # 第二个四边形

    # 创建图形
    plt.figure(figsize=(10, 10))

    # 绘制墙壁边缘
    plt.plot(quadrilateral1[:, 0], quadrilateral1[:, 1], 'b-', label='wall-edge')
    plt.plot([bottom_left_point[0], vertices[2][0]], [bottom_left_point[1], vertices[2][1]], 'r--', label='wall-diagonal')
    plt.plot([vertices[1][0], bottom_right_point[0]], [vertices[1][1], bottom_right_point[1]], 'r--')

    # 绘制窗户边缘
    plt.plot(quadrilateral2[:, 0], quadrilateral2[:, 1], 'g-', label='window-edge')
    plt.plot([vertices[4][0], vertices[6][0]], [vertices[4][1], vertices[6][1]], 'm--', label='window-diagonal')
    plt.plot([vertices[5][0], vertices[7][0]], [vertices[5][1], vertices[7][1]], 'm--')

    edges = [
        (bottom_left_point, vertices[1]),  # 左边
        (vertices[1], vertices[2]),  # 第一条边
        (vertices[2], vertices[3]),  # 第二条边
        (bottom_right_point, bottom_left_point),  # 底边
        (vertices[4], vertices[5]),  # 窗户边
        (vertices[5], vertices[6]),  # 窗户边
        (vertices[6], vertices[7]),  # 窗户边
        (vertices[7], vertices[4]),  # 窗户边
    ]

    for p1, p2 in edges:
        mid_point = (p1 + p2) / 2
        length = np.linalg.norm(p2 - p1)
        plt.text(mid_point[0], mid_point[1], f'{length:.4f}', fontsize=10, ha='center', va='bottom', color='black')

        # 计算并显示对角线长度
        diagonals = [
            (vertices[2], bottom_left_point),
            (vertices[1], bottom_right_point),
            (vertices[4], vertices[6]),
            (vertices[5], vertices[7])
        ]

        for p1, p2 in diagonals:
            mid_point = (p1 + p2) / 2
            length = np.linalg.norm(p2 - p1)
            plt.text(mid_point[0], mid_point[1], f'{length:.4f}', fontsize=12, ha='center', va='bottom', color='black')

    # 添加标签和标题
    plt.title('Point Cloud Pattern')
    plt.legend()
    plt.axis('equal')
    plt.show()

# ============= Match scan points and CAD points =============
def cal_CADandPointCloud_diff(value_from_PointCloud, value_from_CAD):

    # 指定要计算差值的键
    keys_to_compare = [('windows_long_11', 'windows_long_11'), ('windows_long_10', 'windows_long_10'),
                    ('windows_width_11', 'windows_width_11'),('windows_width_10', 'windows_width_10'),
                    ('wall_long_11', 'wall_long_11'),('wall_long_10', 'wall_long_10'),
                    ('wall_width_11', 'wall_width_11'),('wall_width_10', 'wall_width_10')]
    # 存储差值的字典
    diff_dict = {}
    # 计算差值并存储到新字典中
    for key_pc, key_cad in keys_to_compare:
        if key_pc in value_from_PointCloud and key_cad in value_from_CAD:
            diff = round(abs(value_from_PointCloud[key_pc] - value_from_CAD[key_cad]), 2)
            diff_dict[f"{key_pc}"] = diff
    diff_dict['wall_diagonal_diff'] = value_from_PointCloud['wall_diagonal_diff']
    diff_dict['window_diagonal_diff'] = value_from_PointCloud['window_diagonal_diff']

    return diff_dict

# scan_points = np.array(scan_points)
# cad_points = np.array(cad_points)
# ic(scan_points.shape, cad_points.shape)
# ic(scan_points)
# ic(cad_points)
# distances, indices = nearest_neighbor(cad_points, scan_points)
# plot_error(cad_points, scan_points[indices], cad_lines)
# T,_,_ = best_fit_transform(cad_points, scan_points[indices])
# print(T)

# =================数据写入表格====================
def editExcel(excel_template_path, save_result_folder, value_from_PointCloud, para_result):
    excel_template_path = Path(excel_template_path)
    excel_write_path = Path(save_result_folder) / "result.xlsx"
    # copy excel template to result path using shutil
    ic(excel_template_path)
    ic(excel_write_path)
    shutil.copy(excel_template_path, excel_write_path)

    try:
        workbook = openpyxl.load_workbook(excel_write_path)
    except FileNotFoundError:
        workbook = openpyxl.Workbook()

    # 选择工作表
    sheet = workbook.active
    sheet.title = 'Sheet1'

    # 清空D11-D24单元格内容
    for row in range(11, 25):
        sheet[f'D{row}'] = ''

    # 定义单元格映射
    cell_mapping = {
        'windows_long_10': 'D11',
        # 'windows_long_11': 'D12',
        'windows_width_10': 'D12',
        # 'windows_width_11': 'D14',
        # 'window_area': 'D15',
        'window_diagonal_diff': 'D13',
        'wall_long_10': 'D17',
        # 'wall_long_11': 'D20',
        'wall_width_10': 'D18',
        # 'wall_width_11': 'D22',
        # 'wall_area': 'D24',
        'wall_diagonal_diff': 'D19',
        # 'parallel_result': 'D20',
    }

    # 写入值到对应单元格
    for key, cell in cell_mapping.items():
        value = value_from_PointCloud.get(key)
        if value is not None:
            sheet[cell] = value
        elif key == 'parallel_result':
            sheet[cell] = para_result  # 写入平行线结果
            # 设置单元格格式
            sheet[cell].alignment = Alignment(vertical="center", horizontal="center", wrap_text=True)
            sheet[cell].font = Font(name="宋体", size=7, bold=True)

    # 保存文件
    workbook.save(excel_write_path)
    print("数据已成功写入 Excel 文件。")

# ============= export to json =============
import json
def Export2Json(value_from_CAD, value_from_PointCloud, diff_dict, save_result_folder):
    save_result_folder = Path(save_result_folder)
    with open(save_result_folder / "results.json", 'w') as file:
        json.dump({
            'value_from_CAD': value_from_CAD,
            'value_from_PointCloud': value_from_PointCloud,
            'diff_dict': diff_dict
        }, file, indent=4)


def run_boundary_fitting(pcd: o3d.geometry.PointCloud, dxf_path, excel_template_path, save_result_folder,
                         vis = False):
    save_result_folder = Path(save_result_folder)
    # ============= Get CAD points and lines =============
    dxf = ezdxf.readfile(dxf_path)

    msp = dxf.modelspace()
    layers = ['windows_long_10', 'wall_long_10', 'wall_long_11', 
            'wall_width_10', 'wall_width_11', 'windows_width_10', 
            'windows_long_11', 'windows_width_11', 'windows_diagnol_11', 
            'windows_diagnol_10','wall_diagnol_11', 'wall_diagnol_10']
    cad_points, cad_lines = process_dxf_file(msp, layers, False)

    scan_pc = pcd

   #===========不用旋转点云时注释掉这三行=============
    points = np.asarray(scan_pc.points)
    pcd_path = rotate_point_cloud(points, 90)  # 顺时针旋转90°
    scan_pc.points = o3d.utility.Vector3dVector(pcd_path)
    #===========不用旋转点云时注释掉这三行=============

    print(value_from_CAD)
    # ============= Use PCL pybind to segment point cloud and fit lines =============
    pcd_dwn = scan_pc.voxel_down_sample(voxel_size=0.005)
    # pcd_dwn=scan_pc
    orthogonal_pcd = get_plane(pcd_dwn)
    points = np.asarray(orthogonal_pcd.points)
    BF = BoundaryFitting.BoundaryFitting(0.04, 0.04, 600, 0.01)
    lines = BF.fitting(points, points.shape[0], vis)

    lines = np.array(lines)
    lines_func = np.zeros([len(lines), 3])
    lines_func[:, 0] = lines[:, 3]
    lines_func[:, 1] = -lines[:, 2]
    lines_func[:, 2] = lines[:, 0] * lines[:, 3] - lines[:, 1] * lines[:, 2]
    edges = lines[:, [[4, 5], [6, 7]]]
    # scan_points, scan_lines = draw_lines(lines_func, edges)
    ic(lines_func)
    ic(edges)
    
    # plot all edges
    # for edge in edges:
    #     plt.plot(edge[:, 0], edge[:, 1], 'b-')
    #     # show each edge's length
    #     plt.text(edge[0][0], edge[0][1], f'{np.linalg.norm(edge[0] - edge[1]):.4f}', fontsize=10, ha='center', va='bottom', color='black')
    # plt.show()

    horizontal_lines, vertical_lines, sort_V_byXcoord, sort_H_byYcoord = calssify_lines(lines_func, edges)    

    # ============= 计算门窗高宽 ============
    if len(horizontal_lines) == 4 and len(vertical_lines) == 4:
        print("数据检测完全")
        para_result = determine_parallel_lines(sort_V_byXcoord)
        vertices = cal_intersections(lines_func)
        ic(vertices)

        bottom_right_idx = np.argmin(sort_V_byXcoord[3]['edge'][:, 1])
        bottom_right_point = sort_V_byXcoord[3]['edge'][bottom_right_idx]
        
        bottom_left_idx = np.argmin(sort_V_byXcoord[0]['edge'][:, 1])
        bottom_left_point = sort_V_byXcoord[0]['edge'][bottom_left_idx]
        
        wall_long_10 = round(np.linalg.norm(bottom_right_point - bottom_left_point)*1000 ,2)
        wall_long_11 = round(np.linalg.norm(vertices[1] - vertices[2])*1000, 2)
        wall_width_10 = round(np.linalg.norm(bottom_left_point - vertices[1])*1000, 2)
        wall_width_11 = round(np.linalg.norm(bottom_right_point - vertices[2])*1000, 2)

        windows_long_10 = round(np.linalg.norm(vertices[4] - vertices[7])*1000, 2)
        windows_long_11 = round(np.linalg.norm(vertices[5] - vertices[6])*1000, 2)
        windows_width_10 = round(np.linalg.norm(vertices[5] - vertices[4])*1000, 2)
        windows_width_11 = round(np.linalg.norm(vertices[6] - vertices[7])*1000, 2)

        wall_diagonal_diff, window_diagonal_diff = cal_diagonal_diff(vertices, bottom_left_point, bottom_right_point)
    else:
        print("数据检测缺失")
        para_result = None

        wall_long_10 = None
        wall_long_11 = None
        wall_width_10 = None
        wall_width_11 = None

        windows_long_10 = None
        windows_long_11 = None
        windows_width_10 = None
        windows_width_11 = None
        
        wall_diagonal_diff = None
        window_diagonal_diff = None

    value_from_PointCloud = {
        'wall_diagonal_diff': wall_diagonal_diff,  #23D
        'window_diagonal_diff': window_diagonal_diff, #15D
        # 'wall_area': wall_area,
        # 'window_area': window_area,

        'windows_long_10': windows_long_10, #11D
        'windows_long_11': windows_long_11, #12D
        'windows_width_10': windows_width_10, #13D
        'windows_width_11': windows_width_11, #14D

        'wall_long_10': wall_long_10, #19D
        'wall_long_11': wall_long_11, #20D
        'wall_width_10': wall_width_10, #21D
        'wall_width_11': wall_width_11 #22D
    }

    if vis: 
        plot_PointCloud_line(vertices, bottom_left_point, bottom_right_point)
        plot_cad(cad_points, cad_lines)

    diff_dict = cal_CADandPointCloud_diff(value_from_PointCloud, value_from_CAD)
    editExcel(excel_template_path, save_result_folder, value_from_PointCloud, para_result)
    Export2Json(value_from_CAD, value_from_PointCloud, diff_dict, save_result_folder)

    img, transform_matrix = convert_pcd_to_2d_image(scan_pc)

    # line that contain start and end point of
    all_lines = np.array([
        [bottom_right_point, bottom_left_point],  # wall_long_10
        [vertices[1], vertices[2]],  # wall_long_11
        [bottom_left_point, vertices[1]],  # wall_width_10
        [bottom_right_point, vertices[2]],  # wall_width_11
        [vertices[4], vertices[7]],  # windows_long_10
        [vertices[5], vertices[6]],  # windows_long_11
        [vertices[5], vertices[4]],  # windows_width_10
        [vertices[6], vertices[7]]  # windows_width_11
    ])
    img = plot_skeleton_on_image(img, transform_matrix, all_lines)
    img = padding_img_to_ratio_3_2(img)
    cv2.imwrite(str(save_result_folder / "preview.png"), img)

if __name__ == '__main__':
    pcd_path =  r"D:\Inspection_data\20241017014\pcd_surface.ply"
    dxf_path = r"./data/qiangbanzhanshi.dxf"
    excel_template_path = './data/result_a6_vertical.xlsx'
    save_result_folder = r"D:\Inspection_data"

    pcd = o3d.io.read_point_cloud(pcd_path)
    run_boundary_fitting(pcd, dxf_path, excel_template_path, save_result_folder)