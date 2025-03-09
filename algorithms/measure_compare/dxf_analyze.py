import ezdxf
from typing import List
import numpy as np
from icecream import ic
from loguru import logger
from .main_dxf_wall_points import DimensionEntity



# 处理dxf文件，获取各个图层的元素和信息
def process_dxf_file(fMsp, required_layers: List[str], read_all: bool):
    value_from_CAD = {}

    group: dict = fMsp.groupby(dxfattrib='layer')
    print("该文件包含的图层: ", group.keys())
    print(">>>>>>>>>>>")

    if read_all:
        required_layers = list(group.keys())

    for layer_name in required_layers:
        if layer_name not in group:
            logger.warning(f"图层 {layer_name} 不存在")
            continue
        layer_to_read = group[layer_name]
        value_from_CAD[layer_name] = []

        for entity in layer_to_read:
            if entity.dxftype() == 'DIMENSION':
                dim = DimensionEntity(entity)
                start, end = dim.get_defpoints()[:, :2]

                value_from_CAD[layer_name].append({
                    'start': start, 
                    'end': end, 
                    # ROUNDING LENGTH TO 2 DECIMAL PLACES
                    'length': round(np.linalg.norm(end - start) * 100) / 100 
                })



    ic(value_from_CAD)
    return value_from_CAD



def calculate_dimensions(value_from_CAD):
    dimensions = {}
    vertical_lines = []
    horizontal_lines = []

    for layer_name, lines in value_from_CAD.items():
        if not isinstance(lines, list):
            print(f"警告: {layer_name} 的数据格式不正确，跳过此层.")
            continue

        dimensions[layer_name] = {
            'width_top': 0,
            'width_bottom': 0,
            'height_left': 0,
            'height_right': 0,
            'diagonal_top': 0,
            'diagonal_bottom': 0,
            'diagonal_diff': 0,
            'lengths': []
        }

        for data in lines:
            start = data.get('start')
            end = data.get('end')
            length = data.get('length')
            if start is None or end is None or length is None:
                print(f"警告: {layer_name} 中缺少起点、终点或长度信息，跳过.")
                continue

            # 添加线条长度到列表
            dimensions[layer_name]['lengths'].append(length)
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            if dx != 0:
                slope = dy / dx
                angle = np.degrees(np.arctan(slope))
            else:
                angle = 90 if dy > 0 else -90

            # 分类线条
            if abs(angle) >= 85:
                vertical_lines.append(data)
            elif abs(angle) <= 5:
                horizontal_lines.append(data)
            else:
                if angle > 0:
                    dimensions[layer_name]['diagonal_top'] = length
                else:
                    dimensions[layer_name]['diagonal_bottom'] = length

        # 根据起点的 y 值对水平线进行排序
        sorted_horizontal_lines = sorted(horizontal_lines, key=lambda x: x['start'][1])
        if len(sorted_horizontal_lines) > 0:
            dimensions[layer_name]['width_bottom'] = sorted_horizontal_lines[0]['length']
        if len(sorted_horizontal_lines) > 1:
            dimensions[layer_name]['width_top'] = sorted_horizontal_lines[1]['length']

        # 根据起点的 x 值对垂直线进行排序
        sorted_vertical_lines = sorted(vertical_lines, key=lambda x: x['start'][0])
        if len(sorted_vertical_lines) > 0:
            dimensions[layer_name]['height_left'] = sorted_vertical_lines[0]['length']
        if len(sorted_vertical_lines) > 1:
            dimensions[layer_name]['height_right'] = sorted_vertical_lines[1]['length']

        # 计算斜率差
        if dimensions[layer_name]['diagonal_top'] > 0 and dimensions[layer_name]['diagonal_bottom'] > 0:
            dimensions[layer_name]['diagonal_diff'] = abs(dimensions[layer_name]['diagonal_top'] - dimensions[layer_name]['diagonal_bottom'])

    return dimensions



def dxf_analyze(cad_model_path):
    dxf = ezdxf.readfile(cad_model_path)
    msp = dxf.modelspace()
    # 提取所有标注图层名称
    annotation_layers = set()
    for entity in msp:
        if entity.dxftype() in ['TEXT', 'MTEXT', 'DIMENSION']:
            annotation_layers.add(entity.dxf.layer)

    annotation_layers = list(annotation_layers)  # 转换为列表

    value_from_CAD = process_dxf_file(msp, annotation_layers, False)
    # plot_cad(cad_points, cad_lines)

    # 存储维度信息
    dimensions = {}
    for layer in annotation_layers:
        layer_dimensions = calculate_dimensions({layer: value_from_CAD[layer]})
        if layer_dimensions:
            for key, value in layer_dimensions[layer].items():
                dimensions[f"{layer}_{key}"] = value
    print("Dimensions:")
    for key, value in dimensions.items():
        ic(key, value)

    return dimensions, value_from_CAD


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    cad_model_path = r"Z:\data\Project_Hebei\20250103\cad\20241211001.dxf"
    dimensions = dxf_analyze(cad_model_path)
    print("Dimensions:")
    for key, value in dimensions.items():
        ic(key, value)
