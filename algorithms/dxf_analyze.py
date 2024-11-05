# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import math
import ezdxf
from typing import List
from matplotlib import pyplot as plt
import numpy as np
from icecream import ic
from itertools import combinations

def add_info(obj):
        return f"{obj}"

ic.configureOutput(argToStringFunction=add_info)

# 定义保存3d point点的信息
class myPoint:
    _x: float
    _y: float
    _z: float

    def __init__(self, x: float, y: float, z: float):
        self._x = x
        self._y = y
        self._z = z

    def length_to(self, point2):
        point1 = self
        point2 = point2
        a = np.array([point1._x, point1._y, point1._z])
        b = np.array([point2._x, point2._y, point2._z])
        return np.linalg.norm(a-b)
    
    def to_np(self):
        return np.array([self._x, self._y, self._y])
    
    def getPoint(self):
        return self._x, self._y, self._z

    def getx(self):
        return self._x

    def gety(self):
        return self._y

    def getz(self):
        return self._z
    
    def __str__(self):
        return f"[{self._x}, {self._y}, {self._z}]"


# 定义保存2d point点的信息
class my2DPoint:
    _x: float
    _y: float

    def __init__(self, x: float, y: float):
        self._x = x
        self._y = y

    def to_np(self):
        return np.array([self._x, self._y])

    def getPoint(self):
        return self._x, self._y

    def getx(self):
        return self._x

    def gety(self):
        return self._y

    def __str__(self):
        return f"[{self._x}, {self._y}]"

# 定义保存2D线的信息
class my2DLine:

    def __init__(self, start_point: my2DPoint, end_point: my2DPoint):
        _start_point: my2DPoint
        _end_point: my2DPoint
        self._start_point = start_point
        self._end_point = end_point

    def getLineStart(self):
        return  self._start_point

    def getLineEnd(self):
        return self._end_point
    
    def __str__(self):
        return f"{self._start_point} - {self._end_point}"


# 定义保存线的信息
class myLine:
    _start_point: myPoint
    _end_point: myPoint
    _length: float

    def __init__(self, start_point: myPoint, end_point: myPoint):
        self._start_point = start_point
        self._end_point = end_point
        x1, y1, z1 = start_point.getPoint()
        x2, y2, z2 = end_point.getPoint()
        self._length = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

    def getLine(self):
        return self._start_point, self._end_point

    def getLineStart(self):
        return  self._start_point

    def getLineEnd(self):
        return self._end_point

    def getLineLength(self):
        return self._length
    
    def __str__(self):
        return f"{self._start_point} - {self._end_point}"


class CustomLineSet:
    """
    Storing a set of lines in a way of:
        - points: list of 2D points
        - lines: list of points index
    Suportting:
        - add line
        - add point
        - search point by coord value
        - plot all points and lines
    """
    def __init__(self):
        self.points = []
        self.lines = []

    def add_line(self, start_point, end_point):
        start_index = self.search_point(start_point)
        end_index = self.search_point(end_point)
        if start_index == -1:
            self.points.append(start_point)
            start_index = len(self.points) - 1
        if end_index == -1:
            self.points.append(end_point)
            end_index = len(self.points) - 1
        self.lines.append([start_index, end_index])

    def add_point(self, point):
        if self.search_point(point) == -1:
            self.points.append(point)

    def search_point(self, coord):
        for i in range(len(self.points)):
            if abs(self.points[i][0] - coord[0]) < 0.001 and abs(self.points[i][1] - coord[1]) < 0.001:
                return i
        return -1
    
    def plot(self):
        plt.figure(figsize=(8,8))
        plt.scatter(np.array(self.points)[:,0], np.array(self.points)[:,1], color='blue')
        for i in range(len(self.lines)):
            p1 = np.array(self.points[self.lines[i][0]])
            p2 = np.array(self.points[self.lines[i][1]])
            dist = np.linalg.norm(p1-p2)
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='red')
            midpoint = (p1+p2)/2
            plt.text(midpoint[0], midpoint[1], f'{dist:.4f}', fontsize=10, color='gray')
        plt.axis('equal')
        plt.show()

    def __str__(self) -> str:
        return f'{self.points}\n{self.lines}'
        

# 处理dxf文件，获取各个图层的元素和信息
def process_dxf_file(fMsp, required_layers: List[str], read_all: bool):
    group: dict = fMsp.groupby(dxfattrib='layer')
    print("该文件包含的图层: ", group.keys())
    print(">>>>>>>>>>>")
    # 待读取待图层
    layers_to_read = []
    # 保存所有测量的线
    rLines = []
    # 按顺序保存点位
    rPoints = []

    # 如果全部读
    if read_all:
        required_layers = list(group.keys())
    for layer_name in required_layers:
        if layer_name in group:
            layers_to_read.append((layer_name, group[layer_name]))
    # 逐层读取图层
    while len(layers_to_read) > 0:
        next_layer_name, next_layer = layers_to_read[0]
        
        new_layer, layer_len, layer_line,CAD_value = parse_layer(next_layer, next_layer_name)        
        
        layers_to_read.pop(0)
        layers_to_read += new_layer
        if len(layer_line) != 0:
            line1, matched = getLinePoint(layer_line, layer_len, CAD_value)            
            if matched:
                rLines.append(line1)

    for l in rLines:
        l: my2DLine
        # change unit from mm to m
        l.getLineStart()._x = l.getLineStart().getx()/1000
        l.getLineStart()._y = l.getLineStart().gety()/1000
        l.getLineEnd()._x = l.getLineEnd().getx()/1000
        l.getLineEnd()._y = l.getLineEnd().gety()/1000

    lineset = CustomLineSet()
    for l in rLines:
        l: my2DLine
        lineset.add_line(l.getLineStart().getPoint(), l.getLineEnd().getPoint())
    
    ic(lineset.points)
    ic(lineset.lines)
    # lineset.plot()
    return lineset.points, lineset.lines


    # saved_points = []
    # saved_lines = []
    # threshould = 1
    # for line in rLines:
    #     start_point = [line.getLineStart().getx()/1000, line.getLineStart().gety()/1000]
    #     end_point = [line.getLineEnd().getx()/1000, line.getLineEnd().gety()/1000]
    #     start_index = -1
    #     end_index = -1
    #     for i in range(len(saved_points)):
    #         if abs(saved_points[i][0] - start_point[0])<threshould and abs(saved_points[i][1] - start_point[1]) < threshould:
    #             start_index = i
    #         if abs(saved_points[i][0] - end_point[0])<threshould and abs(saved_points[i][1] - end_point[1]) < threshould:
    #             end_index = i
    #     if start_index == -1:
    #         saved_points.append(start_point)
    #         start_index = len(saved_points)-1
    #     if end_index == -1:
    #         saved_points.append(end_point)
    #         end_index = len(saved_points)-1

    #     saved_lines.append([start_index, end_index])
        
    # print("---------------------------------------------------")
    # print(saved_points)
    # print(saved_lines)
    # print("---------------------------------------------------")
    # return saved_points, saved_lines

    # rPoints.append([rLines[0].getLineStart().getx(), rLines[0].getLineStart().gety()])
    # tPointy = rLines[0].getLineEnd()
    # for i in range(len(rLines)):
    #     print("i_point:",i,'-',tPointy.getx(),'-',tPointy.gety())
    #     for line in rLines:
    #         print("i:", i,"line:",line.getLineStart().getx(),'-',line.getLineStart().gety())
    #         if abs(tPointy.getx() - line.getLineStart().getx())<100 and abs(tPointy.gety() - line.getLineStart().gety())<100:
    #             tPointy = line.getLineEnd()
    #             rPoints.append([line.getLineStart().getx(), line.getLineStart().gety()])
    #             break
    #         elif abs(tPointy.getx() - line.getLineEnd().getx())<100 and abs(tPointy.gety() - line.getLineEnd().gety())<100:
    #             tPointy = line.getLineStart()
    #             rPoints.append([line.getLineEnd().getx(), line.getLineEnd().gety()])
    #             break
    # return rPoints
    
value_from_CAD = {}
# 处理在每一个标注层中获取的线数据
def getLinePoint(f_line: List[myLine], f_len, CAD_value):
    all_points = []
    for i in f_line:
        all_points.append(i.getLineStart())
        all_points.append(i.getLineEnd())

    # filter overlap / too close points
    combins = [c for c in  combinations(all_points, 2)]
    for i, j in combins:
            length = i.length_to(j)
            if length < 5:
                all_points.remove(i)
                all_points.remove(j)

    # check if any two points' distance == texted
    combins = [c for c in  combinations(all_points, 2)]
    success = False
    for i, j in combins:
            mLen = i.length_to(j)
            mLen = round(mLen*100)/100
            if abs(f_len-mLen) < 5:
                success = True
                start, end = i, j
                break
    rPoint1 = my2DPoint(i.getx(), i.gety())
    rPoint2 = my2DPoint(j.getx(), j.gety())
    rLine = my2DLine(rPoint1, rPoint2)
        
    # 存储 CAD_value 和 mLen 的对应关系到字典中
    value_from_CAD[CAD_value] = mLen

    if success:
        print("长度匹配成功:textLen--", f_len, "countLen--", mLen)
        ic(start, end)
        
        return rLine, True
    else:
        print("长度匹配失败:textLen--", f_len)
        return rLine, False
    

# 处理其中一个图层
def parse_layer(layer, layer_name: str):
    # print("layerName:",layer_name)
    new_layer_list = []
    noteLen = 0
    noteLine = []
    notePoint = []
    for new_element in layer:
        if new_element.dxftype() == 'INSERT':
            try:
                new_msp = new_element.explode().groupby('layer')
            except:
                continue
            for new_layer_name in new_msp.keys():
                new_layer_list.append((new_layer_name, new_msp[new_layer_name]))
        # 如果该元素类型为 LWPOLYLINE
        elif new_element.dxftype() == 'LWPOLYLINE':
            point_list = list(new_element.vertices_in_wcs())
            for point in point_list:
                print("(", point[0], ",", point[1], ")")
        # 如果类型为point
        elif new_element.dxftype() == 'POINT':
            # print("layer:", layer_name, "-Point:(", new_element.dxf.location[0], ",", new_element.dxf.location[1], ")")
            tPoint = myPoint(new_element.dxf.location[0], new_element.dxf.location[1], new_element.dxf.location[2])
            notePoint.append(tPoint)
        # 如果该元素的类型为 LINE
        elif new_element.dxftype() == 'LINE':
            print("layer:", layer_name, "-Line-start:(", new_element.dxf.start[0], ",", new_element.dxf.start[1], ",", new_element.dxf.start[2], ")",
                  "-Line-end:(", new_element.dxf.end[0], ",", new_element.dxf.end[1], ",", new_element.dxf.start[2], ")")
            # import pdb; pdb.set_trace()
            startPoint = myPoint(new_element.dxf.start[0], new_element.dxf.start[1], new_element.dxf.start[2])
            endPoint = myPoint(new_element.dxf.end[0], new_element.dxf.end[1], new_element.dxf.end[2])
            tLine = myLine(startPoint, endPoint)
            noteLine.append(tLine)
        elif new_element.dxftype() == 'TEXT' or new_element.dxftype() == 'MTEXT':
            if new_element.dxftype() == 'TEXT':
                content_text: str = new_element.dxf.text
            else:
                content_text: str = new_element.plain_text()
            print("layer:", layer_name, "-TEXT:", content_text)
            noteLen = float(content_text)
        elif new_element.dxftype() == "DIMENSION":
            # 炸开按图层名归类
            try:
                new_msp = new_element.explode().groupby('layer')
            except Exception as _:
                continue
            for new_layer_name in new_msp.keys():
                new_layer_list.append((new_layer_name, new_msp[new_layer_name]))
        else:
            print("read_dxf_file(): 暂不支持该格式: ", new_element.dxftype())
    return new_layer_list, noteLen, noteLine, layer_name

def plot_cad(points, connections):
    plt.figure(figsize=(8,8))
    plt.scatter(np.array(points)[:,0], np.array(points)[:,1], color='blue')
    for i in range(len(connections)):
        p1 = np.array(points[connections[i][0]])
        p2 = np.array(points[connections[i][1]])
        dist = np.linalg.norm(p1-p2)
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='red')
        midpoint = (p1+p2)/2
        plt.text(midpoint[0], midpoint[1], f'{dist:.4f}', fontsize=10, color='gray')
    plt.axis('equal')
    plt.show()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    dxf = ezdxf.readfile('qiangbanzhanshi.dxf')
    msp = dxf.modelspace()
    layers = ['windows_long_10', 'wall_long_10', 'wall_long_11', 
            'wall_width_10', 'wall_width_11', 'windows_width_10', 
            'windows_long_11', 'windows_width_11', 'windows_diagnol_11', 
            'windows_diagnol_10','wall_diagnol_11', 'wall_diagnol_10']
    cad_points, cad_lines = process_dxf_file(msp, layers, False)
    plot_cad(cad_points, cad_lines)
    # 计算对角线差
    windows_diagnol_diff = abs(value_from_CAD['windows_diagnol_11'] - value_from_CAD['windows_diagnol_10'])
    wall_diagnol_diff = abs(value_from_CAD['wall_diagnol_11'] - value_from_CAD['wall_diagnol_10'])
    value_from_CAD['windows_diagnol_diff'] = windows_diagnol_diff
    value_from_CAD['wall_diagnol_diff'] = wall_diagnol_diff
    print(value_from_CAD)
    