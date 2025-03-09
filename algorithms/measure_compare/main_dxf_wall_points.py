import ezdxf
import numpy as np
from icecream import ic
from matplotlib import pyplot as plt

np.set_printoptions(precision=2, suppress=True)

class CustomPointSet(list):
    """用于存储和管理点集的自定义类。

    继承自Python内置的list类, 提供了添加点、搜索点和查找交点等功能。
    使用近似相等(误差<1)来判断点是否重合。
    """
    def __init__(self):
        super().__init__()

    def add_point(self, point):
        """
        return:
            ret: 0 if not duplicate
                 1 if duplicate
            idx: new point index if not duplicate, old point index if duplicate
        """
        idx = self.search_point(point)
        if idx == -1:
            # add point if not exist
            self.append(point)
            new_idx = len(self) - 1
            return 0, new_idx
        else:
            return 1, idx

    def search_point(self, point):
        for i in range(len(self)):
            if abs(self[i][0] - point[0]) < 1 and abs(self[i][1] - point[1]) < 0.8:
                return i
        return -1

    def get_intersection_points(self, points):
        intersection_points = []
        for point in points:
            idx = self.search_point(point)
            if idx != -1:
                intersection_points.append(point)
        return intersection_points


class DimensionEntity:
    """表示DXF文件中的一个标注实体。

    属性:
        dim: ezdxf标注实体对象
        measurement: 标注的测量值
        lines: 标注中的线段列表
        line_endpoints: 线段端点坐标列表
        points: 标注中的点坐标列表
        text: 标注文本列表
    """
    def __init__(self, dim):
        self.dim = dim
        self.measurement = dim.get_measurement()
        self.lines = []
        self.line_endpoints = []
        self.points = []
        self.text = []

    def get_text_measurement(self):
        for entity in self.dim.virtual_entities():
            if entity.dxftype() == 'MTEXT':
                return float(entity.dxf.text)
        return None

    def parse_dimension_entity(self):
        for entity in self.dim.virtual_entities():
            if entity.dxftype() == 'LINE':
                self.lines.append(entity)
                self.line_endpoints += [entity.dxf.start, entity.dxf.end]
            elif entity.dxftype() == 'MTEXT':
                self.text.append(entity)
            elif entity.dxftype() == 'POINT':
                self.points.append(entity.dxf.location)
        # self.line_endpoints = np.array(self.line_endpoints)
        # self.points = np.array(self.points)
        
        # minim = np.array(line_endpoints).min(axis=0)
        # line_endpoints -= minim
        # point_locations -= minim

    def plot(self):
        # convert to numpy array
        self.line_endpoints = np.array(self.line_endpoints)
        self.points = np.array(self.points)

        # new plot
        fig = plt.figure(num=1, clear=True)
        ax = fig.add_subplot()
        # plot lines
        for line in self.lines:
            ax.plot([line.dxf.start[0], line.dxf.end[0]], [line.dxf.start[1], line.dxf.end[1]], color='red')
        # plot points
        ax.scatter(self.line_endpoints[:, 0], self.line_endpoints[:, 1], color='red')
        ax.scatter(self.points[:, 0], self.points[:, 1], color='blue')
        # set xy equal scale
        ax.axis('equal')
        plt.show()

    def get_start_end_points(self):
        """
        return:
            start_end_points: 2D array, shape (2, 2)
                start_end_points[0]: start point
                start_end_points[1]: end point
        """
        if len(self.line_endpoints) == 0:
            self.parse_dimension_entity()
        
        # remove duplicate points
        set_line_endpoints = CustomPointSet()
        for point in self.line_endpoints:
            ret, idx = set_line_endpoints.add_point(point)
            if ret:
                # remove idx-th point if exist
                set_line_endpoints.pop(idx)

        # get intersection points with points
        intersection_points = set_line_endpoints.get_intersection_points(self.points)
        intersection_points = np.array(intersection_points)
        ic(intersection_points)
        assert len(intersection_points) == 2, "The result should be 2 points: start and end"

        print("Dimension Start: ", intersection_points[0], "End: ", intersection_points[1])
        return intersection_points

    def get_defpoints(self):
        """
        Get the start and end points of the dimension using defpoints.
        """
        a = self.dim.dxf.defpoint3
        b = self.dim.dxf.defpoint2
        return np.array([a, b])

def get_wall_points(dxf_path: str):
    """
    return:
        wall_points: 2D array, shape (N, 2)
    """
    dxf = ezdxf.readfile(dxf_path)
    msp = dxf.modelspace()

    group: dict = msp.groupby(dxfattrib='layer')
    print("该文件包含的图层: ", group.keys())
    print(">>>>>>>>>>>")

    if "wall" not in group:
        print("wall layer not found")
        return None

    layer_to_read = group["wall"]

    all_points = []
    for entity in layer_to_read:
        if entity.dxftype() == 'DIMENSION':
            dim = DimensionEntity(entity)
            # dim.parse_dimension_entity()
            # dim.plot()
            # all_points.extend(dim.get_start_end_points())
            all_points.extend(dim.get_defpoints())

    # remove duplicate wall points
    set_all_points = CustomPointSet()
    for point in all_points:
        set_all_points.add_point(point)
    print("found wall points number: ", len(set_all_points))

    return np.array(set_all_points)


if __name__ == "__main__":
    wall_points = get_wall_points(r"C:\Users\FENGZiyue\test\data\1212\9J4\9F-CMC-J#-4.dxf")
    ic(wall_points)
    # show wall points
    plt.scatter(wall_points[:, 0], wall_points[:, 1], color='blue')
    plt.axis('equal')
    plt.show()
