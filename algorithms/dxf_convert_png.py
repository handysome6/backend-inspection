import cv2
from ezdxf import recover
from ezdxf.addons.drawing import matplotlib
from .utils import padding_img_to_ratio_3_2

def export_dark_bg(dxf_path: str) -> str:
    """
    导出带有黑色背景的png图像
    """
    path = ".temp.png"
    # Exception handling left out for compactness:
    doc, auditor = recover.readfile(dxf_path)

    if auditor.has_errors:
        raise Exception(auditor.errors)
    matplotlib.qsave(doc.modelspace(), path)
    
    img = cv2.imread(path)
    output_img = padding_img_to_ratio_3_2(img)
    cv2.imwrite(path, output_img)
    return path

if __name__ == '__main__':
    export_dark_bg(r'Data/test.dxf')