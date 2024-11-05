import numpy as np
import cv2 as cv
from icecream import ic
from typing import List

AVG_INTENS_THRES = 50
BINARY_FILTER_THRES = 200


def crop_square(image:np.ndarray, cx, cy, r):
    """
    Crops a square of size (2*r+1) centered at (cx, cy) from the given image.

    Args:
        image: A numpy.ndarray representing the image.
        cx: The x-coordinate of the center of the crop.
        cy: The y-coordinate of the center of the crop.
        r: The radius of the crop.

    Returns:
        cropped_image: A numpy.ndarray representing the cropped image.
        (x1, y1): crop_start
    """
    # Get the shape of the image
    height, width = image.shape[:2]

    # Calculate the top-left corner of the crop
    x1 = max(cx - r, 0)
    y1 = max(cy - r, 0)

    # Calculate the bottom-right corner of the crop
    x2 = min(cx + r, width - 1)
    y2 = min(cy + r, height - 1)

    # Crop the image
    cropped_image = image[y1:y2, x1:x2].copy()

    return cropped_image, (x1, y1)

class Ellipse:
    def __init__(self, center, axes, angle) -> None:
        self.center = center
        self.axes = axes
        self.angle = angle

class CircleEdgeDrawDetector:
    def __init__(self, img:np.ndarray=None, filename="") -> None:
        if img is not None:
            self.src = img.copy()
        else:
            self.src = cv.imread(filename)
        if self.src is None:
            raise Exception("src image is None")
        
        self._init_ed_detector()

    def _init_ed_detector(self):
        self.ed = cv.ximgproc.createEdgeDrawing()

        # you can change parameters (refer the documentation to see all parameters)
        EDParams = cv.ximgproc_EdgeDrawing_Params()
        EDParams.MinPathLength = 20     # try changing this value between 5 to 1000
        EDParams.PFmode = False         # defaut value try to swich it to True
        EDParams.MinLineLength = 10     # try changing this value between 5 to 100
        EDParams.NFAValidation = True   # defaut value try to swich it to False
        EDParams.EdgeDetectionOperator = 3

        self.ed.setParams(EDParams)

    def _detect_ellipse(self):

        src = self.src
        shape = src.shape
        if len(shape) == 2 or shape[-1] == 1:
            gray = src
        else:
            gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

        # Detect edges
        # you should call this before detectLines() and detectEllipses()
        self.ed.detectEdges(gray)
        ellipses = self.ed.detectEllipses()

        ellipse_list: List = []

        #Draw detected circles and ellipses
        if ellipses is not None: # Check if circles and ellipses have been found and only then iterate over these and add them to the image
            for i in range(len(ellipses)):
                # get ellipse 
                center = np.array([ellipses[i][0][0], ellipses[i][0][1]])
                axes = (int(ellipses[i][0][2])+int(ellipses[i][0][3]),int(ellipses[i][0][2])+int(ellipses[i][0][4]))
                angle = ellipses[i][0][5]
                color = (255, 255, 255)

                crop_r = max(*axes) + 10
                center_int = center.astype(np.int32)
                cropped_img, crop_start = crop_square(gray, *center_int, crop_r)
                cropped_center = center_int - crop_start

                mask = np.zeros_like(cropped_img)
                mask = cv.ellipse(mask, cropped_center, axes, angle, 0, 360, color, thickness=-1)
                masked_src = cv.bitwise_and(cropped_img, mask)
                # cv.imshow("test1", masked_src)
                # cv.waitKey(0)

                area = np.pi * axes[0] * axes[1]
                if np.sum(masked_src)/area > AVG_INTENS_THRES:
                    # print(np.sum(masked_src)/area)
                    ellipse_list.append(ellipses[i])

        return ellipse_list


if __name__ == "__main__":
    img = cv.imread(r"C:\workspace\data\2.85m\2.85m_1\1\img\Image.png")
    detector = CircleEdgeDrawDetector(img)
    ellipse_list = detector._detect_ellipse()
    from pprint import pprint
    pprint(ellipse_list)
    # cv.namedWindow("result", cv.WINDOW_NORMAL)
    # cv.imshow("result", img)
    # cv.waitKey(0)