import cloudComPy as cc
cc.initCC()
cc.initCloudCompare()
from icecream import ic
import numpy as np
from .main_rotateToGetMinBB import get_bb_dim

def extract_slices(cloud, dimensions, processRepeatX, processRepeatY):
    cloud_np = cloud.toNpArray()
    max_x = np.max(cloud_np[:, 0])        
    min_x = np.min(cloud_np[:, 0]) 
    max_y = np.max(cloud_np[:, 1])        
    min_y = np.min(cloud_np[:, 1]) 
    max_z = np.max(cloud_np[:, 2])        
    min_z = np.min(cloud_np[:, 2]) 

    if processRepeatX == True and processRepeatY == False:
        minCorner = (float(min_x), float(min_y), float(min_z))
        maxCorner = (float(min_x+0.05), float(max_y), float(max_z))
        bbox = cc.ccBBox(minCorner, maxCorner, True)
    if processRepeatX == False and processRepeatY == True:
        minCorner = (float(min_x), float(min_y), float(min_z))
        maxCorner = (float(max_x), float(min_y+0.05), float(max_z))
        bbox = cc.ccBBox(minCorner, maxCorner, True)   

    slices, _, _ = cc.ExtractSlicesAndContours(entities=[cloud], bbox=bbox, singleSliceMode=False,
                                                            processRepeatX=processRepeatX,
                                                            processRepeatY=processRepeatY,
                                                            processRepeatZ=False,
                                                            extractEnvelopes=False,
                                                            extractLevelSet=True,
                                                            generateRandomColors=False)
  
    left_height, error_left_height, right_height, error_right_height, top_width, error_top_width, bottom_width, error_bottom_width = [None]*8
    results_yl = []
    results_yr = []
    results_xt = []
    results_xb = []
    error_yl, error_yr, error_xt, error_xb = [None]*4
    for slice in slices:
        bb = slice.getOwnBB() 
        x, y, _ = get_bb_dim(bb)        
        if dimensions['wall_height_left'] != 0:
            error_yl = y - dimensions['wall_height_left']
        if dimensions['wall_height_right'] != 0:
            error_yr = y - dimensions['wall_height_right']
        if dimensions['wall_width_top'] != 0:
            error_xt = x - dimensions['wall_width_top']
        if dimensions['wall_width_bottom'] != 0:
            error_xb = x - dimensions['wall_width_bottom']
        if processRepeatX == True and processRepeatY == False:
            results_yl.append((y, error_yl))
            results_yr.append((y, error_yr))
        if processRepeatX == False and processRepeatY == True:
            results_xt.append((x, error_xt))
            results_xb.append((x, error_xb))

    def find_min_error(results):
        lengths = [item[0] for item in results]
        errors = [item[1] for item in results]
        abs_errors = [abs(item[1]) for item in results]
        index_error = np.argmin(abs_errors)
        error = errors[index_error]  
        length = lengths[index_error]  
        return length, error
    
    if processRepeatX == True and processRepeatY == False:
        if error_yl:
            left_height, error_left_height = find_min_error(results_yl)
        if error_yr:
            right_height, error_right_height = find_min_error(results_yr)
    if processRepeatX == False and processRepeatY == True:
        if error_xt:
            top_width, error_top_width = find_min_error(results_xt)
        if error_xb:
            bottom_width, error_bottom_width = find_min_error(results_xb)
    return left_height, error_left_height, right_height, error_right_height, top_width, error_top_width, bottom_width, error_bottom_width

if __name__ == "__main__":
    cloud_path = r"new_cloud-20241213004.ply"
    cloud = cc.loadPointCloud(cloud_path) 
    left_height, error_left_height, right_height, error_right_height,_,_,_,_ = extract_slices(cloud, processRepeatX=True, processRepeatY=False)
    _,_,_,_,top_width, error_top_width, bottom_width, error_bottom_width =  extract_slices(cloud, processRepeatX=False, processRepeatY=True)
    ic(left_height, error_left_height, right_height, error_right_height, top_width, error_top_width, bottom_width, error_bottom_width)

   
