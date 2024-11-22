import numpy as np
from icecream import ic
from loguru import logger
from pathlib import Path
from algorithms.CCTDecoder.cct_decode import CCT_extract
from algorithms.my_pcd import MyPCD

CCT_N = 12

def get_single_cct_world_coord(pcd: MyPCD):
    """
    Get cct ids and world coordinates from a single frame
    """
    frame = pcd
    img = frame.image

    # extract CCT id and coords
    cct_pts, _ = CCT_extract(img, CCT_N)
    ic(cct_pts.keys())

    # get 3d points from cct ids
    world_points_dict = {}
    for id, pt in cct_pts.items():
        x, y = pt
        d3_coord = frame.get_3dcoord_bilinear(x, y)
        if d3_coord is not None:
            world_points_dict[id] = d3_coord
        else:
            print(f"Failed to get 3d coord for id {id} in {frame.folder_path}")

    return world_points_dict

if __name__ == "__main__":
    pcd_folder = Path(r"C:\Users\Andy\Downloads\032\03")
    # get a single frame
    pcd = MyPCD(pcd_folder)
    # get all single cct world coordinates
    cct_ids_world_coords = get_single_cct_world_coord(pcd)
    ic(cct_ids_world_coords)

    # write a loop for getting 2 user input of two cct ids in a single frame
    # get the world coordinates of the two cct ids
    # print the world coordinates
    while True:
        print("\nDetected CCT ids: ", cct_ids_world_coords.keys())
        cct_id1 = int(input("Enter the first CCT id: "))
        cct_id2 = int(input("Enter the second CCT id: "))

        world_coord1 = cct_ids_world_coords[cct_id1]
        world_coord2 = cct_ids_world_coords[cct_id2]
        ic(world_coord1, world_coord2)
        print(f"length between {cct_id1} and {cct_id2}: ", np.linalg.norm(world_coord1 - world_coord2))