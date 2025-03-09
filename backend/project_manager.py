"""
Current wall as current project.
Manage the wall's:
   - dxf file id
   - generate new inspection id
   - photos taken
   - pointcloud and feed into algorithms
   - final measure result
"""

import datetime
from pathlib import Path
from loguru import logger
import asyncio
import concurrent.futures
import open3d as o3d
import cv2
import numpy as np

from backend.inspect_db import db, WallResult, DXF_DIR
from algorithms.calib_concant import combine_frames_extrinsic
from algorithms.utils import padding_img_to_ratio_3_2
from config import CAM_EXT_PKL, TRAJ_EXT_PKL, ROOT_FOLDER, RUN_SIMULATION, SIMULATION_DATA_DIR
from algorithms.pcd_convert_png import plot_skeleton_on_image
from algorithms.measure_compare.measurement import all_measurement

class ProjectManager:
    def __init__(self, wall_index, wall_model):
        self.wall_index = wall_index
        self.wall_model = wall_model
        self.captured_result = {}
        self.pcd = None
        self.postprocess_finished = False
        self.preview_img = None

        # parse wall_model to get the dxf filename
        self.dxf_filename = self.wall_model.split("_")[0] + ".dxf"
        self.dxf_path = DXF_DIR / self.dxf_filename
        if not self.dxf_path.exists():
            logger.error(f"DXF file {self.dxf_filename} not found")
            self.dxf_filename = None

        self.inspect_id = self.generate_new_inspection_id()
        # folder location
        self.saving_path = Path(ROOT_FOLDER) / self.inspect_id

    def generate_new_inspection_id(self):
        # get today's inspection record
        today =  datetime.date.today()
        tomor =  today + datetime.timedelta(days=1)
        row = WallResult.select().where(WallResult.created_date.between(today, tomor))
        num_records = len(row)

        # get str of today
        date = today.strftime('%Y%m%d')

        new_inspect_id = f"{date}{str(num_records).zfill(3)}"
        self.inspect_id = new_inspect_id
        return new_inspect_id

    def add_to_db(self):
        # create inspection folder if not exists
        self.saving_path.mkdir(parents=True, exist_ok=True)

        # create inspection record
        WallResult.create(id=self.inspect_id, frame_folder=self.saving_path, dxf_filename=self.dxf_filename,
                         wall_index=self.wall_index, wall_model=self.wall_model)

    def add_captured_result(self, frame_id, frames_path):
        """
        frame_id: among 1-8
        frames_path: 
            [(left_img, left_pcd, left_depth), 
             (right_img, right_pcd, right_depth)]
        """
        self.captured_result[frame_id] = frames_path

    def get_left_frame(self, frame_id):
        return self.captured_result[frame_id][0]
        
    def get_right_frame(self, frame_id):
        return self.captured_result[frame_id][1]
    
    async def combine_pcds(self):
        """
        After captured all frames, combine them
        """
        if RUN_SIMULATION:
            combine_path = Path(SIMULATION_DATA_DIR)
        else:
            combine_path = self.saving_path

        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            pcd_combined_cropped = await loop.run_in_executor(executor, combine_frames_extrinsic, 
                                                              combine_path, CAM_EXT_PKL, TRAJ_EXT_PKL)
        self.pcd = pcd_combined_cropped

        # save pcd
        o3d.io.write_point_cloud(str(self.saving_path / "pcd_combined.ply"), self.pcd)
        return self.pcd

    async def convert_and_plot_pcd_result(self, pcd):
        from algorithms.pcd_convert_png import convert_pcd_to_2d_image

        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            img, transform_matrix = await loop.run_in_executor(executor, convert_pcd_to_2d_image, pcd)

        self.preview_img = img

        #rotate img by 90 degree anti-clockwise
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        img = padding_img_to_ratio_3_2(img)
        cv2.imwrite(str(self.saving_path / "preview.png"), img)
        return img, transform_matrix

    async def run_algorithms(self):
        pcd_combined = await self.combine_pcds()
        await self.convert_and_plot_pcd_result(pcd_combined)

        # run algorithms
        pcd_path = str(self.saving_path / "pcd_combined.ply")

        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            await loop.run_in_executor(executor, all_measurement, pcd_path, self.dxf_path)

        self.postprocess_finished = True

    async def get_postprocess_preview_img(self):
        while not self.postprocess_finished:
            await asyncio.sleep(0.1)

        path = str(self.saving_path / "img_grey_bg.png")
        return path

