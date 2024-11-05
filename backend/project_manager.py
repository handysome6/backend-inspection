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

from backend.inspect_db import db, DxfFile, WallResult
from algorithms.calib_concant import combine_frames_extrinsic
from algorithms.utils import padding_img_to_ratio_3_2
from algorithms.segment_pc import get_top_surface
from config import CAM_EXT_PKL, TRAJ_EXT_PKL, ROOT_FOLDER, RUN_SIMULATION, SIMULATION_DATA_DIR
from algorithms.fitting_algorithms import run_boundary_fitting
from algorithms.pcd_convert_png import plot_skeleton_on_image

class ProjectManager:
    def __init__(self, dxf_id):
        self.dxf_id = dxf_id
        self.captured_result = {}
        self.pcd = None
        self.postprocess_finished = False
        self.preview_img = None

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

        # select dxf entry
        dxf_file = DxfFile.select().where(DxfFile.id == self.dxf_id)
        # create inspection record
        WallResult.create(id=self.inspect_id, frame_folder=self.saving_path, dxf_file=dxf_file)

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

    async def preprocess_pcd(self):
        if self.pcd is None:
            logger.error("no pcd, please combine pcd first")
            return None
        # self.pcd_surface = get_top_surface(self.pcd)

        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            pcd_surface = await loop.run_in_executor(executor, get_top_surface, 
                                                              self.pcd)
        self.pcd_surface = pcd_surface

        # save pcd_surface
        o3d.io.write_point_cloud(str(self.saving_path / "pcd_surface.ply"), self.pcd_surface)
        return self.pcd_surface

    async def fitting_boundary(self, pcd):
        dxf_path = DxfFile.select().where(DxfFile.id == self.dxf_id).first().storing_path
        if not Path(dxf_path).exists():
            logger.error(f"未找到ID为{self.dxf_id}的DXF文件")
            raise ValueError(f"未找到ID为{self.dxf_id}的DXF文件")
        
        excel_template_path = './data/result_a6_vertical.xlsx'
        save_result_folder = self.saving_path

        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            await loop.run_in_executor(
                executor, 
                run_boundary_fitting, 
                pcd, dxf_path, excel_template_path, save_result_folder)

    async def convert_and_plot_pcd_result(self, pcd):
        from algorithms.pcd_convert_png import convert_pcd_to_2d_image, plot_skeleton_on_image

        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            img, transform_matrix = await loop.run_in_executor(executor, convert_pcd_to_2d_image, pcd)

        self.preview_img = img
        self.transform_matrix = transform_matrix
        img = plot_skeleton_on_image(img, transform_matrix, self.edges)

        #rotate img by 90 degree anti-clockwise
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imwrite("rotated.png", img)
        return img, transform_matrix

    async def run_algorithms(self):
        pcd_combined = await self.combine_pcds()
        pcd_surface = await self.preprocess_pcd()
        await self.fitting_boundary(pcd_surface)

        # run algorithms
        self.postprocess_finished = True

    async def get_postprocess_preview_img(self):
        while not self.postprocess_finished:
            await asyncio.sleep(0.1)

        path = str(self.saving_path / "preview.png")
        return path

