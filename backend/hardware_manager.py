"""
Hardware manager for the inspection project.
Connect to PLC and RVC cameras.
If RUN_SIMULATION is True, the system will not connect to PLC and RVC cameras.
"""

import asyncio
from pathlib import Path
from loguru import logger
from backend.utils import toast_info
from config import RUN_SIMULATION, SIMULATION_DATA_DIR, PLC_WAIT_FOR_WALL
if not RUN_SIMULATION:
    from .plc_backend.async_plc_client import AsyncPLCClient
    from .rvc_cameras.async_rvc import AsyncRVCXCameras


class HardwareManager:
    def __init__(self):
        if RUN_SIMULATION:
            logger.info("Running in simulation mode...")
            self.plc_client = None
            self.rvc_client = None
        else:
            logger.info("Running in real hardware mode...")
            self.plc_client = AsyncPLCClient()
            self.rvc_client = AsyncRVCXCameras()
            if self.rvc_client.system_init() == 1:
                logger.error("RVC cameras init failed!")
                toast_info("硬件：初始化失败")
                return 1

        # show windows toast
        toast_info("硬件：初始化完成")

    async def reset(self):
        if RUN_SIMULATION:
            return
        await self.plc_client.reset()

    async def move_to_and_capture(self, pos_idx: int):
        if RUN_SIMULATION:
            logger.warning(f"sending expirmenetal data, current step: 8")

            await asyncio.sleep(2)
            # get the str(pose_idx).zfill(2) named folder's image
            folder = Path(SIMULATION_DATA_DIR) / f"{str(pos_idx).zfill(2)}"
            left_path = folder / "left"
            right_path = folder / "right"
            img_path_l = left_path / "Image.png"
            img_path_r = right_path / "Image.png"
            pcd_path_l = left_path / "PointCloud.ply"
            pcd_path_r = right_path / "PointCloud.ply"
            depth_path_l = left_path / "Depth.tif"
            depth_path_r = right_path / "Depth.tif"
            # CHECK IF THE FILE EXISTS
            if not img_path_l.exists() or not img_path_r.exists() \
                or not pcd_path_l.exists() or not pcd_path_r.exists() \
                or not depth_path_l.exists() or not depth_path_r.exists():
                logger.error(f"Image files not found in {folder}")
            return (img_path_l, pcd_path_l, depth_path_l), (img_path_r, pcd_path_r, depth_path_r)
        else:
            await self.plc_client.move_to(pos_idx)
            result = await self.rvc_client.capture_dual(pos_idx)
            return result
        
    async def plc_move_to(self, pos_idx: int):
        if RUN_SIMULATION:
            await asyncio.sleep(1)
        else:
            await self.plc_client.move_to(pos_idx)
    
    async def plc_wait_for_prod_line(self):
        await self.plc_client.wait_for_production_line()

    def set_capture_saving_path(self, save_path):
        if RUN_SIMULATION:
            logger.info("Simulaton setting saving path...")
        else:
            self.rvc_client.set_save_path(save_path)

    async def write_capture_finished(self):
        if PLC_WAIT_FOR_WALL:
            self.plc_client.write_capture_finished()
            await asyncio.sleep(1)
