# ANDY LIU
# This is a async class definition for RVC X Camera.
# This class handles two seperate PyRVC X Cameras.
# enables capture two cameras in turn. i.e. first left and then right.


import PyRVC as RVC
import os
import numpy as np
import cv2
# from Utils.Tools import *
from pathlib import Path
import asyncio
import concurrent.futures
from loguru import logger
from icecream import ic

FLIP_LR = 0

CAM_SN_DICT = {
    "left": "G1GM790B002",
    "right": "G1GM790B003"
}

if FLIP_LR:
    CAM_SN_DICT["left"], CAM_SN_DICT["right"] = CAM_SN_DICT["right"], CAM_SN_DICT["left"]



class AsyncRVCXCameras():
    def __init__(self):
        self.left_camera = None
        self.right_camera = None
        self.set_save_path(Path(r"C:\workspace\Data"))

    def set_save_path(self, path):
        self.save_path = Path(path)
        self.save_path.mkdir(parents=True, exist_ok=True)
        logger.info(f"Save path set to {self.save_path}")

    def init_single_camera(self, device_list, sn_num):
        deivce = None
        for i in device_list:
            _, device_info = i.GetDeviceInfo()
            if device_info.sn == sn_num:
                deivce = i

        # Create a RVC X Camera and choose use left side camera.
        x = RVC.X1.Create(deivce, RVC.CameraID_Left)

        # Test RVC X Camera is valid or not.
        if x.IsValid():
            logger.debug("RVC X Camera is valid!")
        else:
            logger.debug("RVC X Camera is not valid!")
            RVC.X1.Destroy(x)
            RVC.SystemShutdown()
            return 1

        # Open RVC X Camera.
        ret1 = x.Open()

        # Test RVC X Camera is opened or not.
        if ret1 and x.IsOpen():
            logger.debug("RVC X Camera is opened!")
        else:
            logger.debug("RVC X Camera is not opened!")
            RVC.X1.Destroy(x)
            RVC.SystemShutdown()
            return 1
        
        # set additional parameters for the camera
        # TODO

        return x

    def system_init(self):
        if self.system_init is True:
            return
        
        logger.info('RVC cameras initing...')
        RVC.SystemInit()
        
        opt = RVC.SystemListDeviceTypeEnum.GigE
        ret, devices = RVC.SystemListDevices(opt)
        logger.info("RVC X GigE Camera devices number:%d" % len(devices))

        #  Find whether any RVC X GigE Camera is connected or not.
        if len(devices) != 2:
            logger.error("Can not find two RVC X GigE Cameras!")
            RVC.SystemShutdown()
            return 1
        
        for device in devices:
            res, info = device.GetDeviceInfo()
            ic(info.sn)

        self.left_camera = self.init_single_camera(devices, CAM_SN_DICT["left"])
        self.right_camera = self.init_single_camera(devices, CAM_SN_DICT["right"])

        logger.info("RVC cameras init successfully.")

    async def capture_dual(self, idx):
        # img_path_l, cloud_path_l, depth_path_l = \
        #     self.capture_single(self.left_camera, save_address=Path(save_address) / "left")
        # img_path_r, cloud_path_r, depth_path_r = \
        #     self.capture_single(self.right_camera, save_address=Path(save_address) / "right")

        left_path = self.save_path / str(idx).zfill(2) / "left"
        right_path = self.save_path / str(idx).zfill(2) / "right"
        loop = asyncio.get_event_loop()
        with concurrent.futures.ThreadPoolExecutor() as executor:
            task1 = asyncio.create_task(self.do_async_job(loop, executor, self.left_camera, left_path))
            await asyncio.gather(task1)
            task2 = asyncio.create_task(self.do_async_job(loop, executor, self.right_camera, right_path))
            await asyncio.gather(task2)
        return task1.result(), task2.result()

    async def do_async_job(self, loop, executor, camera, save_address):
        a = await loop.run_in_executor(executor, self.capture_single, camera, save_address)
        # print(a)
        # print('job done!')
        return a

    def capture_single(self, x, save_address):
        Path(save_address).mkdir(parents=True, exist_ok=True)
        # Capture a point map and a image.
        ret = x.Capture()

        if ret == True:
            print("RVC X Camera capture successed!")
            img_path = save_address / "Image.png"
            cloud_path = save_address / "PointCloud.ply"
            depth_path = save_address / "Depth.tif"

            # Get image data and image size.
            img = x.GetImage()
            width, height = img.GetSize().cols, img.GetSize().rows
            # print("width=%d, height=%d" % (width, height))

            # Convert image to array and save it.
            img_copy = np.array(img, copy=False)
            cv2.imwrite(str(img_path), img_copy)

            # # Save point map (m) to file.
            # point_map = x.GetPointMap()
            # if point_map.Save(str(cloud_path), RVC.PointMapUnitEnum.Meter):
            #     logger.debug("Save point map successed!")
            # else:
            #     logger.debug("Save point map failed!")

            # Save Colored point map (m) to file.
            point_map = x.GetPointMap()
            if point_map.SaveWithImage(str(cloud_path), img, RVC.PointMapUnitEnum.Meter):
                logger.debug("Save depth map successed!")
            else:
                logger.debug("Save depth map failed!")
            
            return img_path, cloud_path, depth_path
        else:
            logger.error("RVC X Camera capture failed!")
            print(RVC.GetLastErrorMessage())
            x.Close()
            RVC.X1.Destroy(x)
            RVC.SystemShutdown()
            return 1


    def system_shutdown(self):
        # Close RVC X Camera.
        self.left_camera.Close()
        self.right_camera.Close()

        # Destroy RVC X Camera.
        RVC.X1.Destroy(self.left_camera)
        RVC.X1.Destroy(self.right_camera)

        # Shutdown RVC X Camera system.
        RVC.SystemShutdown()


if __name__ == "__main__":
    rvc = AsyncRVCXCameras()
    rvc.system_init()
    asyncio.run(rvc.capture_dual())
    rvc.system_shutdown()
    print("Done!")