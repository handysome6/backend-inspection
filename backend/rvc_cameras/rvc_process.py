# Andy Liu
# Description:
# This is a process wrapper for RVC X Camera.
# This class handles two seperate PyRVC X Cameras.
# enables capture two cameras in turn. i.e. first left and then right.

import PyRVC as RVC
import os
import numpy as np
import cv2
from pathlib import Path
from multiprocessing import Process
from multiprocessing.connection import Connection
from multiprocessing import Pipe
import asyncio

FLIP_LR = 0

class RVCXCamerasProcess(Process):
    def __init__(self, conn: Connection):
        super().__init__()
        self.left_camera = None
        self.right_camera = None
        self.conn = conn
        self.is_running = True

    def run(self):
        self.system_init()
        self.left_camera = self.init_single_camera(FLIP_LR)
        self.right_camera = self.init_single_camera(1 - FLIP_LR)

        while self.is_running:
            # wait for command
            command = self.conn.recv()
            if command == "capture":
                left_image, right_image = self.capture()
                self.conn.send((left_image, right_image))
            elif command == "stop":
                self.is_running = False
            else:
                print("Unknown command: %s" % command)

    def system_init(self):
        RVC.SystemInit()
        
        opt = RVC.SystemListDeviceTypeEnum.GigE
        ret, self.devices = RVC.SystemListDevices(opt)
        print("RVC X GigE Camera devices number:%d" % len(self.devices))
        
    def init_single_camera(self, camera_id):
        # Create a RVC X Camera and choose use left side camera.
        x = RVC.X1.Create(self.devices[camera_id], RVC.CameraID_Left)

        # Test RVC X Camera is valid or not.
        if x.IsValid():
            print("RVC X Camera is valid!")
        else:
            print("RVC X Camera is not valid!")
            RVC.X1.Destroy(x)
            RVC.SystemShutdown()
            return 1

        # Open RVC X Camera.
        ret1 = x.Open()

        # Test RVC X Camera is opened or not.
        if ret1 and x.IsOpen():
            print("RVC X Camera is opened!")
        else:
            print("RVC X Camera is not opened!")
            RVC.X1.Destroy(x)
            RVC.SystemShutdown()
            return 1

        # set additional parameters for the camera
        # TODO

        return x
    
    def capture(self):
        left_image = self.capture_single_camera(self.left_camera, "Data/left")
        right_image = self.capture_single_camera(self.right_camera, "Data/right")
        return left_image, right_image
    
    def capture_single_camera(self, x, save_address):
        save_address = Path(save_address)
        save_address.mkdir(parents=True, exist_ok=True)
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
            print("width=%d, height=%d" % (width, height))

            # Convert image to array and save it.
            img = np.array(img, copy=False)
            cv2.imwrite(str(img_path), img)

            # Save point map (m) to file.
            point_map = x.GetPointMap()
            if point_map.Save(str(cloud_path), RVC.PointMapUnitEnum.Meter):
                print("Save point map successed!")
            else:
                print("Save point map failed!")

            # Get depth map and save it.
            depth_map = x.GetDepthMap()
            if depth_map.SaveDepthMap(str(depth_path), True):
                print("Save depth map successed!")
            else:
                print("Save depth map failed!")
            
            return img_path, cloud_path, depth_path
        else:
            print("RVC X Camera capture failed!")
            print(RVC.GetLastErrorMessage())
            x.Close()
            RVC.X1.Destroy(x)
            RVC.SystemShutdown()
            return 1

    def system_shutdown(self):
        # Destroy RVC X Camera.
        RVC.X1.Destroy(self.left_camera)
        RVC.X1.Destroy(self.right_camera)

        # Shutdown RVC X Camera system.
        RVC.SystemShutdown()

    def __del__(self):
        self.system_shutdown()
        self.conn.close()



if __name__ == "__main__":
    conn1, conn2 = Pipe()
    process = RVCXCamerasProcess(conn1)
    process.start()
    import time
    time.sleep(10)
    conn2.send("capture")
    a = conn2.recv()
    print(a)
    time.sleep(5)
    conn2.send("stop")
    process.join()
    print("Process stopped!")