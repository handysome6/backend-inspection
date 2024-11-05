from .rvc_process import RVCXCamerasProcess
from multiprocessing import Pipe


class SyncedRVCXCameras():
    def __init__(self):
        self.conn, _conn = Pipe()
        self.process = RVCXCamerasProcess(_conn)
        self.process.start()

    def capture(self):
        self.conn.send("capture")
        left_image, right_image = self.conn.recv()
        return left_image, right_image

    def stop(self):
        self.conn.send("stop")
        self.process.join()
        print("Process stopped!")

    def __del__(self):
        self.stop()


if __name__ == "__main__":
    rvc = SyncedRVCXCameras()
    import time
    time.sleep(10)
    left_image, right_image = rvc.capture()
    print(left_image, right_image)
    time.sleep(5)
    rvc.stop()