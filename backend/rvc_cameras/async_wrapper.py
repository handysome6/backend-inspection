from .rvc_process import RVCXCamerasProcess
from multiprocessing import Pipe
import asyncio

class AsyncRVCXCameras():
    def __init__(self):
        self.conn, _conn = Pipe()
        self.process = RVCXCamerasProcess(_conn)
        self.process.start()

    async def capture(self):
        self.conn.send("capture")
        while not self.conn.poll(None):
            await asyncio.sleep(0.2)
        left_image, right_image = self.conn.recv()
        return left_image, right_image

    def stop(self):
        self.conn.send("stop")
        self.process.join()
        print("Process stopped!")

    def __del__(self):
        self.stop()


if __name__ == "__main__":
    import asyncio
    loop = asyncio.get_event_loop()
    
    rvc = AsyncRVCXCameras()
    left_image, right_image = loop.run_until_complete(rvc.capture())
    print(left_image, right_image)
    rvc.stop()

