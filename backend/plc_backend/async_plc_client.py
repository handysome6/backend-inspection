import asyncio
from pyModbusTCP.client import ModbusClient
from loguru import logger
from config import PLC_HOST, PLC_PORT


class PLC_D901():
    RESET = 1
    STOP = 2
    FORWARD = 8
    BACKWARD = 16

class PLC_D904():
    """
    墙板就位信号
    """
    NOT_IN_POS = 0
    IN_POS = 1

class PLC_D905():
    """
    拍照完成信号
    """
    NOT_FINISHED = 0
    FINISHED = 1

class AsyncPLCClient():
    def __init__(self, host=PLC_HOST, port=PLC_PORT):
        self.client = ModbusClient(host=host, port=port)
        self.client.debug = True
        self.client.open()
        self.ready = False
        # asyncio.run(self.reset())

    async def reset(self):
        # reset PLC status
        logger.info('resetting')
        self.client.write_single_register(902, 1)
        await asyncio.sleep(0.3)
        self.ready = True

    async def move_to(self, pos_idx: int):
        if not self.ready:
            await self.reset()
        logger.info(f"Moving to position idx {pos_idx}...")
        self.client.write_single_register(902, pos_idx)
        await self.wait_for_inpos()

    async def wait_for_inpos(self):
        # signal register to listen - TODO
        await asyncio.sleep(0.1)
        while 1:
            # return 0 if not in pos, return id if in pos
            in_pos = self.client.read_holding_registers(903, 1)[0]
            # print(in_pos)
            if in_pos != 0:
                logger.info(f"Moved to target pos {in_pos}")
                break
            await asyncio.sleep(0.1)

    async def wait_for_production_line(self):
        await asyncio.sleep(0.1)
        while 1:
            # return 0 if not in pos, return 1 if in pos
            prodline_in_pos = self.client.read_holding_registers(904, 1)[0]
            # print(prodline_in_pos)
            if prodline_in_pos == PLC_D904.IN_POS:
                logger.info(f"Production line in position!")
                break
            await asyncio.sleep(0.1)

    def write_capture_finished(self):
        self.client.write_single_register(905, PLC_D905.FINISHED)

    def stop(self):
        self.client.write_single_register(901, PLC_D901.STOP)
        self.ready = False


if __name__ == "__main__":
    c = AsyncPLCClient()
    # c.move_to(4)
    asyncio.run(c.move_to(4))
