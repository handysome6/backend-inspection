"""
Main file for the inspection project.
"""
from envActivation import setup_environment
setup_environment()
import cloudComPy as cc
cc.initCC()
from backend.async_io import AsyncIO
from backend.server.fusion_server import FusionServerHandler
import asyncio
from loguru import logger
import traceback

async def main():
    fusion_server = FusionServerHandler()
    task1 = asyncio.create_task(fusion_server.start_server())
    test = AsyncIO()
    task2 = asyncio.create_task(test.async_input('input: '))
    await asyncio.gather(task1, task2)

try:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
except BaseException as e:
    e = traceback.format_exc(); logger.error(e)
    logger.info("Server stopped")
    loop.stop()
    pass
