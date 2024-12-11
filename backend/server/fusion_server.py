# Andy Liu
# Description:
# This is the main server file for the fusion server. 
# It include both websocket and http server for handling front-end requests.
# The server will be running on port 1337.
# The server will be listening to the following routes:
# - /ws: websocket route for handling websocket requests
# - /exportExcel: route for handling exporting excel file
# - /leftImage: route for handling left image requests
# - /rightImage: route for handling right image requests
# - /imageList: route for handling image list requests
# - /uploadDxf: route for handling uploading dxf file requests

import json
import base64
import aiohttp
import asyncio
import hashlib
import aiofiles
import traceback
import aiohttp_cors
from aiohttp import web
from pathlib import Path
from icecream import ic
import cv2

from backend.utils import logger
from backend.inspect_db import db, DxfFile, WallResult, db_add_dxf_file
from backend.hardware_manager import HardwareManager
from backend.project_manager import ProjectManager
from config import RUN_SIMULATION, PLC_WAIT_FOR_WALL
from algorithms.dxf_convert_png import export_dark_bg


class FusionServerHandler:
    def __init__(self):
        self.current_step = 1  # current step of the server
        self.ws = None          # current websocket connect

        logger.info('Fusion Server Handler initialized')
        logger.info(f'current step: {self.current_step}')

        self.hardware_manager = HardwareManager()
        self.project_manager = None

        self.dxf_file_id = None
        self.current_capture_task = None

    async def run_capture_process(self):
        try:
            # 等待产线就位
            if not RUN_SIMULATION and PLC_WAIT_FOR_WALL:
                logger.info("waiting for production line in position...")
                await self.hardware_manager.pcl_wait_for_prod_line()
            else:
                logger.info("skipped waiting for production line")

            steps_list = [1,2,3,4,5,6,7,8]
            # steps_list = [4,5,6,7,8]
            self.project_manager = ProjectManager(self.dxf_file_id)
            self.hardware_manager.set_capture_saving_path(self.project_manager.saving_path)

            for step in steps_list:
                res = await self.hardware_manager.move_to_and_capture(step)
                self.project_manager.add_captured_result(step, res)
                await self.ws_notify_img_step(step)

            self.project_manager.add_to_db()
            logger.info("start post-processing...")

            from config import USE_FAKE_DATA
            if not USE_FAKE_DATA:
                self.post_process_task = asyncio.create_task(self.post_process_coroutine())
            else:
                pass
        except asyncio.CancelledError:
            logger.info("capture process cancelled")
        except Exception as e:
            logger.error(f"Error in run_capture_process: {e}")

    async def post_process_coroutine(self):
        await self.project_manager.run_algorithms()
        logger.info("Sending refresh command to client")
        await self.ws.send_str(json.dumps({'cmd':'refresh'}))

    async def websocket_handler(self, request):
        """
        Usage:
            prepare for websocket requests
            handle ping-pong for keep-alive
            handle shutdown websocket connection
        """
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.ws = ws
        logger.info('ws connection established')

        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                if msg.data == 'close':                 # close the websocket connection
                    logger.info('WS received close, ws connection closed')
                    await ws.close()
                elif msg.data == 'ping':                # ping pong heartbeat
                    await ws.send_str('pong')
                    logger.debug('WS received ping, ws send pong')
                    continue
                else:
                    loop = asyncio.get_running_loop()
                    loop.create_task(self.ws_state_control(msg))
                    # loop.run_until_complete(self.ws_notify_img_step(msg))
                    # await self.ws_state_control(msg)
            elif msg.type == aiohttp.WSMsgType.ERROR:
                logger.error(f'ws connection closed with exception {ws.exception()}')
                self.ws = None

        return ws

    async def ws_state_control(self, msg):
        """
        handle the websocket message
        :param msg: the websocket message
        """
        try:
            data = json.loads(msg.data)
            if "state" in data:
                cmd = data['state']
                if cmd == 'stop':
                    # TODO - stop current move and set img_step to 1
                    pass
                elif cmd == 'restart':
                    # TODO - restart current photo-taking from 1
                    pass
            if 'step' in data:
                # update global state
                self.current_step = int(data['step'])
                logger.info(f"current step updated to {self.current_step}")
                await self.ws.send_str("success")
                if self.current_step == 3:
                    # add task of running capture process
                    self.current_capture_task = asyncio.create_task(self.run_capture_process())
            else:
                logger.warning(f'unknown ws message {msg.data}')
                # await ws.send_str('some websocket message payload')
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            await self.ws.send_str(f"Internal Server Error, connection closed. \n{e}")
            await self.ws.close()
            self.ws = None

    async def ws_notify_img_step(self, img_step):
        """
        notify the front-end that the image is ready
        :param img_step: the step of the image
        """
        if self.ws is None:
            return
        try:
            logger.info(f'notify client image step: {img_step}')
            await self.ws.send_str(json.dumps({ 'state':'image ready','imageStep':img_step}))
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            await self.ws.send_str(f"Internal Server Error, connection closed. \n{e}")
            await self.ws.close()
            self.ws = None

    async def upload_dxf_handler(self, request):
        """
        handle the dxf file upload request - POST
        :param request: the http request
        :return: the http response
            {
                'success': True,
                'data': {'id': file_id},
                'message': '上传成功'
            }
        """
        try:
            data = await request.post()
            dxf_file = data['dxfFile']          # dxf file as a FileField
            filename = Path(dxf_file.filename)

            # check if the file is a dxf file
            if filename.suffix != '.dxf':
                logger.warning('Not a dxf file')
                return web.json_response({'success': False, 'data': None, 'message': 'Not a dxf file'})

            # save the FIELFIELD
            self.dxf_file_id = await db_add_dxf_file(filename, dxf_file.file.read())
            self.current_step = 2
            logger.info(f"current step updated to {self.current_step}")
            return web.json_response({'success': True, 'data': {'id': self.dxf_file_id}, 'message': '上传成功'})
        
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_concant_num_handler(self, request):
        """
        handle the concant number request - GET
        :param request: the http request
        :return: the http response
            {
                'success': True,
                'data': {'num': num},
                'message': ''
            }
        """
        try:
            dxfId = request.query['dxfId']
            # query the dxf file
            dxf_file = DxfFile.select().where(DxfFile.id == dxfId).first()
            # TODO: call the dxf algorithm to get the concant number
            num = 8
            logger.info(f"returning concant number: {num}")
            return web.json_response({'success': True, 'data': {'num': num}, 'message': ''})
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_left_img_handler(self, request):
        """
        handle the left image request - GET
        :param request: the http request
        :return: the http response
            FileResponse containing the left image
        """
        try:
            path = request.path
            imageStep = request.query['imageStep']
            left_frame = self.project_manager.get_left_frame(int(imageStep))
            return web.FileResponse(left_frame[0])
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_right_img_handler(self, request):
        """
        handle the right image request - GET
        :param request: the http request
        :return: the http response
            FileResponse containing the right image
        """
        try:
            path = request.path
            imageStep = request.query['imageStep']
            right_frame = self.project_manager.get_right_frame(int(imageStep))
            return web.FileResponse(right_frame[0])
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_img_list_handler(self, request):
        """
        handle the image list request - GET
        :param request: the http request
        :return: the http response
            {
                'success': True,
                'data': [{'id': id, 'date': date, 'num': num, 'imageSrc': image_base64}],
                'message': ''
            }
        """
        try:
            path = request.path
            # database query
            image_list = []
            for wall_result in WallResult.select():
                image_path = Path(wall_result.frame_folder) / 'preview.png'
                async with aiofiles.open(image_path, "rb") as f:
                    image = await f.read()
                image = base64.b64encode(image)
                image = image.decode('utf-8')
                image_list.append({
                    'id': wall_result.id,
                    'date': wall_result.created_date.strftime('%Y-%m-%d %H:%M:%S'),
                    'num': 8,
                    'imageSrc': image
                })
            
            return web.json_response({'success': True, 'data': image_list, 'message': ''})
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_file_info_handler(self, request):
        """
        handle the file info request - GET
        :param request: the http request
            {
                'id': image id
            }
        :return: the http response
            FileResponse containing the image file
        """
        try:
            path = request.path
            img_id = request.query['wallID']
            # query the image file
            wall_result:WallResult = WallResult.select().where(WallResult.id == img_id).first()
            if wall_result is None:
                raise Exception(f"Image ID {img_id} not found")
            # TODO: get the real preview image file
            from config import USE_FAKE_DATA
            if not USE_FAKE_DATA:
                image_path = await self.project_manager.get_postprocess_preview_img()
            else:
                image_path = "./Data/preview.png"
            if not Path(image_path).exists():
                logger.error("Preview image not found.")
            return web.FileResponse(image_path)
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_export_excel_handler(self, request):
        """
        handle the export excel request - GET
        :param request: the http request
        :return: the http response
            FileResponse containing the excel file
        """
        try:
            path = request.path
            # return FielResponse containing the excel file
            return web.FileResponse(r"Data\检测表.xlsx")
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_dxf_by_img_id_handler(self, request):
        """
        handle the dxf file request - GET
        :param request: the http request
        :return: the http response
            FileResponse containing the dxf file
        """
        try:
            path = request.path
            logger.info(str(len(request.query)))
            wall_id = request.query['wallID']
            # get the corresponding dxf_id from the database
            dxf_id = WallResult.select().where(WallResult.id == wall_id).first().dxf_file_id

            # query the dxf file
            dxf_file: DxfFile = DxfFile.select().where(DxfFile.id == dxf_id).first()
            if dxf_file is None:
                logger.error(f"DXF ID {dxf_id} not found")
                raise Exception(f"DXF ID {dxf_id} not found")
            
            from config import USE_FAKE_DATA
            if not USE_FAKE_DATA:
                output_path = export_dark_bg(dxf_file.storing_path)
                return web.FileResponse(output_path)
            else:
                return web.FileResponse("./Data/cad.png")
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_printer_start_handler(self, request):
        """
        handle the printer start request - GET
        :param request: the http request
        :return: the http response
            json_response containing the success or failed
        """
        try:
            path = request.path
            logger.info("printing current result...")
            from backend.test_printer import print_excel
            print_excel(r"C:\workspace\backend-inspection\Data\fake_res.xlsx")
            # TODO - printer function
            if True:
                return web.json_response({'success': True, 'data': None, 'message': ''})
            else:
                return web.json_response({'success': False, 'data': None, 'message': '没有有效的检测结果可以打印'})

        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_excel_data_handler(self, request):
        """
        handle the excel data request - GET
        :param request: the http request
        :return: the http response
            json_response containing the measure result
        """
        try:
            path = request.path

            # TODO: retrive the computed measure result
            res = {
                'abd': 2.007,
                'abc': 2.007,
                'abe': 2.007
            }
            return web.json_response({'success': True, 'data': res, 'message':''})
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_new_project_handler(self, request):
        """
        handle the new project request - GET
        :param request: the http request
        :return: the http response
            json_response containing success
        """
        try:
            path = request.path
            query = request.query
            print(query)

            self.reset_task = asyncio.create_task(self.hardware_manager.reset())

            self.project_manager = None

            await self.hardware_manager.write_capture_finished()
            return web.json_response({'success': True, 'data': None, 'message':''})
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})

    async def get_wall_id_handler(self, request):
        """
        handle the new project request - GET
        :param request: the http request
        :return: the http response
            json_response containing success
        """
        try:
            path = request.path
            wall_id = self.project_manager.inspect_id
            logger.info(wall_id)
            return web.json_response({'success': True, 'data': {'wallID': wall_id}, 'message':''})
        except Exception as e:
            e = traceback.format_exc(); logger.error(e)
            return web.json_response({'success': False, 'data': None, 'message': str(e)})


    async def start_server(self):
        @web.middleware
        async def print_url_path(request: web.Request, handler):
            method = request.method
            path = request.path
            query_info = request.query_string
            logger.info(f"Request: {method} {path} {query_info}")
            return await handler(request)
        
        @web.middleware
        async def cache_control(request: web.Request, handler):
            response: web.Response = await handler(request)
            # resource_name = request.match_info.route.name
            response.headers.setdefault('Cache-Control', 'no-cache')
            return response
        
        app = web.Application(middlewares=[print_url_path, cache_control], client_max_size=1048576*1e2)
        app.router.add_get('/ws', self.websocket_handler)

        cors = aiohttp_cors.setup(app)
        cors.add(app.router.add_route("GET", "/exportExcel", self.get_export_excel_handler))
        cors.add(app.router.add_route("GET", "/leftImage", self.get_left_img_handler))
        cors.add(app.router.add_route("GET", "/rightImage", self.get_right_img_handler))
        cors.add(app.router.add_route("GET", "/imageList", self.get_img_list_handler))

        cors.add(app.router.add_route("GET", "/fileInfo", self.get_file_info_handler))
        cors.add(app.router.add_route("GET", "/dxfByImageID", self.get_dxf_by_img_id_handler))

        cors.add(app.router.add_route("GET", "/wallID", self.get_wall_id_handler))

        cors.add(app.router.add_route("GET", "/concantNum", self.get_concant_num_handler))
        # TODO
        cors.add(app.router.add_route("GET", "/printerStart", self.get_printer_start_handler))
        # cors.add(app.router.add_route("GET", "/fileInfoCompare", self.get_file_info_compare_handler))
        cors.add(app.router.add_route("GET", "/excelDataInfo", self.get_excel_data_handler))
        cors.add(app.router.add_route("GET", "/newProject", self.get_new_project_handler))

        resource = cors.add(app.router.add_resource("/uploadDxf"))
        route = cors.add(
            resource.add_route("POST", self.upload_dxf_handler), {
                "*": aiohttp_cors.ResourceOptions(
                    allow_credentials=True,
                    expose_headers=("X-Custom-Server-Header",),
                    allow_headers=("X-Requested-With", "Content-Type"),
                    max_age=3600,
                )
            })

        self.runner = web.AppRunner(app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, '0.0.0.0', 1337)
        await site.start()
        logger.info(f"Server started at " + site.name)
        return self.runner

    async def stop_server(self):
        await self.runner.cleanup()
        print("Server stopped")


