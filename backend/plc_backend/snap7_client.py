import snap7
from backend.utils import logger

client = snap7.client.Client()
client.connect("192.168.110.180", 0, 1)
logger.info(f"PLC Snap7: {client.get_connected()}")
logger.info(f"PLC Snap7: {client.get_cpu_info()}")


def read_wall_index_and_model() -> tuple[int, str]:
    # 产品编号 Uint
    index = snap7.util.get_uint(client.db_read(96, 2624, 2), 0)
    # 产品型号 String
    model = snap7.util.get_string(client.db_read(96, 2626, 200), 0)
    return index, model

if __name__ == "__main__":
    index, model = read_wall_index_and_model()
    logger.info(f"PLC Snap7: {index}, {model}")
