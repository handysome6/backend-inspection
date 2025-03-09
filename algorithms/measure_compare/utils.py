
import time
import functools
from typing import Callable
from loguru import logger


# 函数运行时间计时器
def timer(func: Callable) -> Callable:
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        logger.info(f"函数 {func.__name__} 运行时间: {end_time - start_time:.4f} 秒")
        return result
    return wrapper