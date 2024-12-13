"""
Utility functions for the inspection project.
"""

##########################################################################
# loguru formatter
##########################################################################
import os
import sys
os.environ["LOGURU_LEVEL"] = "INFO"
from loguru import logger
logger.remove()
fmt = "<green>[{time:YYYY-MM-DD HH:mm:ss}]</green> - <level>[{level: <8}]</level> - <cyan>[{function}</cyan>:<cyan>{line}]</cyan> - <level>{message}</level>"
logger.add(sys.stderr, format=fmt)


##########################################################################
# windows toast
##########################################################################
from windows_toasts import WindowsToaster, Toast, ToastDuration
_toaster = WindowsToaster('Inspection')
_newToast = Toast(duration=ToastDuration.Short)
def toast_info(msg: str):
    _newToast.text_fields = [msg]
    _toaster.show_toast(_newToast)


##########################################################################
# temp file context manager
##########################################################################
import os
import tempfile
from contextlib import contextmanager
@contextmanager
def MyTempFile(suffix=''):
    """
    创建一个临时文件，并返回其路径, 退出with块后自动删除
    """
    fd, path = tempfile.mkstemp(suffix=suffix)
    try:
        yield path
    finally:
        os.close(fd)
        os.unlink(path)