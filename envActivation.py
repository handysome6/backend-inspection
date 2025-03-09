import os
import sys
from pathlib import Path
from icecream import ic

batch_file_path = r"C:\workspace\CloudComPy310\envCloudComPy.bat"
# 获取脚本所在目录
SCRIPT_DIR = Path(batch_file_path).parent.absolute()
CLOUDCOMPY_ROOT = SCRIPT_DIR
ic(SCRIPT_DIR)
ic(CLOUDCOMPY_ROOT)

# 设置环境变量
def setup_environment():
    # 设置 PYTHONPATH
    python_path = os.environ.get('PYTHONPATH', '')
    python_path = f"{CLOUDCOMPY_ROOT}/CloudCompare;{python_path}"
    python_path = f"{CLOUDCOMPY_ROOT}/doc/PythonAPI_test;{python_path}"
    os.environ['PYTHONPATH'] = python_path

    # 添加到 Python 路径
    sys.path.append(str(CLOUDCOMPY_ROOT / "CloudCompare"))
    sys.path.append(str(CLOUDCOMPY_ROOT / "doc" / "PythonAPI_test"))
    
    # 设置 PATH
    path = os.environ.get('PATH', '')
    path = f"{CLOUDCOMPY_ROOT}/CloudCompare;{CLOUDCOMPY_ROOT}/ccViewer;{SCRIPT_DIR};{path}"
    path = f"{CLOUDCOMPY_ROOT}/CloudCompare/plugins;{path}"
    os.environ['PATH'] = path

if __name__ == '__main__':
    setup_environment()
    
    # 检查环境
    try:
        print("Checking environment, Python test: import cloudComPy")
        import cloudComPy
        print("Environment OK!")
    except Exception as e:
        print("Incorrect Environment! Problem with Python test!")
        sys.exit(1)

