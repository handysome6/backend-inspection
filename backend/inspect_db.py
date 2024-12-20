"""
Database for the inspection project.
"""
from peewee import *
import datetime
from pathlib import Path
from icecream import ic
import aiofiles
from loguru import logger
from config import ROOT_FOLDER

DB_DIR = Path(ROOT_FOLDER) / ".db"
DXF_DIR = DB_DIR / 'dxf'  # data directory for storing dxf files
DXF_DIR.mkdir(parents=True, exist_ok=True)

DB_PATH = DB_DIR / 'inspection.db'
db = SqliteDatabase(str(DB_PATH))


class BaseModel(Model):
    class Meta:
        database = db

class WallResult(BaseModel):
    id = CharField(unique=True)
    frame_folder = CharField()
    dxf_filename = CharField(null=True)
    wall_index = IntegerField()
    wall_model = CharField()
    created_date = DateTimeField(default=datetime.datetime.now)

db.connect()
db.create_tables([WallResult])

async def db_add_dxf_file(filename: str, stream: bytes) -> str:
    """
    add a dxf file to the database folder
    :param filename: the filename of the dxf file
    :param stream: the binary stream of the dxf file

    :return: the filename of the dxf file
    """
    # check if the file already exists under the folder
    dxf_path = DXF_DIR / filename
    if dxf_path.exists():
        logger.warning(f"the file {filename} already exists")
        return dxf_path.name

    # save the file
    dxf_path = DXF_DIR / filename
    async with aiofiles.open(dxf_path, "wb") as f:
        await f.write(stream)

    return dxf_path.name

if __name__ == "__main__":
    WallResult.create(id=1, frame_folder=str(Path('.').absolute()), dxf_filename=None,
                        wall_index=1, wall_model="test_test")
