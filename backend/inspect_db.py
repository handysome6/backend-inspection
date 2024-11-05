"""
Database for the inspection project.
"""
from peewee import *
import datetime
from pathlib import Path
from icecream import ic
import hashlib
from config import ROOT_FOLDER
import asyncio
import aiofiles


DB_DIR = Path(ROOT_FOLDER) / ".db"
DXF_DIR = DB_DIR / 'dxf'  # data directory for storing dxf files
DXF_DIR.mkdir(parents=True, exist_ok=True)

DB_PATH = DB_DIR / 'inspection.db'
db = SqliteDatabase(str(DB_PATH))


class BaseModel(Model):
    class Meta:
        database = db

class DxfFile(BaseModel):
    id = AutoField()
    filename = CharField(unique=False)
    storing_path = CharField()
    hash = CharField()

class WallResult(BaseModel):
    id = CharField(unique=True)
    frame_folder = CharField()
    dxf_file = ForeignKeyField(DxfFile, backref='wall_results')
    created_date = DateTimeField(default=datetime.datetime.now)

db.connect()
db.create_tables([DxfFile, WallResult])


def test_add_dxf_file(filename: str, stream: bytes) -> int:
    """
    add a dxf file to the database
    :param filename: the filename of the dxf file
    :param stream: the binary stream of the dxf file

    :return: the id (primary key) of the dxf file
    """
    # check if the file already exists
    md5 = hashlib.md5(stream).hexdigest()
    dxf_file = DxfFile.select().where(DxfFile.hash == md5).first()

    if dxf_file is None:
        # save the file
        dxf_path = DXF_DIR / f"{md5}.dxf"
        with open(dxf_path, "wb") as f:
            f.write(stream)
        dxf_file = DxfFile.create(filename=filename, storing_path=str(dxf_path), hash=md5)
    else:
        # the file already exists
        pass
    return dxf_file.id

def sample_imageset_create(image_dir):
    # get a dxf file
    dxf_file = DxfFile.select().where(DxfFile.filename == 'qiangban.dxf').get()

    # create a new image set
    image_set = WallResult.create(image_dir=image_dir, dxf_file=dxf_file, frame_number=3)

    return image_set

async def db_add_dxf_file(filename: str, stream: bytes) -> int:
    """
    add a dxf file to the database
    :param filename: the filename of the dxf file
    :param stream: the binary stream of the dxf file

    :return: the id (primary key) of the dxf file
    """
    # check if the file already exists
    md5 = hashlib.md5(stream).hexdigest()
    dxf_file = DxfFile.select().where(DxfFile.hash == md5).first()

    if dxf_file is None:
        # save the file
        dxf_path = DXF_DIR / f"{md5}.dxf"
        async with aiofiles.open(dxf_path, "wb") as f:
            await f.write(stream)
        dxf_file = DxfFile.create(filename=filename, storing_path=str(dxf_path), hash=md5)
    else:
        # the file already exists
        pass
    return dxf_file.id


if __name__ == '__main__':
    # # add a new dxf file
    # path = Path(r'Data\qiangban.dxf')
    # filename = path.name
    # stream = path.read_bytes()
    # db_add_dxf_file(filename, stream)

    dxf_file = DxfFile.select().where(DxfFile.filename == 'qiangban.dxf').get()
    print(dxf_file.filename)