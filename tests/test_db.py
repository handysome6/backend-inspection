from backend.inspect_db import db, WallResult

def sample_imageset_create(image_dir):
    # get a dxf file
    dxf_filename = 'test0.dxf'

    # create a new image set
    image_set = WallResult.create(id = 0, frame_folder=image_dir, dxf_filename=dxf_filename)

    return image_set

from pathlib import Path
# sample_imageset_create(str(Path('.').absolute()))
row = WallResult.select().first()
print(row.frame_folder)

import datetime
today =  datetime.date.today()
tomor =  today + datetime.timedelta(days=1)
row = WallResult.select().where(WallResult.created_date.between(today, tomor))
print(len(row))


print(today.strftime('%Y%m%d') )