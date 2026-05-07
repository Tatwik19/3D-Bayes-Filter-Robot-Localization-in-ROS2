import os
import re


def parse_world_file(path):
    """Read a Stage .world file and return a dict of {fiducial_id: (x, y)}.

    Looks for blocks of the form:
        my_block ( pose [ x y ... ]  fiducial_return N )
    """
    landmarks = {}

    if not os.path.exists(path):
        print(f'[map_parser] ERROR: world file not found at {path}')
        return landmarks

    with open(path, 'r') as f:
        content = f.read()

    block_re = re.compile(r'my_block\s*\((.*?)\)', re.DOTALL)
    pose_re  = re.compile(r'pose\s*\[\s*([-\d.]+)\s+([-\d.]+)')
    id_re    = re.compile(r'fiducial_return\s+(\d+)')

    for block in block_re.findall(content):
        p = pose_re.search(block)
        i = id_re.search(block)
        if p and i:
            landmarks[int(i.group(1))] = (float(p.group(1)), float(p.group(2)))

    return landmarks