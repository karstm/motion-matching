import bvh
import numpy as np
import os
import glob

os.chdir(os.path.dirname(__file__))

""" Basic function for mirroring animation data with this particular skeleton structure """

def animation_mirror(i, j, lrot, lpos, names):

    joints_mirror = np.array([(
        names.index('Left'+n[5:]) if n.startswith('Right') else (
        names.index('Right'+n[4:]) if n.startswith('Left') else 
        names.index(n))) for n in names])

    lpos[:, :, 2] = -lpos[:, :, 2]
    lrot[:, :, i] = -lrot[:, :, i]
    lrot[:, :, j] = -lrot[:, :, j]

    #lrot[:, :, 0] = ((mirrored_rotation + np.pi) % (2 * np.pi)) - np.pi


    pos_mirror = lpos[:, joints_mirror, :]
    rot_mirror = lrot[:, joints_mirror, :]
    new_names = []

    for idx in joints_mirror:
        new_names.append(names[idx])

    return rot_mirror, pos_mirror, new_names

""" Files to Process """
folder_path = os.path.join(os.path.dirname(__file__), "..", "data", "mocap", "lafan1")
files = glob.glob(os.path.join(folder_path, "*"))

""" Loop Over Files """

configurations = ['xyz']
pairs = [(1, 2)]

for filename in files:
    if ('mirror' in filename): continue
    for conf in configurations:
        for i, j in pairs:
            print('Loading "%s" %s...' % (filename, "(Mirrored)"))
            bvh_data = bvh.load(filename)
            
            bvh_data['rotations'], bvh_data['positions'], bvh_data['names'] = animation_mirror(i, j, bvh_data['rotations'], bvh_data['positions'], bvh_data['names'])
            bvh_data['order'] = conf

            mirrored_file_name = filename[:-4] + '_mirror.bvh'

            bvh.save(mirrored_file_name, bvh_data, frametime=bvh_data['frametime'], flip=True)