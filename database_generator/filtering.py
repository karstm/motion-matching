import quat
import bvh
import scipy.signal as signal
from scipy.interpolate import griddata
import numpy as np
import glob

upsample = False

import os
os.chdir(os.path.dirname(__file__))

""" Files to Process """
folder_path = os.path.join(os.path.dirname(__file__), "..", "data", "mocap", "lafan1_selected")
files = glob.glob(os.path.join(folder_path, "*"))



def generate_sim_bone(bvh=None):
    # Extract positions and rotations
    positions = bvh['positions']
    rotations = quat.unroll(quat.from_euler(np.radians(bvh['rotations']), order=bvh['order']))
    ''' Super sample'''
    if upsample:
        nframes = positions.shape[0]
        nbones = positions.shape[1]
        
        # Supersample data to 60 fps
        original_times = np.linspace(0, nframes - 1, nframes)
        sample_times = np.linspace(0, nframes - 1, (nframes * 2 - 1))

        # This does a cubic interpolation of the data for supersampling
        positions = griddata(original_times, positions.reshape([nframes, -1]), sample_times, method='cubic').reshape([len(sample_times), nbones, 3])
        rotations = griddata(original_times, rotations.reshape([nframes, -1]), sample_times, method='cubic').reshape([len(sample_times), nbones, 4])
        
        # Need to re-normalize after super-sampling
        rotations = quat.normalize(rotations)

        bvh['frametime'] = 1 / 60.0
    
    ''' Simulation bone and filtering '''
    # First compute world space positions/rotations
    global_rotations, global_positions = quat.fk(rotations, positions, bvh['parents'])
    nframes = positions.shape[0]
    
    # Specify joints to use for simulation bone 
    sim_position_joint = bvh['names'].index("Spine2")
    sim_rotation_joint = bvh['names'].index("Hips")
    
    # Position comes from spine joint
    sim_position = np.array([1.0, 0.0, 1.0]) * global_positions[:,sim_position_joint:sim_position_joint+1]
    sim_position = signal.savgol_filter(sim_position, min(31, nframes), 3, axis=0, mode='interp')
    
    # Direction comes from projected hip forward direction
    sim_direction = np.array([1.0, 0.0, 1.0]) * quat.mul_vec(global_rotations[:,sim_rotation_joint:sim_rotation_joint+1], np.array([0.0, 1.0, 0.0]))

    # We need to re-normalize the direction after both projection and smoothing
    sim_direction = sim_direction / np.sqrt(np.sum(np.square(sim_direction), axis=-1))[...,np.newaxis]
    sim_direction = signal.savgol_filter(sim_direction, min(61, nframes), 3, axis=0, mode='interp')
    sim_direction = sim_direction / np.sqrt(np.sum(np.square(sim_direction), axis=-1)[...,np.newaxis])
    
    # Extract rotation from direction
    sim_rotation = quat.normalize(quat.between(np.array([1, 0, 0]), sim_direction))

    # Transform first joints to be local to sim and append sim as root bone
    positions[:,0:1] = quat.mul_vec(quat.inv(sim_rotation), positions[:,0:1] - sim_position)
    rotations[:,0:1] = quat.mul(quat.inv(sim_rotation), rotations[:,0:1])
    
    positions = np.concatenate([sim_position, positions], axis=1)
    rotations = np.concatenate([sim_rotation, rotations], axis=1)
    
    # Save positions and rotations
    bvh['positions'] = positions
    bvh['rotations'] = np.degrees(quat.to_euler(rotations, order='xyz'))
    
    # Add offsets of new bone and adjust hips offset
    bvh['offsets'] = np.concatenate([[[0.0, 0.0, 0.0]], np.array(bvh['offsets'])], axis=0)
    bvh['offsets'][1] = [0.0, 0.0, 0.0]

    # Add simulation bone as parent and re-parent all other bones
    bvh['parents'] = np.concatenate([[-1], bvh['parents'] + 1])
    bvh['names'] = ['SimulationBone'] + bvh['names']

    return bvh


""" Loop Over Files """
for filename in files:
    if ('filtered' in filename): continue
    print('Loading "%s" ...' % (filename))
    bvh_data = generate_sim_bone(bvh=bvh.load(filename))  
    new_filename = os.path.basename(filename)[:-4]+ '_filtered.bvh'
    new_file_name = os.path.join(os.path.dirname(__file__), "..", "data", "mocap", "lafan1_filtered", new_filename)
    bvh.save(new_file_name, bvh_data, frametime=bvh_data['frametime'], save_hip_position=True)