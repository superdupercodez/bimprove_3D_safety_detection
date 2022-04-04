import open3d as o3d
import numpy as np
import pandas as pd
import os, glob, pickle
from pathlib import Path
from os.path import join, exists, dirname, abspath, isdir
from tqdm import tqdm
import logging

from .utils import DataProcessing, get_min_bbox, BEVBox3D
from .base_dataset import BaseDataset, BaseDatasetSplit
from ..utils import make_dir, DATASET

log = logging.getLogger(__name__)

from .base_dataset import BaseDataset

class BIMProveKITTI(BaseDataset):
    def __init__(self, dataset_path, cache_dir='./logs/cache', name="BIMProveKITTI", **kwargs):
        super().__init__(dataset_path=dataset_path, cache_dir=cache_dir, use_cache=False, name=name, **kwargs)
        # read file lists.
        cfg = self.cfg
        self.name = cfg.name
        self.dataset_path = cfg.dataset_path       

        self.train_files = glob.glob(join(self.dataset_path, 'training', 'clouds', '*.ply'))
        self.val_files = glob.glob(join(self.dataset_path, 'validation', 'clouds', '*.ply'))
        self.all_files = self.train_files + self.val_files
        
        self.num_classes = 2
        self.label_to_names = self.get_label_to_names()
        
    @staticmethod
    def get_label_to_names():
        """Returns a label to names dictionary object.

        Returns:
            A dict where keys are label numbers and values are the corresponding
            names.
        """
        label_to_names = {
            0: 'net',
            1: 'barrier',
            2: 'DontCare',
        }
        return label_to_names
        
    def get_split(self, split):        
        return BIMProveKITTISplit(self, split=split)

    def get_split_list(self, split):
        if split in ['train', 'training']:
            return self.train_files
        elif split in ['val', 'validation']:
            return self.val_files
        elif split in ['all']:
            return self.train_files + self.val_files
        else:
            raise ValueError("Invalid split {}".format(split))
    
    @staticmethod
    def read_labels(path, R=None):
        log.debug(f'* Opening {path}')
        if not Path(path).exists():
            log.debug(f'* Not found file {path}')
            return []
        
        with open(path, 'r') as f:
            labels = f.readlines()
            
        objects = []
        for label in labels:
            log.debug(f'Reading label {label}')
            '''            
            F#  #Values    Name      Description
            ----------------------------------------------------------------------------
            0       1    type       Describes the type of object: 'Car', 'Van', 'Truck',
                                    'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                                    'Misc' or 'DontCare'
            1       1    truncated  Float from 0 (non-truncated) to 1 (truncated), where
                                    truncated refers to the object leaving image boundaries
            2       1    occluded   Integer (0,1,2,3) indicating occlusion state:
                                        0 = fully visible, 1 = partly occluded
                                        2 = largely occluded, 3 = unknown
            3       1    alpha      Observation angle of object, ranging [-pi..pi]
            4-7     4    bbox       2D bounding box of object in the image (0-based index):
                                    contains left, top, right, bottom pixel coordinates
            8-10    3    dimensions 3D object dimensions: height, width, length (in meters)
            11-13   3    location   3D object location x,y,z in camera coordinates (in meters)
            14      1    rotation_y Rotation ry around Y-axis in camera coordinates [-pi..pi]
            15      1    score      Only for results: Float, indicating confidence in
                                    detection, needed for p/r curves, higher is better.
            '''
            label = label.strip().split(' ')
            name = label[0] if label[0] in BIMProveKITTI.get_label_to_names().values() else 'DontCare'           
            size = [float(label[9]), float(label[8]), float(label[10])]  # w,h,l            
            #This is funked up - need to substract 2.3 from the Z coordinates in Kitti labeling system
            center = np.array([float(label[13]), -1.0 * float(label[11]), (float(label[12]) - 2.3)])                        
            yaw = float(label[14]) - np.pi
            yaw = yaw - np.floor(yaw / (2 * np.pi) + 0.5) * 2 * np.pi                                   
            objects.append(Object3d(name, center, size, yaw))
            
            log.debug(f"**************** {path} ****************") 
            log.debug(f"CENTER IS {center}")
            log.debug(f"SIZE   IS {size}")
            log.debug(f"YAW    IS {yaw}")
        return objects

    @staticmethod
    def _extend_matrix(mat):
        mat = np.concatenate(
            [mat, np.array([[0., 0., 1., 0.]], dtype=mat.dtype)], axis=0)
        return mat

    @staticmethod
    def read_calib(path):
        """Reads calibiration for the dataset. You can use them to compare
        modeled results to observed results.

        Returns:
            The camera and the camera image used in calibration.
        """
        assert Path(path).exists()

        with open(path, 'r') as f:
            lines = f.readlines()

        obj = lines[0].strip().split(' ')[1:]
        P0 = np.array(obj, dtype=np.float32).reshape(3, 4)

        obj = lines[1].strip().split(' ')[1:]
        P1 = np.array(obj, dtype=np.float32).reshape(3, 4)

        obj = lines[2].strip().split(' ')[1:]
        P2 = np.array(obj, dtype=np.float32).reshape(3, 4)

        obj = lines[3].strip().split(' ')[1:]
        P3 = np.array(obj, dtype=np.float32).reshape(3, 4)

        P0 = BIMProveKITTI._extend_matrix(P0)
        P1 = BIMProveKITTI._extend_matrix(P1)
        P2 = BIMProveKITTI._extend_matrix(P2)
        P3 = BIMProveKITTI._extend_matrix(P3)

        obj = lines[4].strip().split(' ')[1:]
        rect_4x4 = np.eye(4, dtype=np.float32)
        rect_4x4[:3, :3] = np.array(obj, dtype=np.float32).reshape(3, 3)

        obj = lines[5].strip().split(' ')[1:]
        Tr_velo_to_cam = np.eye(4, dtype=np.float32)
        Tr_velo_to_cam[:3] = np.array(obj, dtype=np.float32).reshape(3, 4)

        world_cam = np.transpose(rect_4x4 @ Tr_velo_to_cam)
        cam_img = np.transpose(P2)

        return {'world_cam': world_cam, 'cam_img': cam_img}    
    
    def is_tested(self, attr):
        # checks whether attr['name'] is already tested.
        pass

    def save_test_result(self, results, attr):
        test_result_folder = "./logs/test_shit/"
        make_dir(test_result_folder)
        for attr, res in zip(attrs, results):
            name = attr['name']
            path = join(test_result_folder, name + '.txt')
            f = open(path, 'w')
            for box in res:
                f.write(box.to_kitti_format(box.confidence))
                f.write('\n')


class BIMProveKITTISplit():
    def __init__(self, dataset, split='train'):
        self.split = split
        self.path_list = dataset.get_split_list(split)
        self.split = split
        self.dataset = dataset
        log.info("Found {} pointclouds for {}".format(len(self.path_list), split))
        
    def __len__(self):
        return len(self.path_list)

    def get_data(self, idx):
        pc_path = self.path_list[idx]
        label_path = pc_path.replace('clouds', 'labels').replace('.ply', '.txt')                
        
        point_c = o3d.io.read_point_cloud(pc_path)    
        log.debug(f"Retrieved {idx} with {len(point_c.points)} points from {pc_path}")
        points = np.asarray(point_c.points, dtype=np.float32)
                
        #Use red from RGB as intensity
        colors = np.asarray(point_c.colors, dtype=np.float32)                
        intensity = colors[:,0]
        points = np.insert(points, 3, intensity, axis=1)
        
        calib = self.dataset.read_calib('./data/000001.txt')
        bboxes = self.dataset.read_labels(label_path)
        
        return {'point': points, 'feat': None, 'calib': calib, 'bounding_boxes': bboxes}

    def get_attr(self, idx):
        path = self.path_list[idx]
        name = path.split('/')[-1]
        return {'name': name, 'path': path, 'split': self.split}

class Object3d(BEVBox3D):
    """Stores object specific details like bbox coordinates."""

    def __init__(self, name, center, size, yaw):
        super().__init__(center, size, yaw, name, 1.0)
        self.occlusion = 0.0

DATASET._register_module(BIMProveKITTI)
