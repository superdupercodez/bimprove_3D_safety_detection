import glob
import open3d as o3d
import os
import numpy as np
import math

data_root_path = "./"

def visualize(pcd, labels):
    viz_items = [pcd]
    for label_attr in labels:
        ##Values    Name      Description
        #----------------------------------------------------------------------------
        #   1    type         Describes the type of object: 'Car', 'Van', 'Truck',
        #                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
        #                     'Misc' or 'DontCare'
        #   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
        #                     truncated refers to the object leaving image boundaries
        #   1    occluded     Integer (0,1,2,3) indicating occlusion state:
        #                     0 = fully visible, 1 = partly occluded
        #                     2 = largely occluded, 3 = unknown
        #   1    alpha        Observation angle of object, ranging [-pi..pi]
        #   4    bbox         2D bounding box of object in the image (0-based index):
        #                     contains left, top, right, bottom pixel coordinates
        #   3    dimensions   3D object dimensions: height, width, length (in meters)
        #   3    location     3D object location x,y,z in camera coordinates (in meters)
        #   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
        #   1    score        Only for results: Float, indicating confidence in
        #                     detection, needed for p/r curves, higher is better.
        #8-10  dims - hwl
        #11-13 ctr - xyz
        #14 rotation_y
        mesh_box = o3d.geometry.TriangleMesh.create_box(width=float(label_attr[10]), height=float(label_attr[9]), depth=float(label_attr[8]))
        new_left_lower_corner = np.array([float(label_attr[13]), -1.0 * float(label_attr[11]), (float(label_attr[12]) - 2.3)]) - mesh_box.get_center()            
        mesh_box = mesh_box.translate(new_left_lower_corner)                  
        yaw = float(label_attr[14])
        box_R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, (np.pi/2.0) - yaw])
        mesh_box = mesh_box.rotate(box_R)
        
        bounding_box_vertices = mesh_box.get_oriented_bounding_box()
        bounding_box_vertices.color = [1, 0, 0]
        
        viz_items.append(bounding_box_vertices)        
    o3d.visualization.draw_geometries(viz_items)

def rotate_all(pcd, labels, angle_rad=np.pi/2):
    R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, angle_rad])
    R_center = pcd.get_center()
    pcd.rotate(R)
    for label_attr in labels:
        yaw = float(label_attr[14]) - angle_rad
        label_attr[14] = str(yaw)
        center = np.array([float(label_attr[13]), -1.0 * float(label_attr[11]), (float(label_attr[12]))])
        new_center = np.dot(R, center)
        label_attr[13] = str(new_center[0])
        label_attr[11] = str(-1.0 * new_center[1])
        label_attr[12] = str(new_center[2])
    return pcd, labels

def save_new(ply, lbl, pcd, labels, prefix='gen_'):
    ply_path, ply_basename = os.path.split(ply)
    lbl_path, lbl_basename = os.path.split(lbl)
    ply_basename = prefix + ply_basename
    lbl_basename = prefix + lbl_basename
    ply = os.path.join(ply_path, ply_basename)
    lbl = os.path.join(lbl_path, lbl_basename)
    o3d.io.write_point_cloud(ply, pcd)
    f = open(lbl, "w")
    f.seek(0)
    lines = []
    for label in labels:
        label.append('\n')
        lines.append(' '.join(label))
    f.writelines(lines)
    f.truncate()                 
    f.close()
   
plys = glob.glob(os.path.join(data_root_path = "./", "*", "clouds", "*.ply"))
lbls = glob.glob(os.path.join(data_root_path = "./", "*", "labels", "*.txt"))

circle_divider = 6
angle_rads = np.arange(np.pi/circle_divider, 2*np.pi, np.pi/circle_divider)

for angle_rad in angle_rads:
    for ply in plys:
        print(ply, f'rot_{angle_rad}_')
        lbl = ply.replace(".ply", ".txt").replace("clouds", "labels")
        if lbl in lbls:
            pcd = o3d.io.read_point_cloud(ply)
            viz_shit = [pcd]
            f = open(lbl, "r+")
            lines = f.readlines()
            labels = []
            for line in lines:
                label_attr = line.split()
                labels.append(label_attr)
            pcd, labels = rotate_all(pcd, labels, angle_rad)
            save_new(ply, lbl, pcd, labels, prefix=f'rot_{angle_rad}_')
            #visualize(pcd, labels)
            f.close()


               
                