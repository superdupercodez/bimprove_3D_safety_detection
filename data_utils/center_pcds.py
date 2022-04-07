import glob
import open3d as o3d
import os

data_root_path = "./"

ply_folders = [glob.glob(os.path.join(data_root_path, "training", "clouds", "*.ply")),
               glob.glob(os.path.join(data_root_path, "validation", "clouds", "*.ply")),
              ]

for ply_folder in ply_folders:
    for ply_file in ply_folder:
        pcd = o3d.io.read_point_cloud(ply_file)
        lbl_file = ply_file.replace(".ply", ".txt").replace("clouds", "labels")
        if len(pcd.points) > 0:
            ctr = pcd.get_center()
            min_z = pcd.get_min_bound()[2]
            pcd = pcd.translate([-1.0 * ctr[0], -1.0 * ctr[1], -1.0 * min_z])
            o3d.io.write_point_cloud(ply_file, pcd)
            if os.path.exists(lbl_file):
                f = open(lbl_file, "r+")
                lines = f.readlines()
                for idx, line in enumerate(lines):
                    if len(line) > 0:
                        items = line.split()                    
                        items[13] = str(round(float(items[13]) - ctr[0], 8))
                        items[11] = str(round(float(items[11]) + ctr[1], 8))
                        items.append('\n')
                        lines[idx] = ' '.join(items)
                f.seek(0)
                f.writelines(lines)
                f.truncate()                 
                f.close()

        
        
        
        
