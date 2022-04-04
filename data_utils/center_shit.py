import glob
import open3d as o3d
import os

plys = glob.glob(os.path.join("2nd_5by5chunks", "validation", "clouds", "*.ply"))
ply_folders = [glob.glob(os.path.join("5by5chunks", "training", "clouds", "*.ply")),
               glob.glob(os.path.join("5by5chunks", "validation", "clouds", "*.ply")),
              ]

for ply_folder in ply_folders:
    for ply_file in ply_folder:
        pcd = o3d.io.read_point_cloud(ply_file)
        lbl_file = ply_file.replace(".ply", ".txt").replace("clouds", "labels")
        #downpcd = pcd.voxel_down_sample(voxel_size=0.005)
        #o3d.visualization.draw_geometries([pcd])
        #random_down_sample(self, sampling_ratio)
        #print(len(pcd.points))
        if len(pcd.points) > 0:
            ctr = pcd.get_center()
            min_z = pcd.get_min_bound()[2]
            pcd = pcd.translate([-1.0 * ctr[0], -1.0 * ctr[1], -1.0 * min_z])
            o3d.io.write_point_cloud(ply_file, pcd)
            if os.path.exists(lbl_file):
                #Open label file and shift all boxes x and y to a new center
                f = open(lbl_file, "r+")
                lines = f.readlines()
                #print(lines)
                for idx, line in enumerate(lines):
                    if len(line) > 0:
                        #['net', '0', '0', '0', '0', '0', '0', '0', '1.67348765', '0.20681611', '2.11083571', '11.59029332', '5.39522072', '-37.46305614', '2.04914062']
                        #ctr [-37.23337596 -11.57550914   1.44319308]
                        items = line.split()                    
                        #print(items, ctr)
                        items[13] = str(round(float(items[13]) - ctr[0], 8))
                        items[11] = str(round(float(items[11]) + ctr[1], 8))
                        items.append('\n')
                        #print(' '.join(items))
                        lines[idx] = ' '.join(items)
                        #new_lines.append(' '.join(items))
                    #print('*')
                    #print(lines)           
                f.seek(0)
                f.writelines(lines)
                f.truncate()                 
                f.close()
                #exit()
        
        
        
        
