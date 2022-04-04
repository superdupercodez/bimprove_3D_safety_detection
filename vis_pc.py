import open3d.ml.torch as ml3d  # or open3d.ml.tf as ml3d

root_path = "/home/tai/Desktop/bimprove_data/pointcs/bimprove_scanned/sideview_chunked/generated_5by5_chunks/"
dataset_path = root_path

dataset = ml3d.datasets.BIMProveKITTI(dataset_path=dataset_path)

#split = 'training'
#split = 'validation'
#split = 'testing'
split = 'all'
indices = range(500)
#indices = split.__len__()
vis = ml3d.vis.Visualizer()
vis.visualize_dataset(dataset, split, indices=indices)
