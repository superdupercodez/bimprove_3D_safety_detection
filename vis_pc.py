import open3d.ml.torch as ml3d  # or open3d.ml.tf as ml3d

dataset = ml3d.datasets.BIMProveKITTI(dataset_path='/media/tai/datasink/bimprove_data/pointcs/bimprove_scanned/sideview_chunked/flipped_5by5chunks')
#dataset = ml3d.datasets.ShitSet(dataset_path='/media/tai/datasink/bimprove_data/pointcs/bimprove_scanned/sideview_chunked/small_5by5chunks')
#dataset = ml3d.datasets.ShitSet(dataset_path='/media/tai/datasink/bimprove_data/pointcs/bimprove_scanned/sideview_chunked/5by5chunks')
#dataset = ml3d.datasets.ShitSet(dataset_path='/media/tai/datasink/bimprove_data/pointcs/bimprove_scanned/stairs_rand_sampled/')
#dataset = ml3d.datasets.ShitSet(dataset_path='/media/tai/datasink/bimprove_data/pointcs/bimprove_scanned/stairs_chunked/')
#dataset = ml3d.datasets.ShitSet(dataset_path='./new_sideview/')
#dataset = ml3d.datasets.KITTI(dataset_path='/media/tai/datasink/bimprove_data/pointcs/datasets/KITTI/Kitti')
#dataset = ml3d.datasets.KITTI(dataset_path='/media/tai/datasink/bimprove_data/pointcs/datasets/KITTI/MiniKitti')
#dataset = ml3d.datasets.SemanticKITTI(dataset_path='/media/tai/datasink/bimprove_data/pointcs/datasets/SemanticKITTI/')

#split = 'training'
#split = 'validation'
split = 'all'
#split = 'testing'

all_split = dataset.get_split(split)
vis = ml3d.vis.Visualizer()
vis.visualize_dataset(dataset, split, indices=range(all_split.__len__()))
#vis.visualize_dataset(dataset, 'all', indices=range(3))
