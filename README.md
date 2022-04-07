# BIMProve foundation for training point cloud safety object detection models

This is a foundation for training a 3D safety related object detection capable point cloud ML model. The model architecture intended to be applied is [PointPillars](https://arxiv.org/abs/1812.05784) available in the [Open3D-ML](https://github.com/isl-org/Open3D-ML) machine learning extension for [Open3D](https://github.com/isl-org/Open3D). Other model architectures and point cloud processing libraries may be compatible too. See Open3D and Open3D-ML specific instructions for setting them up. This is only a BIMProve project specific addition in order to consume BIMProve point clouds and produce domain specific 3D object detection model. BIMProve data not included here. Open3D-ML supports either PyTorch or TensorFlow. At the time of writing this PyTorch with more stabile AdamW optmizer implementation allows higher precision with less training in this case. Unlike the typical LIDAR scanned point clouds, BIMPRove point clouds are stiched and dense.

Data folders are expected to be structured as:
```
training -|
          --clouds
          --labels

validation -|
            --clouds
            --labels
```
Where `clouds` contains the point cloud files and the `labels` contains the associated label files.

This repository includes
 - [`bimprovekitti.py`](ml3d/datasets/bimprovekitti.py) BIMProve specific [KITTI](https://github.com/bostondiditeam/kitti/blob/master/resources/devkit_object/readme.txt)-labelled point cloud data feeder for [Open3D-ML](https://github.com/isl-org/Open3D-ML)
 - [`__init__.py`](ml3d/datasets/__init__.py) Init file with the added BIMProve dataset
 - [`vis_pc.py`](vis_pc.py) Visualizer for the dataset
 - [`V0_pointpillars_bimprovekitti.yml`](V0_pointpillars_bimprovekitti.yml) - V0 version of the PointPillars model and data configuration
 
## Installation
- Install the required Open3D and Open3D-ML dependencies
- copy `bimprovekitti.py` and `__init__.py` to the intended location
- switch root to the local copy of the Open3D-ML with the BIMProve extendion by issuing
```
#source ./set_open3d_ml_root.sh`
```
- generate the initial bounding boxes cache file (Only for the first time if no changes in the data)
```
#python3 scripts/collect_bboxes.py --dataset_path <path_to_your_dataset> --dataset_type BimProveKITTI
```

## Training
- train your model with the BIMProve dataset and the provided configurations (See Open3D-ML for further instructions and alternative approaches) using
```
#python3 scripts/run_pipeline.py torch -c ./V0_pointpillars_bimprovekitti.yml --dataset_path <path_to_your_dataset> --pipeline ObjectDetection
```
Or using Tensorflow instead of PyTorch

```
#python3 scripts/run_pipeline.py tf -c ./V0_pointpillars_bimprovekitti.yml --dataset_path <path_to_your_dataset> --pipeline ObjectDetection
```
For questions and comments contact [Tommi Aihkisalo](mailto:tommi.aihkisalovtt.fi)

