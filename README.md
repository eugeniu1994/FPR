
# FPR — Fast Point Ranking  

Fast Point Ranking (FPR) provides **robust cloud voxelization and denoising** for LiDAR odometry and mapping in **adverse weather conditions**.  
It leverages range image projection and rank-based voxelization to select or average points per voxel, reducing noise while preserving geometric structure.  

![Fast Point Ranking](https://github.com/eugeniu1994/FPR/blob/c896008b1138366255c03d361d6a3ca314d78f84/paper-teaser.png)

## ✨ Features

- **Range image projection** for efficient organization of LiDAR points  
- **Rank estimation**   
- **Rank-based voxelization**:  
  - First point per voxel (highest rank)  
  - Weighted average of points (rank-weighted centroid)  
- **Scan cleaning**: drop low-rank/noisy points (configurable percentage)  
- Designed for **robust LiDAR odometry & mapping in rain, fog, or dust**  

---

## 🚀 Build Instructions

```sh
cd ~/catkin_ws/src/ #change this according to your system
git clone https://github.com/eugeniu1994/FPR.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="fpr"
source devel/setup.bash
```

▶️ Usage

Run the demo on your own bag file:
```sh
roslaunch fpr demo_bag.launch bag_file:=/media/eugeniu/T7/evo-bags/1_hesai-CPT_2024-07-25-12-48-43.bag
```

Run the demo on your own Adverse-Weather-Kitti-360:
```sh
roslaunch fpr demo_bin.launch
```

Point clouds affected by 🌧️ rain, ❄️ snow, and 🌫️ fog are available in the Adverse Weather KITTI-360 dataset

https://etsin.fairdata.fi/dataset/fcd18634-79dc-4151-be54-cd452ac3b7b6/data

⚙️ Configuration
Main parameters you can adjust (via launch/config files):

voxel_size: size of the voxel grid (in meters)

how_many_to_filter: percentage of lowest-rank points to drop (e.g. 10 = drop 10%)

use_weighted: use intensity-weighted centroid instead of single max-intensity point

Real-world snow scenario on Velodyne VLS 128 scans.
![Fast Point Ranking](https://github.com/eugeniu1994/FPR/blob/c896008b1138366255c03d361d6a3ca314d78f84/paper-teaser.png)

🛠️ TODO

Add support for different point types (PointXYZ, PointXYZI, etc.)

Integrate with scan registration (e.g. ICP)

📧 Maintainer
Eugeniu Vezeteu
📩 vezeteu.eugeniu@yahoo.com


