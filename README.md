# ros2_laser_scan_merger

A ROS 2 C++ package for **merging multiple LiDAR / LaserScan sensors into a single
filtered PointCloud2**, and optionally converting that merged cloud into a **virtual 2D LaserScan**.

## What this fork does (important)

Compared to the original repository, this fork:

- ✅ Merges **multiple LiDARs / scans into a single PointCloud2**
- ✅ Transforms all data into a **common robot frame** (e.g. `base_link`)
- ✅ Applies **CropBox filtering** to remove robot footprint points
- ✅ Applies **voxel downsampling** for performance
- ✅ Converts the merged cloud into a **single LaserScan** using `pointcloud_to_laserscan`
- ✅ Supports **RL-friendly scan preprocessing** (reduced FOV, no redundant angles)
- ❌ Does **not** publish a merged scan directly from C++ (conversion is delegated)

## Typical pipeline
Multiple LiDARs -> C++ cloud merger (this package) -> Filtered/merged PointCloud2 -> pointcloud_to_laserscan -> Merged LaserScan (navigation)


## Prerequisites
1. ROS2 (Tested on Humble)
2. Working LiDAR drivers
3. Rviz2 (optional)
4. [Pointcloud to Laserscan](https://github.com/ros-perception/pointcloud_to_laserscan)

## Build and install
```bash
git clone https://github.com/<your_fork>/ros2_laser_scan_merger.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Launching the merger + scan conversion
```bash
ros2 launch ros2_laser_scan_merger merge_2_scan_launch.py
```
This launch file:

-  starts the C++ cloud merger
-  starts 'pointcloud_to_laserscan' for the merged cloud only
-  publishes:
-  merged PointCloud2
-  merged LaserScan

## Scan FoV calibration (recommended)

By default, the LaserScan is generated with a full 360 degree angular range.
However, your physical LiDAR setup usually covers only a subset of angles.

To remove redundant scan bins:

### Step 1: Run the FoV inference tool
```bash
ros2 run ros2_laser_scan_merger infer_scan_fov.py
```

### Step 2: Drive the robot
- Rotate in place
- Drive near walls/obstacles
- Let it run for ~1 minute

### Step 3: Read the output
The tool prints:
- observed angular segments
- a recommended [angle_min, angle_max] with margin

### Step 4: Update scan parameters
Copy the suggested angles into your `pointcloud_to_laserscan` configuration and restart

## Configuration files
All configuration lives in `config/`

`params.yaml` (cloud merger)
Controls:

- input cloud topics
- target frame
- cropbox size (robot footprint removal)
- voxel size
- output cloud topic

`pcl2_to_laserscan.yaml`
Controls:

- scan angular range
- angular resolution
- height filtering
- range limits

Important notes:

- If `use_inf: false`, "no return" is encoded as `range_max + inf_epsilon`
- Height filtering is relative to the target frame (e.g. `base_link`)