# G1 Humanoid Robot LiDAR Mapping & Localization with FAST-LIO

This project is based on the Unitree G1 humanoid robot, using the Livox Mid-360 LiDAR for mapping and localization. It adopts the FAST-LIO algorithm, supports ROS1 Noetic, and is designed for Ubuntu 20.04.

## 🧩 Requirements

- Ubuntu 20.04  
- ROS1 Noetic  
- Livox Mid-360 LiDAR  
- CMake >= 3.18

## 🔧 Build Instructions

### 1. Install Livox SDK

```bash
cd ~/catkin_ws/Livox-SDK/
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 2. Build the ROS workspace

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

## 🗺️ Run Mapping

### Play rosbag data

```bash
rosbag play map.bag
```

### Launch the mapping node

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

⚠️ The default map is inverted (due to upside-down LiDAR installation), so you need to modify the mapping code to ensure the **Z-axis points upward**.

## 📍 Run Localization

### Play localization test data

```bash
rosbag play localization_test.bag
```

### Launch the localization node

```bash
source devel/setup.bash
roslaunch mid360_locate localization.launch
```

## 🛠️ Assignment Tasks

1. **Modify the mapping code**: ensure that the generated map has Z-axis pointing up  
2. **Modify the localization code**: use your own generated map instead of the default one

---

## 📌 Notes

- Make sure the point cloud orientation is correct (recommended to flip Y/Z axis accordingly)
- If ROS cannot recognize the `.pcd` map file, consider exporting a standard format using CloudCompare
- ICP or NDT can be used for initial map alignment

---

## 🤝 Credits

This project is based on FAST-LIO and Livox-SDK. Thanks to the open-source contributors.