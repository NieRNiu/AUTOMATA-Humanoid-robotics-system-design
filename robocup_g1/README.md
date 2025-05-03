# 安装与使用指南

## 一. 环境必备(非conda环境下，conda环境需要设置python路径)
### ROS1 环境安装，版本为 Noetic
使用鱼香ROS一键安装指令:
wget http://fishros.com/install -O fishros && . fishros

安装完毕后重启系统。

### Gazebo 环境安装, 版本为 Gazebo Classic
参照以下安装流程：
#### 1. 更新系统
sudo apt update && sudo apt upgrade -y
#### 2. 安装依赖库
sudo apt install -y \
    cmake \
    curl \
    git \
    libfreeimage-dev \
    libprotoc-dev \
    protobuf-compiler \
    libignition-math6-dev \
    libsqlite3-dev \
    libtinyxml2-dev \
    libgflags-dev \
    libavformat-dev \
    libavcodec-dev
    
#### 3. 添加 Gazebo 官方软件源
sudo apt install -y wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
#### 4. 安装 Gazebo
sudo apt update
sudo apt install -y gazebo11 libgazebo11-dev
#### 5. 配置环境变量
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
#### 6. 验证安装
gazebo worlds/empty.world

安装完毕后重启系统。

### Eigen3.4 安装，版本为3.4
```bash
cd robocup_g1/eigen-3.4.0
mkdir build
cd build
cmake ..
sudo make install
```
### RealSense SDK 安装
```bash
sudo apt-get install ros-noetic-realsense2-camera
```

### Ignition-math 安装
```bash
sudo apt install libignition-math4-dev
```
### 如果有遇到以下依赖项缺少，可以参考安装，没有的话不需要安装：（可选）
```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-joint-state-controller ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-xacro
```
## 二. 编译流程
```bash
cd ~/robocup_g1/
catkin_make
```

## 三. 启动流程

```bash
source ~/robocup_g1/devel/setup.bash
roslaunch unitree_guide gazebo.launch
```

```bash
source ~/robocup_g1/devel/setup.bash
./devel/lib/unitree_guide/junior_ctrl
```

## 四. 模式切换

在 `junior_ctrl` 主界面输入 `2`，控制机器人从 **State_Passive** 切换到 **fixed stand**。回到 Gazebo 主界面，按下暂停键，然后在主菜单中选择 **Edit/Reset Model Poses** 以重置机器人的位姿。在 `junior_ctrl` 主界面输入 `4`，控制机器人从 **fixed stand** 切换到 **LOCOMOTION**。回到 Gazebo 主界面，点击播放键，重启应用。


## 五. 按键控制
### 控制键说明

- **前后运动**：
  - **`W` 键**：向前运动
  - **`S` 键**：向后运动

- **左右平移**：
  - **`A` 键**：向左平移
  - **`D` 键**：向右平移

- **左右旋转**：
  - **`J` 键**：向左旋转
  - **`L` 键**：向右旋转
  

### 六. 接口控制说明
运行example中的接口控制demo可以实现对机器人G1的程序控制。




