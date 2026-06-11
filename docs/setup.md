# Environment Setup

## 1. Clone

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b jazzy https://github.com/nguyenhau442001/Differential_Drive_Mobile_Robot.git
```

## 2. macOS (Tahoe)

### 2.1 Create environment

```bash
conda create -n ros2_jazzy
conda activate ros2_jazzy
conda install -c conda-forge -c robostack-jazzy ros-jazzy-desktop ros-jazzy-ros-gz
```

### 2.2 Add to ~/.zshrc

```bash
conda activate ros2_jazzy
source ~/ros2_ws/install/setup.zsh
export GZ_IP=127.0.0.1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_RENDER_ENGINE_GUI=ogre
export GZ_SIM_RENDER_ENGINE_SERVER=ogre
export GZ_SIM_RESOURCE_PATH=$CONDA_PREFIX/share/nav2_minimal_tb4_sim/worlds
```

### 2.3 Build

```bash
cd ~/ros2_ws && colcon build && source install/setup.zsh
```

## 3. Ubuntu 24.04

```bash
cd ~/ros2_ws/src/Mobile_Robot
chmod +x setup_ubuntu.sh && ./setup_ubuntu.sh
```
