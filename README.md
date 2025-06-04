# 🦾 AI-Worker

## **1. Introduction**

This is the AI-Worker project repository. It supports the control of the **Follower** device (hand, lift, and neck) as well as the **Leader** device (arms and hands).

This package supports **ROS 2 Jazzy** and **Gazebo Harmonic** on Ubuntu 24.04.


## **2. Installation Methods**

You can choose between two installation methods:

### **Option 1: Using Docker (Recommended)**

This method provides an isolated environment with all dependencies pre-installed.

1. **Install Docker and Docker Compose**
   Follow the official Docker installation guide: [Install Docker Engine](https://docs.docker.com/engine/install/)

2. **Clone the Repository**
   ```bash
   git clone https://github.com/ROBOTIS-GIT/ai_worker.git
   cd ai_worker
   ```

3. **Container Management**
   The repository includes a container management script with the following commands:

   ```bash
   # Show help
   ./docker/container.sh help

   # Start container with Gazebo support
   ./docker/container.sh start with_gz

   # Start container without Gazebo support
   ./docker/container.sh start without_gz

   # Enter the running container
   ./docker/container.sh enter

   # Stop and remove the container
   ./docker/container.sh stop
   ```

   [***Note***] <u>When stopping the container, you'll be asked for confirmation as this will remove all unsaved data in the container.</u>

4. **Data Persistence**
   The container maps the following directories for data persistence:
   - `./docker/workspace:/workspace` - The workspace directory inside the docker folder is mapped to `/workspace` inside the container
   
   [***Important***] <u>Data Persistence Rules:
   - Data in `/workspace` inside the container is saved to `docker/workspace` on your host
   - Container restart (using `docker restart`) maintains all data
   - Container removal (using `container.sh stop`) will remove all data except what's in the mapped `/workspace` directory
   - Always save your work in the `/workspace` directory to ensure it persists after container removal</u>


### **Option 2: Host Installation**

Follow these steps if you prefer to install directly on your host system:

1. **Prerequisites**

   - **Supported ROS Version**
     ![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
     This package is compatible only with **ROS 2 Jazzy**. Ensure that ROS 2 Jazzy is properly installed.

   - **USB Port Permissions**
     To enable communication with the hardware, add your user to the `dialout` group:
     ```bash
     sudo usermod -aG dialout $USER
     ```
     **A login and logout are required.**

2. **Install Required Packages**

Install the following dependencies:

```bash
sudo apt-get update && sudo apt-get install -y \
    libboost-all-dev \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-tf-transformations \
    ros-jazzy-gz* \
    ros-jazzy-pal-statistics \
    python3-tk
sudo apt-get install -y ros-jazzy-moveit-* --no-install-recommends
```

3. Clone the Repository**

Navigate to your ROS 2 workspace and clone the repository:

```bash
cd ~/${WORKSPACE}/src
```

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
```

4. **Build the Package**

Compile the package using `colcon`:

```bash
cd ~/${WORKSPACE}
colcon build --symlink-install
```

5. **Source the Workspace**

```bash
source ~/${WORKSPACE}/install/setup.bash
```


## **3. Execution Commands**

### **Step 1: Choose Your Operating Mode**

#### **1️⃣ Leader-Follower Mode**

For **leader-follower functionality**, use:

```bash
ros2 launch ffw_bringup ffw_teleop_with_rh.launch.py
```

Ensure proper connection and detection of leader and follower devices.

**Operation Sequence upon Launch**

1. Establish hardware connection with the ***follower*** device.
2. Move the *follower* device to the **initial position**.( Standing position with arms outstretched)
3. Establish hardware connection with the ***leader*** device.

#### **2️⃣ Standalone Mode**

For **standalone mode**, launch:

```bash
ros2 launch ffw_bringup hardware_follower_teleop_with_rh.launch.py
```

*Only the follower is connected to the hardware interface.



```
ros2 launch ffw_bringup hardware_leader_with_rh.launch.py
```

*Only the Leader is connected to the hardware interface.



```
ros2 launch ffw_bringup hardware_follower_with_rh.launch.py
```

*Mode to operate the follower via MoveIt

#### **3️⃣ Gazebo Simulation Mode**

For **Gazebo simulation mode**, launch:

```bash
ros2 launch ffw_bringup gazebo.launch.py
```

Ensure that Gazebo Harmonic is properly installed and configured before running the simulation.


### **Step 2: Extend Functionality**

#### **1. MoveIt! Launch**

Enable MoveIt functionality for advanced motion planning in RViz:
*Before operation, either `hardware_follower_standalone.launch.py` or Gazebo must be launched."

```bash
ros2 launch ffw_moveit_config moveit_core.launch.py
```

Move interactive markers to position the robotic arm, then click **Plan** and **Execute**.

#### **2. GUI Teleop**

While `hardware_follower_teleop_with_rh.launch.py` is running, you can execute the GUI with the command

```bash
ros2 run ffw_teleop keyboard_control_with_rh.py`.
```

