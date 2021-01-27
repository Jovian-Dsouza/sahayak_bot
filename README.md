# **_Sahayak Bot_**

## Installation

### Cloning this repo 
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src && git clone https://github.com/Jovian-Dsouza/sahayak_bot
```

###  ROS installation and Prerequisites

- Follow the Official Installation instruction from the [ROS-Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) website. 

- or use the `ros_install.sh`, which is configured to install ROS melodic on Ubuntu 18.04 

```bash
cd ~/catkin_ws/src/sahayak_bot
sudo chmod +x ros_install.sh
./ros_install.sh

#Install Prerequisites
sudo chmod +x install.sh
./install.sh
```

### Creating alias 

- Append these lines to `.bashrc`

```bash
source ~/catkin_ws/devel/setup.bash
alias cm="catkin_make -C ~/catkin_ws && source ~/catkin_ws/devel/setup.bash"
```

- For `zsh` append these lines to `.zshrc`

```bash
source /opt/ros/melodic/setup.zsh
source ~/catkin_ws/devel/setup.zsh
alias cm="catkin_make -C ~/catkin_ws && source ~/catkin_ws/devel/setup.zsh"
```

- Finally source `.bashrc'

```bash
source ~/.bashrc
```

### Build the workspace 

- We are using `catkin_make` to build the workspace. 

```bash
cd ~/catkin_ws
catkin_make

#OR use the alias cm to build and source the workspace
cm
```

### Testing

```
roslaunch ebot_description task5.launch
```