# **_Sahayak Bot_**



## Welcome the eYRC 2020's Sahayak Bot theme !!!  

Please find the base package for this theme from this repository. 



> This package is tested on **Ubuntu 18, ROS-Melodic**. We recommend you to use this OS and ROS distribution only.



## Installation

### Prequites

```
sudo apt-get install ros-melodic-object-recognition-msgs
```


###  ROS installation

- Follow the Official Installation instruction from the [ROS-Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) website. We recommend to use the following command for **step 1.4**. This will ensure all the basic package availability and possibly avoid any "missing packages" errors.

```bash
sudo apt install ros-melodic-desktop-full
```

For those who are familiar with debugging in ROS, may use  `ros-melodic-*` of their choice.

- **Or** you can follow the instruction from the Task Book (mdbook) available on the [eYRC portal](https://portal.e-yantra.org/login).

### Additional Packages 

- "Teleop" package to control `ebot` or any model (all you would need to do is change the message type, do explore in your spare time) from your terminal. For those who have install `ros-melodic-desktop-full` don't need to install this package, since it already available. 

```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```

### Cloning this repository

Inside your `cakin_ws/src`, enter the following command to clone (download) this repository. 

```bash
git clone https://github.com/vishalgpt579/sahayak_bot.git
```


<!-- > **Note:** Since we have shifted from simple drive download to more sophisticated approach of sharing packages, there are some limitation in the normal approach, mainly the size of upload. However, this is solved by using Githhub's LFS (Large File System). And hence, you might face a little delay (depending upon your network) in downloading the files tracked by LSF.  -->

