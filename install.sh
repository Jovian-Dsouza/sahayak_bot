#!/bin/bash

#Install Ros navigation stack
sudo apt-get install ros-melodic-gmapping \
ros-melodic-teleop-twist-keyboard \
ros-melodic-navigation \
ros-melodic-tf2-sensor-msgs \
ros-melodic-amcl \
ros-melodic-map-server -y 

#Install moveit
sudo apt install ros-melodic-moveit -y
sudo apt install ros-melodic-moveit-ros-planning -y
sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers -y
sudo apt-get install ros-melodic-object-recognition-msgs -y

#Install python PCL
sudo apt install python-pip
pip install --upgrade pip

pip install cython==0.26.1
pip install numpy==1.16.6

sudo apt-get update -y
sudo apt-get install libpcl-dev -y
sudo apt-get install pcl-tools -y

mkdir ~/python-pcl && cd ~/python-pcl
git clone https://github.com/Jovian-Dsouza/python-pcl.git
cd ~/python-pcl/python-pcl
python setup.py build
python setup.py build_ext -i
sudo python setup.py install
sudo rm -r ~/python-pcl 

#Install scikit learn
pip install scikit-learn==0.18
pip install matplotlib
