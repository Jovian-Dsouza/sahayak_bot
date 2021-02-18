#!/usr/bin/env python
import rospy
# [ INFO] [1613394677.969269274]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...

# ******************************************
# /home/jovian/task5_ws/src/sahayak_bot/ebot_gazebo

# rospy.wait_for_service('/gazebo_gui/set_physics_properties')
rospy.sleep(1)
print('******************************************')
print('/home/jovian/task5_ws/src/sahayak_bot/ebot_gazebo')
rospy.wait_for_service('/link_attacher_node/detach')