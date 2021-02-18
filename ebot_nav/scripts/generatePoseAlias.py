#!/usr/bin/env python
from poses import poses
from tf.transformations import euler_from_quaternion
import rospkg

path = rospkg.RosPack().get_path('ebot_nav') + '/scripts/locations.sh'

with open(path, "w") as f:
    for key in poses.keys():
        print(key)
        p = poses[key].position
        q = poses[key].orientation
        x = p.x
        y = p.y
        ya = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        f.write('alias %s="roslaunch ebot_description arm_nav_test.launch x:=%0.4f y:=%0.4f ya:=%0.4f nav:=false"\n' % (key, x, y, ya))


with open(path, "a") as f:
    for key in poses.keys():
        # print(key)
        p = poses[key].position
        q = poses[key].orientation
        x = p.x
        y = p.y
        ya = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        f.write('alias nav_%s="roslaunch ebot_description nav_test.launch x:=%0.4f y:=%0.4f ya:=%0.4f"\n' % (key, x, y, ya))

print(">> Location file has been generated")        