#!/usr/bin/python3.8
#
from fixed_wing_OOP import fixed_wing
import os

blah = fixed_wing(landing_lat=43.645620,landing_lon=-79.384368,landing_alt=5,landing_bearing=90.0)
path = os.path.join(os.path.expanduser("~"), "catkin_ws/src/px-lib-fixed-wing/src/mission.txt")
blah.main(path)