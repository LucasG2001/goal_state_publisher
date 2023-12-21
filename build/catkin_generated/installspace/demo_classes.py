#!/usr/bin/python3.8
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import rospy
from moveit_msgs.msg import PlanningScene


