#!/usr/bin/env python

import rospy
import rosbag
import rospkg
import matplotlib.pyplot as plt
import sys, os
from os import listdir

def plot_traj():
    rospack = rospkg.RosPack()
    path = rospack.get_path("commands_to_unity")

    for file in listdir(path + "/bags"):
        bag_path = os.path.join(path + "/bags", file)
        bag = rosbag.Bag(bag_path)

    # bag = rosbag.Bag(path + "/husky_traj.bag")
    # bag = rosbag.Bag(path + "/bags/traj_test.bag")

        x = []
        y = []
        z = []


        for topic, msg, t in bag.read_messages(topics=["/ground_truth/state"]):
            position = msg.pose.pose.position
            x.append(position.x)
            y.append(position.y)

        plt.figure()
        plt.plot(x, y, '-x', label="Husky Trajectory")
        plt.legend()
        plt.title("Trajectory of Husky in " + file.replace(".bag",""))
        plt.grid()
        plt.show()

if __name__ == '__main__':
      rospy.init_node('traj_plotter')
      plot_traj()