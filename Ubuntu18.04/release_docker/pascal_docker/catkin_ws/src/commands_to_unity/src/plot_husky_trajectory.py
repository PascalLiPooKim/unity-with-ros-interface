#!/usr/bin/env python

import rospy
import rosbag
import rospkg
import matplotlib.pyplot as plt
import sys, os
from os import listdir

#  Function to read rosbags and plot the recorded trajectory of Husky
def plot_traj():
    rospack = rospkg.RosPack()
    path = rospack.get_path("commands_to_unity")
    path = rospack.get_path("custom_husky_gazebo")

    # Read all .bag files from folder
    for file in listdir(path + "/bags"):
        bag_path = os.path.join(path + "/bags", file)
        bag = rosbag.Bag(bag_path)

        x = []
        y = []
        z = []

        # Append a and y coordinates of Husky to their respective list
        for topic, msg, t in bag.read_messages(topics=["/ground_truth/state"]):
            position = msg.pose.pose.position
            x.append(position.x)
            y.append(position.y)

        # Plot the graph
        plt.figure()
        plt.plot(x, y, '-x', label="Husky Trajectory")
        plt.legend()
        plt.title("Trajectory of Husky in " + file.replace(".bag",""))
        plt.grid()
        plt.show()

if __name__ == '__main__':
      rospy.init_node('traj_plotter')
      plot_traj()