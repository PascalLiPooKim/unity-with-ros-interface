#!/usr/bin/env python

import rospy
import rosbag
import rospkg
# import Tkinter 
import matplotlib.pyplot as plt
import sys, os
from os import listdir


#  Function to read rosbags and plot the recorded trajectory of Husky
def plot_traj():
    rospack = rospkg.RosPack()
    # path = rospack.get_path("commands_to_unity")
    # path = rospack.get_path("custom_husky_gazebo")

    # print(path)
    # Read all .bag files from folder
    # for file in listdir(path + "/bags"):
    path_list = []

    plt.figure()
    for i in range(2):
        # bag_path = os.path.join(path + "/bags", file)
        bag_path = "/home/vr-pc/Desktop/husky_course_3_hard_5FAS_V_66_2021-08-18-14-17-35.bag"
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
        
        plt.plot(x, y, '--', label="Husky Trajectory for Candidate {}".format(i))
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Trajectory of Husky using Vision only")
    plt.grid()
    plt.show()

if __name__ == '__main__':
      rospy.init_node('traj_plotter')
      plot_traj()