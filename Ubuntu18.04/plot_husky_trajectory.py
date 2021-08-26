#!/usr/bin/env python

import rospy
import rosbag
import rospkg
# import Tkinter 
import matplotlib.pyplot as plt
import sys, os
from os import listdir
# import seaborn as sns



plt.style.use("ggplot")
params = {
   'axes.labelsize': 25,
   'font.size': 25,
   'font.family': 'sans-serif',
   'font.serif': 'Arial',
   'legend.fontsize': 15,
   'xtick.labelsize': 25,
   'ytick.labelsize': 25, 
   'figure.figsize': [25, 15],
   'axes.titlesize': 30
   } 
plt.rcParams.update(params)


#  Function to read rosbags and plot the recorded trajectory of Husky
def plot_traj_course_2_V():
    rospack = rospkg.RosPack()
    
    dir_path = "/home/vr-pc/Desktop/courses_2_3_traj_bags/course_2/vision/"

    plt.figure("Course 2 V")
    for i, file in enumerate(os.listdir(dir_path)):
        
        bag_path = dir_path + file
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
        
        plt.plot(x, y, '--', label="Husky Trajectory for Candidate {}".format(i + 1), linewidth=7)
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    # plt.title("Trajectory of Husky using Vision only of Degraded Vision Experiment")
    plt.grid()
    plt.axis('off')
    plt.show()


def plot_traj_course_2_VA():
    rospack = rospkg.RosPack()
    
    dir_path = "/home/vr-pc/Desktop/courses_2_3_traj_bags/course_2/vision_audio/"

    plt.figure("Course 2 VA")
    for i, file in enumerate(os.listdir(dir_path)):
        
        bag_path = dir_path + file
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
        
        plt.plot(x, y, '--', label="Husky Trajectory of Candidate {}".format(i), linewidth=7)
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    # plt.title("Trajectory of Husky using Vision & Audio for Degraded Vision Experiment")
    plt.axis('off')
    plt.grid()
    plt.show()


def plot_traj_course_3_V():
    
    dir_path = "/home/vr-pc/Desktop/courses_2_3_traj_bags/course_3/vision/"

    plt.figure("Course 3 V")
    for i, file in enumerate(os.listdir(dir_path)):
        
        bag_path = dir_path + file
        
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
        
        plt.plot(x, y, '--', label="Husky Trajectory of Candidate {}".format(i), linewidth=7)
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    # plt.title("Trajectory of Husky using Vision only in Workload Experiment")
    plt.axis('off')
    plt.grid()
    plt.show()


def plot_traj_course_3_VA():
    
    dir_path = "/home/vr-pc/Desktop/courses_2_3_traj_bags/course_3/vision_audio/"

    plt.figure("Course 3 VA")
    for i, file in enumerate(os.listdir(dir_path)):
       
        bag_path = dir_path + file
        
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
        
        plt.plot(x, y, '--', label="Husky Trajectory of Candidate {}".format(i), linewidth=7)
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    # plt.title("Trajectory of Husky using Vision & Audio in Workload Experiment")
    plt.grid()
    plt.axis('off')
    plt.show()

if __name__ == '__main__':
      rospy.init_node('traj_plotter')
      plot_traj_course_3_VA()
      plot_traj_course_3_V()
      plot_traj_course_2_VA()
      plot_traj_course_2_V()
      