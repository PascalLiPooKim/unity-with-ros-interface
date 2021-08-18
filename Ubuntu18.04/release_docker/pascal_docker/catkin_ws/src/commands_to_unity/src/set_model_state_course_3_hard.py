#! /usr/bin/env python

"""
Script to make a model move at specific speed and specific path in Gazebo for course_3
(Workload Experiment).
"""


import rospy
from gazebo_msgs.msg import ModelState, ModelStates


# Function to publish the action that the models need to perform 
def pose_publisher_course_3_hard():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)

    # Initialize position of youbot
    pose_msg_youbot = ModelState()
    pose_msg_youbot.model_name = 'youbot'
    
    pose_msg_youbot.pose.position.x = 3.0
    pose_msg_youbot.pose.position.y = -7.55


    # Set flags to change direction
    going_left = False
    going_right = True

    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        # Path of youbot
        if pose_msg_youbot.pose.position.y < -3.9 and going_right:
            pose_msg_youbot.pose.position.y += 0.5/25
            if pose_msg_youbot.pose.position.y >= -4.0:
                going_right = False
                going_left = True
        elif pose_msg_youbot.pose.position.y > -7.6 and going_left:
            pose_msg_youbot.pose.position.y -= 0.5/25
            if pose_msg_youbot.pose.position.y <= -7.5:
                going_right = True
                going_left = False

        

        pub.publish(pose_msg_youbot)

        rate.sleep()
 
if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
          pose_publisher_course_3_hard()
      except rospy.ROSInterruptException:
          pass