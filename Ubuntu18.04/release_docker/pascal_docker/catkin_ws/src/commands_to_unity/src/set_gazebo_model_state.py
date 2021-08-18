#! /usr/bin/env python
"""
Script to make models move at specific speeds and specific path
(not being used)
"""

import rospy
from gazebo_msgs.msg import ModelState
 
def pose_publisher_course_2_easy():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'fire_hydrant'
    rate = rospy.Rate(20)
    pose_msg.pose.position.x = 5.2
    pose_msg.pose.position.y = -2.0
    going_left = False
    going_right = True
    while not rospy.is_shutdown():


        if pose_msg.pose.position.x < 8.9 and going_right:
            pose_msg.pose.position.x += 1./30
            if pose_msg.pose.position.x > 8.9:
                going_right = False
                going_left = True
        elif pose_msg.pose.position.x > 5.2 and going_left:
            pose_msg.pose.position.x -= 1./30
            if pose_msg.pose.position.x < 5.2:
                going_right = True
                going_left = False

        pub.publish(pose_msg)
        rate.sleep()
 
if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
          pose_publisher_course_2_easy()
      except rospy.ROSInterruptException:
          pass