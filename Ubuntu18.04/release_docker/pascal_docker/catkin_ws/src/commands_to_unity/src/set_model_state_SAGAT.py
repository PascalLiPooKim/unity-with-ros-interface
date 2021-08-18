#! /usr/bin/env python
"""
Script for making models to move  in specific direction and at constant speed in SAGAT course
(Situational Awareness Main Experiment)
"""
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
 
def pose_publisher_course_3_hard():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=20)

    # Initialize the position of the robots and persons
    pose_msg_pioneer3at = ModelState()
    pose_msg_pioneer3at.model_name = 'pioneer3at'
    
    pose_msg_pioneer3at.pose.position.x = -9.5
    pose_msg_pioneer3at.pose.position.y = -9.0

    pub.publish(pose_msg_pioneer3at)


    pose_msg_pioneer2dx = ModelState()
    pose_msg_pioneer2dx.model_name = "pioneer2dx"

    pose_msg_pioneer2dx.pose.position.x = -4.0
    pose_msg_pioneer2dx.pose.position.y = -10.2

    pub.publish(pose_msg_pioneer2dx)


    pose_msg_pioneer2dx_0 = ModelState()
    pose_msg_pioneer2dx_0.model_name = "pioneer2dx_0"

    pose_msg_pioneer2dx_0.pose.position.x = -4.0
    pose_msg_pioneer2dx_0.pose.position.y = -7.0

    pub.publish(pose_msg_pioneer2dx_0)


    pose_msg_person_walking = ModelState()
    pose_msg_person_walking.model_name = "person_walking"

    pose_msg_person_walking.pose.position.x = 9.0
    pose_msg_person_walking.pose.position.y = -8.5

    pub.publish(pose_msg_person_walking)


    pose_msg_person_walking_0 = ModelState()
    pose_msg_person_walking_0.model_name = "person_walking_0"

    pose_msg_person_walking_0.pose.position.x = 4.5
    pose_msg_person_walking_0.pose.position.y = 3.0

    pub.publish(pose_msg_person_walking_0)

    pose_msg_person_walking_1 = ModelState()
    pose_msg_person_walking_1.model_name = "person_walking_1"

    pose_msg_person_walking_1.pose.position.x = -4.5
    pose_msg_person_walking_1.pose.position.y = 6.8

    pub.publish(pose_msg_person_walking_1)


    pose_msg_pioneer2dx_1 = ModelState()
    pose_msg_pioneer2dx_1.model_name = "pioneer2dx_1"

    pose_msg_pioneer2dx_1.pose.position.x = 3.0
    pose_msg_pioneer2dx_1.pose.position.y = 7.0

    pub.publish(pose_msg_pioneer2dx_1)

    rospy.sleep(1.0)

    # Set flags to change direction of models
    pioneer3at_up = True
    pioneer3at_down = False

    pioneer2dx_left = True
    pioneer2dx_right = False

    pioneer2dx_0_left = True
    pioneer2dx_0_right = False

    person_walking_left = True
    person_walking_right = False

    person_walking_1_up = True
    person_walking_1_down = False

    pioneer2dx_1_left = True
    pioneer2dx_1_right = False

    person_walking_0_up = True
    person_walking_0_down = False

    r = 50
    rate = rospy.Rate(r)

    # Publish trajectory of models for them to make in Gazebo
    while not rospy.is_shutdown():

        if pose_msg_pioneer3at.pose.position.y < -6.0 and pioneer3at_up:
            pose_msg_pioneer3at.pose.position.y += 0.5/r
            if pose_msg_pioneer3at.pose.position.y >= -6.5:
                pioneer3at_up = False
                pioneer3at_down = True
        elif pose_msg_pioneer3at.pose.position.y > -9.1 and pioneer3at_down:
            pose_msg_pioneer3at.pose.position.y -= 0.5/r
            if pose_msg_pioneer3at.pose.position.y <= -9.0:
                pioneer3at_up = True
                pioneer3at_down = False

        
        pub.publish(pose_msg_pioneer3at)



        if pose_msg_pioneer2dx.pose.position.x < -3.9 and pioneer2dx_right:
            pose_msg_pioneer2dx.pose.position.x += 0.5/r
            if pose_msg_pioneer2dx.pose.position.x >= -4.0:
                pioneer2dx_right = False
                pioneer2dx_left = True
        elif pose_msg_pioneer2dx.pose.position.x > -8.1 and pioneer2dx_left:
            pose_msg_pioneer2dx.pose.position.x -= 0.5/r
            if pose_msg_pioneer2dx.pose.position.x <= -8.0:
                pioneer2dx_right = True
                pioneer2dx_left = False

        
        pub.publish(pose_msg_pioneer2dx)



        if pose_msg_pioneer2dx_0.pose.position.x < -3.9 and pioneer2dx_0_right:
            pose_msg_pioneer2dx_0.pose.position.x += 0.5/r
            if pose_msg_pioneer2dx_0.pose.position.x >= -4.0:
                pioneer2dx_0_right = False
                pioneer2dx_0_left = True
        elif pose_msg_pioneer2dx_0.pose.position.x > -8.1 and pioneer2dx_0_left:
            pose_msg_pioneer2dx_0.pose.position.x -= 0.5/r
            if pose_msg_pioneer2dx_0.pose.position.x <= -8.0:
                pioneer2dx_0_right = True
                pioneer2dx_0_left = False

        
        pub.publish(pose_msg_pioneer2dx_0)



        if pose_msg_person_walking.pose.position.x > 3.0 and person_walking_right:
            pose_msg_person_walking.pose.position.x -= 0.5/r
            if pose_msg_person_walking.pose.position.x <= 3.2:
                person_walking_right = False
                person_walking_left = True
        elif pose_msg_person_walking.pose.position.x < 9.1 and person_walking_left:
            pose_msg_person_walking.pose.position.x += 0.5/r
            if pose_msg_person_walking.pose.position.x >= 8.9:
                person_walking_right = True
                person_walking_left = False

        
        pub.publish(pose_msg_person_walking)


        if pose_msg_person_walking_1.pose.position.y < 10.0 and person_walking_1_up:
            pose_msg_person_walking_1.pose.position.y += 0.5/r
            if pose_msg_person_walking_1.pose.position.y >= 9.8:
                person_walking_1_up = False
                person_walking_1_down = True
        elif pose_msg_person_walking_1.pose.position.y > 2.0 and person_walking_1_down:
            pose_msg_person_walking_1.pose.position.y -= 0.5/r
            if pose_msg_person_walking_1.pose.position.y <= 2.8:
                person_walking_1_up = True
                person_walking_1_down = False

        
        pub.publish(pose_msg_person_walking_1)


        if pose_msg_pioneer2dx_1.pose.position.x > 1.0 and pioneer2dx_1_left:
            pose_msg_pioneer2dx_1.pose.position.x -= 0.5/r
            if pose_msg_pioneer2dx_1.pose.position.x <= 1.2:
                pioneer2dx_1_right = True
                pioneer2dx_1_left = False
        elif pose_msg_pioneer2dx_1.pose.position.x < 5.0 and pioneer2dx_1_right:
            pose_msg_pioneer2dx_1.pose.position.x += 0.5/r
            if pose_msg_pioneer2dx_1.pose.position.x >= 4.8:
                pioneer2dx_1_right = False
                pioneer2dx_1_left = True

        
        pub.publish(pose_msg_pioneer2dx_1)


        if pose_msg_person_walking_0.pose.position.y < 5.0 and person_walking_0_up:
            pose_msg_person_walking_0.pose.position.y += 0.5/r
            if pose_msg_person_walking_0.pose.position.y >= 4.8:
                person_walking_0_up = False
                person_walking_0_down = True
        elif pose_msg_person_walking_0.pose.position.y > 2.0 and person_walking_0_down:
            pose_msg_person_walking_0.pose.position.y -= 0.5/r
            if pose_msg_person_walking_0.pose.position.y <= 2.8:
                person_walking_0_up = True
                person_walking_0_down = False

        
        pub.publish(pose_msg_person_walking_0)

        rate.sleep()
 
if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
          pose_publisher_course_3_hard()
      except rospy.ROSInterruptException:
          pass