#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
 
def pose_publisher_course_2_easy():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)

    pose_msg_fire_hydrant = ModelState()
    pose_msg_fire_hydrant.model_name = 'fire_hydrant' # Construction Barrel_1
    
    pose_msg_fire_hydrant.pose.position.x = 5.2
    pose_msg_fire_hydrant.pose.position.y = -2.0

    pose_msg_barrel_1 = ModelState()
    pose_msg_barrel_1.model_name = "Construction Barrel_1"
    
    pose_msg_barrel_1.pose.position.x = -1.0
    pose_msg_barrel_1.pose.position.y = 2.0


    going_left = False
    going_right = True

    going_up = True
    going_down = False
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        # for i in range(1000):
        #     pose_msg_fire_hydrant.pose.position.x += 0.05/30
        #     pub.publish(pose_msg_fire_hydrant)

        # for j in range(1000):
        #     pose_msg_fire_hydrant.pose.position.x -= 0.05/30
        #     pub.publish(pose_msg_fire_hydrant)

        if pose_msg_fire_hydrant.pose.position.x < 8.9 and going_right:
            pose_msg_fire_hydrant.pose.position.x += 0.5/25
            if pose_msg_fire_hydrant.pose.position.x > 8.9:
                going_right = False
                going_left = True
        elif pose_msg_fire_hydrant.pose.position.x > 5.2 and going_left:
            pose_msg_fire_hydrant.pose.position.x -= 0.5/25
            if pose_msg_fire_hydrant.pose.position.x < 5.2:
                going_right = True
                going_left = False

        if pose_msg_barrel_1.pose.position.y < 8.0 and going_up:
            pose_msg_barrel_1.pose.position.y += 0.8/30
            if pose_msg_barrel_1.pose.position.y > 7.9:
                going_up = False
                going_down = True
        elif pose_msg_barrel_1.pose.position.y > 2.3 and going_down:
            pose_msg_barrel_1.pose.position.y -= 0.8/30
            if pose_msg_barrel_1.pose.position.y < 2.4:
                going_up = True
                going_down = False

        pub.publish(pose_msg_fire_hydrant)
        pub.publish(pose_msg_barrel_1)
        rate.sleep()
 
if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
          pose_publisher_course_2_easy()
      except rospy.ROSInterruptException:
          pass