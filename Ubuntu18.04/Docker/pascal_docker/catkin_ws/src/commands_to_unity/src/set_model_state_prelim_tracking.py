#!/usr/bin/env python
import rospy
import rospkg
import time
from gazebo_msgs.msg import ModelState
import numpy as np
import rosbag
# import tf
# import geometry_msgs
 
pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)

def pose_publisher_prelim_tracking(model_name, x, y, iter, r, s):
    
    # pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)

    pose_msg = ModelState()
    pose_msg.model_name = model_name # Construction Barrel_1
    
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pub.publish(pose_msg)

    time.sleep(2)

    # dest = np.array([[-6.0, 6.0], [6.0, 6.0], [6.0, -6.0], [-6.0, -6.0]])
    i = 0

    rate = rospy.Rate(r)

    rospack = rospkg.RosPack()
    path = rospack.get_path("commands_to_unity")

    bag = rosbag.Bag(path + "/bags/model_traj.bag", 'w')
    # while not rospy.is_shutdown():

        # for i in range(1000):
        #     pose_msg.pose.position.x += 0.5/30
        #     pub.publish(pose_msg)

        # for j in range(1000):
        #     pose_msg.pose.position.x += 1/30
        #     pub.publish(pose_msg)

        # if pose_msg.pose.position.x < (x - 1.0)*-1 and pose_msg.pose.position.y < (y + 1.0):
        #     pose_msg.pose.position.x += 3.0/30
        # elif pose_msg.pose.position.x > -1*x and pose_msg.pose.position.y > (-y - 1.0):
        #     pose_msg.pose.position.y -= 3.0/30
        # elif pose_msg.pose.position.x > 1*(x + 1.0) and pose_msg.pose.position.y > (-y - 1.0):
        #     pose_msg.pose.position.x -= 3.0/30

        # if pose_msg.pose.position.x < limits[1,0] and pose_msg.pose.position.y < limits[1, 1]:
        #     pose_msg.pose.position.x += 2.5/30
        # elif pose_msg.pose.position.x > limits[2,0] and pose_msg.pose.position.y > limits[2, 1]:
        #     pose_msg.pose.position.y -= 2.5/30
        # elif pose_msg.pose.position.x > limits[3,0] and pose_msg.pose.position.y > limits[3, 1]:
        #     pose_msg.pose.position.x -= 2.5/30
        # elif pose_msg.pose.position.x < limits[0,0] and pose_msg.pose.position.y > limits[0, 1]:
        #     pose_msg.pose.position.y += 2.5/30
        


        # print(pose_msg.pose.position.x, pose_msg.pose.position.y, i)
        # i+=1
        # pub.publish(pose_msg)
        # rate.sleep()
    try:

        for i in range(iter):
            pose_msg.pose.position.y -= s/r
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()


        for i in range(iter):
            pose_msg.pose.position.x -= s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()

        for i in range(iter):
            pose_msg.pose.position.y += s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()


        for i in range(iter):
            pose_msg.pose.position.x += s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()


        for i in range(iter):
            pose_msg.pose.position.y -= s/r
            pose_msg.pose.position.x -= s/r
            bag.write("model_traj", pose_msg.pose.position)
            print(pose_msg.pose.position.x, pose_msg.pose.position.y)
            pub.publish(pose_msg)
            rate.sleep()

    finally:
        bag.close()

    
def pose_publisher_prelim_static(model_name, x, y):
    # pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(30)
    pose_msg = ModelState()
    pose_msg.model_name = model_name # Construction Barrel_1
    
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pub.publish(pose_msg)
    rate.sleep()


def move_models_near_husky():

    # pose_publisher_prelim_static("Construction Barrel", 1.5, 0.0)
    # time.sleep(10)


    # pose_publisher_prelim_static("Construction Barrel", 20.0, 20.0)
    # pose_publisher_prelim_static("r2", 0.0, 1.5)
    # time.sleep(10)

    # pose_publisher_prelim_static("r2", 20.0, 18.0)
    # pose_publisher_prelim_static("Dumpster", 0.0, -2.0)
    # time.sleep(10)
    
    # pose_publisher_prelim_static("Dumpster", 20.0, 15.0)
    # time.sleep(5)


    # pose_publisher_prelim_static("Construction Barrel", 1.5, 0.0)
    # pose_publisher_prelim_static("r2", 0.0, 1.5)
    # time.sleep(10)

    # pose_publisher_prelim_static("Construction Barrel", 20.0, 20.0)
    # pose_publisher_prelim_static("Dumpster", 0.0, -2.0)
    # time.sleep(10)

    # pose_publisher_prelim_static("r2", 20.0, 18.0)
    # time.sleep(10)

    # pose_publisher_prelim_static("Dumpster", 20.0, 15.0)
    # youbot

    while True:

        input_key = input()

        if input_key == 1:
            pose_publisher_prelim_static("Construction Barrel", 1.5, 0.0)
            time.sleep(1)

        elif input_key == 2:
            # pose_publisher_prelim_static("Construction Barrel", 20.0, 20.0)
            pose_publisher_prelim_static("r2", 0.0, 1.5)
            time.sleep(1)

        elif input_key == 3:
            # pose_publisher_prelim_static("r2", 20.0, 18.0)
            pose_publisher_prelim_static("Dumpster", 0.0, -2.0)
            time.sleep(1)


        # elif input_key == 5:
        #     pose_publisher_prelim_static("Construction Barrel", 1.5, 0.0)
        #     pose_publisher_prelim_static("r2", 0.0, 1.5)
        #     time.sleep(3)

        # elif input_key == 6:
        #     pose_publisher_prelim_static("Construction Barrel", 20.0, 20.0)
        #     pose_publisher_prelim_static("Dumpster", 0.0, -2.0)
        #     time.sleep(3)


        elif input_key == 4:
            move_model_around_clockwise(-2.0, 2.0, 2.0, 50)
            

        elif input_key == 5:
            move_model_around_anticlockwise(-2.0, -2.0, 2.0, 50)


        elif input_key == 7:
            pose_publisher_prelim_static("Construction Barrel", 20.0, 20.0)
            time.sleep(1)


        elif input_key == 8:
            pose_publisher_prelim_static("r2", 20.0, 18.0)
            time.sleep(1)

        elif input_key == 9:
            pose_publisher_prelim_static("Dumpster", 20.0, 15.0)
            time.sleep(1)

        

        # elif input_key == 8:
        #     pose_publisher_prelim_static("Dumpster", 20.0, 15.0)


        elif input_key == 0:
            break

        else:
            print("Invalid Key pressed")


def move_model_around_clockwise(init_x, init_y, s, r):
    pose_msg = ModelState()
    pose_msg.model_name = "mars_rover" # Construction Barrel_1
    
    pose_msg.pose.position.x = init_x
    pose_msg.pose.position.y = init_y

    # quaternion = tf.transformations.quaternion_from_euler(3.14, 0.0, 0.0)
    

    # pose_msg.pose.orientation.x = quaternion[0]
    # pose_msg.pose.orientation.y = quaternion[1]
    # pose_msg.pose.orientation.z = quaternion[2]
    # pose_msg.pose.orientation.w = quaternion[3]
    pub.publish(pose_msg)

    rate = rospy.Rate(r)

    for i in range(100):
        pose_msg.pose.position.x += s/r
        # bag.write("model_traj", pose_msg.pose.position)
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()

    for i in range(100):
        pose_msg.pose.position.y -= s/r
        # bag.write("model_traj", pose_msg.pose.position)
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()


    for i in range(100):
        pose_msg.pose.position.x -= s/r
        # bag.write("model_traj", pose_msg.pose.position)
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()


def move_model_around_anticlockwise(init_x, init_y, s, r):
    pose_msg = ModelState()
    pose_msg.model_name = "mars_rover" # Construction Barrel_1
    
    pose_msg.pose.position.x = init_x
    pose_msg.pose.position.y = init_y

    # quaternion = tf.transformations.quaternion_from_euler(3.14, 0.0, 0.0)
    

    # pose_msg.pose.orientation.x = quaternion[0]
    # pose_msg.pose.orientation.y = quaternion[1]
    # pose_msg.pose.orientation.z = quaternion[2]
    # pose_msg.pose.orientation.w = quaternion[3]
    pub.publish(pose_msg)

    rate = rospy.Rate(r)

    for i in range(100):
        pose_msg.pose.position.x += s/r
        # bag.write("model_traj", pose_msg.pose.position)
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()

    for i in range(100):
        pose_msg.pose.position.y += s/r
        # bag.write("model_traj", pose_msg.pose.position)
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()


    for i in range(100):
        pose_msg.pose.position.x -= s/r
        # bag.write("model_traj", pose_msg.pose.position)
        print(pose_msg.pose.position.x, pose_msg.pose.position.y)
        pub.publish(pose_msg)
        rate.sleep()



    


if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
        # pose_publisher_prelim_tracking("mars_rover", 5.0, 5.0, 200, 30, 1.0)
        time.sleep(1)
        # pose_publisher_prelim_static("Construction Barrel", 1.5, 0.0)
        # pose_publisher_prelim_static("r2", 0.0, 1.5)
        # pose_publisher_prelim_static("Dumpster", 0.0, -2.0)
          #pose_publisher_prelim_tracking("mars_rover", 1.0, 1.0)


        move_models_near_husky()
        # move_model_around(-3.0, 3.0, 2.0, 50)
      except rospy.ROSInterruptException:
          pass
