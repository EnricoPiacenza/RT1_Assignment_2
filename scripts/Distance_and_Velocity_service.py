#! /usr/bin/env python3

import queue
import rospy
from assignment_2_2023.msg import PositionVelocity, PlanningGoal
from assignment_2_2023.srv import info
from nav_msgs.msg import Odometry



q_vx = queue.Queue(10)
q_vz = queue.Queue(10)

def goal_callback(msg):

    global x_g,y_g

    x_g = msg.target_pose.pose.position.x
    y_g = msg.target_pose.pose.position.y

def PosVel_Callback(data):

    global position_velocity
    global q_vx
    global q_vz

    # Aquring Position and Velocity
    position_velocity = PositionVelocity()
    position_velocity.x = data.pose.pose.position.x
    position_velocity.y = data.pose.pose.position.y
    position_velocity.vel_x = data.twist.twist.linear.x
    position_velocity.vel_z = data.twist.twist.angular.z

    # Filling the queues
    if q_vx.full():
        q_vx.get()
        q_vx.put(position_velocity.vel_x)
    else:
        q_vx.put(position_velocity.vel_x)

    if q_vz.full():
        q_vz.get()
        q_vz.put(position_velocity.vel_z)
    else:
        q_vz.put(position_velocity.vel_z)

def dist_Callback(req):

    global position_velocity
    global q_vx
    global q_vz

    # Computing the distance to the goal
    x = x_g - position_velocity.x
    y = y_g - position_velocity.y

    # Computing the avarage velocity
    v_x = sum(list(q_vx.queue)) / q_vx.qsize()
    v_z = sum(list(q_vz.queue)) / q_vz.qsize()

    return x, y, v_x, v_z


def main():
    
    # Initialize the nodes
    rospy.init_node('Distance_and_Velocity_service', anonymous=True)

    # Subscribe to the topic /odom
    rospy.Subscriber('/odom', Odometry, PosVel_Callback)

    # Subscribe to the topic /goal_topic
    rospy.Subscriber('goal_topic', PlanningGoal, goal_callback)

    # Create a service server Dist_target
    s = rospy.Service('Dist_target', info, dist_Callback)    

    rospy.spin()
    

if __name__ == '__main__':
    main()