#! /usr/bin/env python3

import rospy
from assignment_2_2023.srv import cord
from assignment_2_2023.msg import PlanningGoal


# Service callback function
def cord_Callback(_req):

    """
    callback function for the service Cord_serv,
    it returns the goal cordiantes x, y, z.
    """

    return x, y, z

def goal_callback(msg):

    """
    callback function for the topic /goal_topic,
    it saves the goal position in global variables x, y, z.
    """

    global x,y,z

    x = msg.target_pose.pose.position.x
    y = msg.target_pose.pose.position.y
    z = msg.target_pose.pose.position.z
    


def main():

    # Initialize the nodes
    rospy.init_node('Cordinate_service', anonymous=True)

    # Create a subscriber
    _sub = rospy.Subscriber('goal_topic', PlanningGoal, goal_callback)

    # Create the service to print the goal position
    _s = rospy.Service('Cord_serv', cord, cord_Callback)

    rospy.spin()

if __name__ == '__main__':
    main()


