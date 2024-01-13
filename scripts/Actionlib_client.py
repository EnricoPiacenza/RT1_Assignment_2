#! /usr/bin/env python3

import select
import sys
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PositionVelocity
from nav_msgs.msg import Odometry
import threading
import time


# Create the publisher objects
pub_PosVel = rospy.Publisher('PosVel', PositionVelocity, queue_size=10)
pub_goal = rospy.Publisher('goal_topic', PlanningGoal, queue_size=10)


def PosVel_Callback(data):

    """
    Callback function for the subscriber to the topic /odom.
    It creates a custom message with the position and velocity of the robot according to
    the custom message PositionVelocity.msg and publishes it on the topic /PosVel using 
    the publisher pub_PosVel.

    inputs: data (Odometry)
    """

    # Aquring Position and Velocity
    position_velocity = PositionVelocity()
    position_velocity.x = data.pose.pose.position.x
    position_velocity.y = data.pose.pose.position.y
    position_velocity.vel_x = data.twist.twist.linear.x
    position_velocity.vel_z = data.twist.twist.angular.z

    # Publish the custom message
    pub_PosVel.publish(position_velocity)




def input_with_timeout(timeout):

    """
    Function to get the user input with timeout.
    It returns the user input if it is given within the timeout, otherwise it returns None.

    inputs: timeout (float)
    output: user_input (string)
    """
    user_input = None
    start_time = time.time()
    while time.time() - start_time < timeout:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            user_input = input()
            break
    return user_input



def ask_user_to_cancel(client, stop_event):

    """
    Function to ask the user if they want to cancel the goal.
    inputs: client (SimpleActionClient)
    """
    print('Do you want to cancel the goal? (yes/no): ')
    while True:
        user_input = input_with_timeout(1)
        if user_input == 'yes':
            client.cancel_goal()
            rospy.loginfo('Goal cancelled by user.')
            break

def goal_reached(client, stop_event):

    """
    Function that waits for the goal to be reached.
    inputs: client (SimpleActionClient), stop_event (threading.Event)
    """

    # Send the goal to the action server
    rospy.loginfo('Sending goal...')
    client.send_goal(goal)

    # Wait for the goal to finish
    rospy.loginfo('Waiting for result...')
    client.wait_for_result()

    # Print the result
    result = client.get_result()
    rospy.loginfo('Result received: %s', result)

    # Set the stop event to end the cancel_thread
    stop_event.set()


def create_goal(x_goal, y_goal):
        
    """
    Function to create the goal to send to the action server, 
    it publishes the goal created on the topic /goal_topic.
    inputs: x_goal (float), y_goal (float)
    """
    
    global goal
    
    # Create the goal to send to the action server
    goal = PlanningGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    
    # Publish the goal
    pub_goal.publish(goal)

def ask_target_position():
        
    """
    Function to ask the user for the goal coordinates.
    output: x_goal (float), y_goal (float)
    """
    
    # Ask the user for the goal coordinates
    while True:
        x_goal = float(input('Enter the goal x position (-8;8): '))
        if -8 <= x_goal <= 8:
            break
        else:
            print('Invalid x position. Try again.')
    while True:
        y_goal = float(input('Enter the goal y position (-8;8): '))
        if -8 <= y_goal <= 8:
            break
        else:
            print('Invalid y position. Try again.')
        
    return x_goal, y_goal


def main():

    """
    Main function of the node.
    it initializes the node, it subscribes to the topic /odom, it creates the Actionlib client
    it starts a loop that asks the user for the goal coordinates, sends the goal to the action server.
    it inizialize 2 thread, one that asks checks if the user wants to cancel the goal
    the other that waits for the goal to be reached.
    """
    
    # Initialize the node
    rospy.init_node('Actionlib_Client', anonymous=True)

    # Subscribe to the topic /odom to publish the custom message about position and velocity
    rospy.Subscriber('/odom', Odometry, PosVel_Callback)

    # Create an Actionlib client
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)

    # Wait until the action server is up and running (max 20 seconds)
    rospy.loginfo('Waiting for action server...')
    if not client.wait_for_server(rospy.Duration(20)):
        rospy.loginfo('Error: Action server not available.')
    else:
        rospy.loginfo('Action server is up and running.')

    while True:
        
        x_goal, y_goal = ask_target_position()

        create_goal(x_goal, y_goal)

        # Create a stop event for the thread
        stop_event = threading.Event()

        # Start a new thread that asks the user if they want to cancel the goal
        cancel_thread = threading.Thread(target=ask_user_to_cancel, args=(client, stop_event))
        cancel_thread.start()

        # Start a new thread that waits for the goal to be reached
        goal_thread = threading.Thread(target=goal_reached, args=(client,stop_event))
        goal_thread.start()

        # Wait for the goal_thread to finish
        goal_thread.join()

        # Set the stop event to end the cancel_thread
        stop_event.set() 


if __name__ == '__main__':
    main()