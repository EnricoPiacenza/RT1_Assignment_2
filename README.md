### Define libraries, modules and global variables needed for the correct functioning of the program

1. Import libraries and modules.

2. Define the publisher objects:
    - `pub_PosVel`: publisher on the topic PosVel.
    - `pub_goal`: publisher on the topic goal_topic.

### Pseudocode of function main()

1. Function `main()`:
    - Initialize the node using `rospy.init_node('Actionlib_Client', anonymous=True)`.
    - Subscribe to the topic `/odom` using `rospy.Subscriber('/odom', Odometry, PosVel_Callback)`.
    - Create an Actionlib client using `actionlib.SimpleActionClient('/reaching_goal', PlanningAction)`.
    - If the action server doesn't start within 20 seconds:
        - Print an error message.
    - else:
        - Print a success message.
    - Start an infinite loop:
        - Ask the user for the goal coordinates using the function `ask_target_position()`.
        - Create the goal using the function `create_goal(x_goal, y_goal)`.
        - Inizialize the stop event:
            - `stop_event = threading.Event()`
        - Start a new thread to check if the user wants to cancel the goal:
            - `cancel_thread = threading.Thread(target=ask_user_to_cancel, args=(client))`
            - `cancel_thread.start()`

        - Start a new thread to wait for the goal to be reached:
            - `goal_thread = threading.Thread(target=goal_reached, args=(client, stop_event))`
            - `goal_thread.start()`

        - Wait for the `goal_thread` to finish using `goal_thread.join()`.
        - Set the stop event to end the `cancel_thread` using `stop_event.set()`.


### Pseudocode of the functions called in main():

1. Function `ask_target_position()`:
    - Start an infinite loop to acquire the x cordinate:
        - Ask the user to enter the x_goal position within the range (-8, 8):
        - If the entered value is within the valid range (-8 to 8):
            - Exit the loop.
        - else:
            - Display an error message: "Invalid x position. Try again."
    - Start an infinite loop to acquire the y cordinate:
        - Ask the user to enter the y_goal position within the range (-8, 8):
        - If the entered value is within the valid range (-8 to 8):
            - Exit the loop.
        - else:
            - Display an error message: "Invalid x position. Try again."
    - Return the validated x_goal and y_goal coordinates.

2. Function `create_goal(x_goal, y_goal)`:
    - Create a global variable `goal` initialized as an instance of `PlanningGoal()` to form the goal to send to the action server.
    - Set the target pose of the goal using `PoseStamped()` by assigning:
        - `goal.target_pose.pose.position.x` = `x_goal`.
        - `goal.target_pose.pose.position.y` = `y_goal`.
    - Publish the created goal message (`goal`) on the `/goal_topic` using the publisher `pub_goal`.

### Pseudocode of target functions of threads

1. Function `ask_user_to_cancel(client)`:
- Print "Do you want to cancel the goal? (yes/no): ".
- Start an infinite loop:
    - Use the `check_input()` function to retrive the user input every interval of 1 second.
    - Check the user's input:
    - If the user inputs is equal to `yes`:
      - Cancel the goal using `client.cancel_goal()`.
      - Log the information: "Goal cancelled by user." using `rospy.loginfo()`.
      - Exit the loop.

2. Function `check_input(interval)`:
    - Initialize `user_input` as `None`.
    - Record the `start_time` using `time.time()`.
    - Start a loop that runs until the time elapsed reaches the specified `interval` recived as input.
        - If any input is detected on `sys.stdin`:
            - Record the user's input and assign it to `user_input`.
            - Exit the loop.
    - Return the collected `user_input`.

3. Function `goal_reached(client, stop_event)`:
    - Log an info message: "Sending goal..." using `rospy.loginfo()`.
    - Send the goal to the action server using the provided `client` via `client.send_goal(goal)`.
    - Log an info message: "Waiting for result..." using `rospy.loginfo()`.
    - Wait for the goal to finish via `client.wait_for_result()`.
    - Retrieve the result using `client.get_result()` and store it in the variable `result`.
    - Log an info message containing the received result: "Result received: %s" using `rospy.loginfo(result)`.
    - Set the `stop_event` to indicate the goal thread has completed using `stop_event.set()`.


### Pseudocode of the Callback functions

1. Function `PosVel_Callback(data)`:
    - Create a custom message `position_velocity` using `PositionVelocity()` and the data retrived from the topic `\odom`:
    - Set the `x` attribute to the x-coordinate of the robot's position (`data.pose.pose.position.x`).
    - Set the `y` attribute to the y-coordinate of the robot's position (`data.pose.pose.position.y`).
    - Set the `vel_x` attribute to the linear velocity in the x-direction (`data.twist.twist.linear.x`).
    - Set the `vel_z` attribute to the angular velocity in the z-direction (`data.twist.twist.angular.z`).
    - Publish the created custom message `position_velocity` on the topic `/PosVel` using the publisher `pub_PosVel`.



