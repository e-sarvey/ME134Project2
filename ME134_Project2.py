import math
import numpy as np
import paho.mqtt.client as mqtt

# Arm lengths (we chose all the same length links)
l0 = 4.25  # Half the base length between motors
l1 = 8.5  # Length of upper arm
l2 = 8.5  # Length of forearm

# intialize mqtt params and connect to broker
topic = "robot"
client_name = "Group_Two"
client = mqtt.Client(client_name)
client.connect("")
print(f'Connected to MQTT broker as {client}')


def invKin(x, y):
    # get motor angles for a single point (x,y)
    try:
        # Calculate distance from origin to end effector
        distance = math.sqrt(x**2 + y**2)

        # Pre-calculation for distance from motors
        dist_left = math.sqrt((l0 + x)**2 + y**2)
        dist_right = math.sqrt((l0 - x)**2 + y**2)

        # Handle division by zero for invalid distances
        if dist_left == 0 or dist_right == 0:
            return None  # Return None to mark as invalid if distances are zero

        # Angle from left motor to end effector
        beta1 = math.atan2(y, (l0 + x))

        # Angle from right motor to end effector
        beta2 = math.atan2(y, (l0 - x))

        # Calculate alpha angles using trigonometric identities
        alpha1 = math.atan2(
            math.sqrt(l1**2 - (l2**2 - dist_left**2 + l1**2) / (2 * l1 * dist_left)**2),
            (l2**2 - dist_left**2 + l1**2) / (2 * l1 * dist_left)
        )
        alpha2 = math.atan2(
            math.sqrt(l1**2 - (l2**2 - dist_right**2 + l1**2) / (2 * l1 * dist_right)**2),
            (l2**2 - dist_right**2 + l1**2) / (2 * l1 * dist_right)
        )

        # Compute final shoulder angles
        shoulder1 = beta1 + alpha1
        shoulder2 = math.pi - beta2 - alpha2

        # Ensure angles are within valid range
        if shoulder1 < 0 or shoulder1 > math.pi or shoulder2 < 0 or shoulder2 > math.pi:
            return None  # Invalid angle regime detected

        # Calculate forward kinematics to get passive joint positions
        p1_x = -l0 + l1 * math.cos(shoulder1)
        p1_y = l1 * math.sin(shoulder1)
        p2_x = l0 + l1 * math.cos(shoulder2)
        p2_y = l1 * math.sin(shoulder2)

        # Validate end effector position
        if y < min(p1_y, p2_y):
            return None  # Mark as invalid if end effector y-coordinate is below the farther passive joint y-coordinate

        return (shoulder1, shoulder2, beta1, beta2, alpha1, alpha2)

    except Exception as e:
        print(f"Error occurred at (x, y) = ({x}, {y}): {e}")
        return None  # Return None if any error occurs during calculations

def moveTime():
    # function to estimate time needed for steppers to move. Will need to see if needed.
    # Should iterate through points in thetas in main to find distance based on motor operating speed.

def main():
    trajectory_x = [1, 2, 3] # example trajectories
    trajectory_y = [1, 2, 3]

    if len(trajectory_x) != len(trajectory_y):
        print("Desired trajectories are incompatible sizes")
        return
    
    # use inverse kinematics to get motor angles
    thetas = [invKin(x,y) for x, y in zip(trajectory_x,trajectory_y)]
    
    # Calculate corresponding movement times
    times = moveTime()
    
    # Combine each theta pair with corresponding time
    if len(thetas) == len(times):
        # Creating a list of triplets (theta1, theta2, time)
        thetas_with_time = [(theta[0], theta[1], time) for theta, time in zip(thetas, times) if theta is not None]
        
        # Publish the result
        client.publish(topic, str(thetas_with_time))
        # print(f"Published thetas with time: {thetas_with_time}")
    else:
        print("Number of time values does not match number of theta pairs")
        return


### RUNNING MAIN CODE ###
try:
    main()
except Exception as e:
    print(f"an error occured: {e}")
finally:
    print("Program Finished")