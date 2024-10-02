import math
import time
import numpy as np
import matplotlib.pyplot as plt # For the simulation option
import matplotlib.animation as animation
import paho.mqtt.client as mqtt # mqtt service library
from mysecrets import tufts_robot, my_mqtt_broker # wifi and mqtt credentials from secrets file in git ignored
from trackpad_tracing import TrackpadDraw
# Arm lengths
l0 = 4.25  # Half the base length between motors
l1 = 8.5   # Length of upper arm
l2 = 8.5   # Length of forearm

# Initialize MQTT params and connect to broker
topic = "ME134/motors"
client_name = "Group_Two"
client = mqtt.Client(client_name)
client.connect(my_mqtt_broker["ip"])
print(f'Connected to MQTT broker as {client}')

drawer = TrackpadDraw()

def invKin(x, y):
# This function returns the desired motor angles for a single input point (x,y)
# Based on code from an existing library, but modified to better handle errors, out of bound positions, and singularity points. Also replaced arccos() use with equivilant atan2() for increased robustness.
    try:
        distance = math.sqrt(x**2 + y**2)
        dist_left = math.sqrt((l0 + x)**2 + y**2)
        dist_right = math.sqrt((l0 - x)**2 + y**2)

        if dist_left == 0 or dist_right == 0:
            return None

        beta1 = math.atan2(y, (l0 + x))
        beta2 = math.atan2(y, (l0 - x))

        alpha1 = math.acos((l1**2 + dist_left**2 - l2**2) / (2 * l1 * dist_left))
        alpha2 = math.acos((l1**2 + dist_right**2 - l2**2) / (2 * l1 * dist_right))

        shoulder1 = beta1 + alpha1
        shoulder2 = math.pi - beta2 - alpha2

        if shoulder1 < 0 or shoulder1 > math.pi or shoulder2 < 0 or shoulder2 > math.pi:
            return None

        return (shoulder1, shoulder2)

    except Exception as e:
        print(f"Error occurred at (x, y) = ({x}, {y}): {e}")
        return None

def workspaceMap(xlim=(-10, 10), ylim=(0, 20), resolution=100, x_traj=None, y_traj=None):
    # Create a grid of points within the given limits
    x_range = np.linspace(xlim[0], xlim[1], resolution)
    y_range = np.linspace(ylim[0], ylim[1], resolution)
    
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Workspace Map')

    # Plot workspace points with validity check
    for x in x_range:
        for y in y_range:
            angles = invKin(x, y)
            if angles is not None:
                ax.plot(x, y, marker='o', color='lightgreen', markersize=2)  # Valid angles
            else:
                ax.plot(x, y, marker='o', color='lightcoral', markersize=2)  # Invalid (out of reach)
    
    # Plot the trajectory if provided
    if x_traj is not None and y_traj is not None:
        # Ensure x_traj and y_traj have the same length
        if len(x_traj) != len(y_traj):
            raise ValueError("x_traj and y_traj must have the same length")
        
        # Plot the trajectory as a separate line
        ax.plot(x_traj, y_traj, marker='o', color='blue', linestyle='-', label='Trajectory')
        ax.legend()

    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def armSim(formatted_thetas):
# This function simulates the position of the arm and input path
    theta_pairs = formatted_thetas.split(";")
    
    fig, ax = plt.subplots()
    ax.set_xlim(-30, 30)
    ax.set_ylim(0, 25)
    ax.set_aspect('equal', adjustable='box')

    motor_left_x = -l0
    motor_right_x = l0
    motor_y = 0

    # List to store end effector positions for path tracing
    ee_positions = []

    def update(i):
        ax.cla()
        ax.set_xlim(-30, 30)
        ax.set_ylim(0, 25)
        ax.set_aspect('equal', adjustable='box')

        if i < len(theta_pairs):
            theta1_deg, theta2_deg = map(float, theta_pairs[i].split(","))
            theta1 = math.radians(theta1_deg)
            theta2 = math.radians(theta2_deg)

            p1_x = motor_left_x + l1 * math.cos(theta1)
            p1_y = motor_y + l1 * math.sin(theta1)
            p2_x = motor_right_x + l1 * math.cos(theta2)
            p2_y = motor_y + l1 * math.sin(theta2)

            # Calculate end effector position
            d = math.sqrt((p2_x - p1_x)**2 + (p2_y - p1_y)**2)
            if d <= 2 * l2:
                a = d / 2
                h = math.sqrt(l2**2 - a**2)

                midpoint_x = (p1_x + p2_x) / 2
                midpoint_y = (p1_y + p2_y) / 2

                delta_x = p2_x - p1_x
                delta_y = p2_y - p1_y
                perp_x = -delta_y
                perp_y = delta_x
                mag = math.sqrt(perp_x**2 + perp_y**2)
                perp_x /= mag
                perp_y /= mag

                ee_x = midpoint_x + h * perp_x
                ee_y = midpoint_y + h * perp_y
                
                # Append the end effector position to the list
                ee_positions.append((ee_x, ee_y))

                # Plot the upper arms
                ax.plot([motor_left_x, p1_x], [motor_y, p1_y], 'b-o', label=None)
                ax.plot([motor_right_x, p2_x], [motor_y, p2_y], 'r-o', label=None)

                # Plot the forearms and end effector
                ax.plot([p1_x, ee_x], [p1_y, ee_y], 'g-o', label=None)
                ax.plot([p2_x, ee_x], [p2_y, ee_y], 'm-o', label=None)
                ax.plot(ee_x, ee_y, 'ko', label=None)
                
                # Plot the path of the end effector
                if len(ee_positions) > 1:
                    path_x, path_y = zip(*ee_positions)
                    ax.plot(path_x, path_y, 'k--', label='End Effector Path')

            else:
                print(f"Invalid configuration at step {i}: forearms cannot connect.")
        
        ax.set_title("5-Bar Arm Simulation")
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")

    ani = animation.FuncAnimation(fig, update, frames=len(theta_pairs), interval=1000, repeat=True)
    plt.show()

def main():
    # Input trajectory. This section can be updated to use a different trajectory planning function in the future.
    #trajectory_x = [0,-2, -2, 2, 2, -2, 0]  # Example trajectories (2x2 square)
    #trajectory_y = [15.86,12, 10, 10, 12, 12,15.86] 
    #trajectory_x = [0, 0, 0 , 0, 0, 0, 0]  # Example trajectories (2x2 square)
    #trajectory_y = [15.86, 14, 13, 12, 11, 10, 15.86]
    trajectory_x, trajectory_y = drawer.clickDraw(save=False,scale_factor=5, offset=(-5/2, 9)) # you have to draw twice for some reason
    time.sleep(0.5)
    workspaceMap(x_traj=trajectory_x,y_traj=trajectory_y)
    # trajectory vectors need to be the same length of course
    if len(trajectory_x) != len(trajectory_y):
        print("Desired trajectories are incompatible sizes")
        return
    
    results = [invKin(x, y) for x, y in zip(trajectory_x, trajectory_y)] # perform ik on each corresponding pair of coordinates to get pair of angles
    thetas = [(math.degrees(result[0]), math.degrees(result[1])) if result else None for result in results] # convert to degrees
    valid_thetas = [theta for theta in thetas if theta is not None] # make sure None doesnt get passed to the ESP
    formatted_thetas = ";".join([f"{theta1:.2f},{theta2:.2f}" for theta1, theta2 in valid_thetas]) # reformat into structure that matches parsing on ESP to be sent over MQTT
    
    print("Formatted theta string:", formatted_thetas)
    print("-------------------------------------------")
    time.sleep(1)
    print("Peparing to send trajectory to ESP-32.......")
    time.sleep(0.5)
    print("SENT!")
    client.publish(topic, formatted_thetas) # SEND OVER MQTT TO RUN ON ESP32 
    time.sleep(1)
    # armSim(formatted_thetas)

### RUNNING MAIN CODE ###
try:
    # everything runs in the function main() but put in a try-except block to clean up error handling :)
    main()
# except Exception as e:
#     print(f"An error occurred: {e}")
finally:
    print("Program Finished")
