import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import paho.mqtt.client as mqtt
from mysecrets import tufts_robot, my_mqtt_broker

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

def invKin(x, y):
    # Get motor angles for a single point (x,y)
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

def armSim(formatted_thetas):
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
    trajectory_x = [0,-2, -2, 2, 2, -2,0]  # Example trajectories (2x2 square)
    trajectory_y = [15.86,12, 10, 10, 12, 12,15.86]

    if len(trajectory_x) != len(trajectory_y):
        print("Desired trajectories are incompatible sizes")
        return
    
    results = [invKin(x, y) for x, y in zip(trajectory_x, trajectory_y)]
    thetas = [(math.degrees(result[0]), math.degrees(result[1])) if result else None for result in results]
    valid_thetas = [theta for theta in thetas if theta is not None]
    formatted_thetas = ";".join([f"{theta1:.2f},{theta2:.2f}" for theta1, theta2 in valid_thetas])
    
    print("Formatted theta string:", formatted_thetas)
    print("-------------------------------------------")
    time.sleep(1)
    print("Peparing to send trajectory to ESP-32.......")
    time.sleep(2)
    print("SENT!")
    client.publish(topic, formatted_thetas)
    time.sleep(1)
    # armSim(formatted_thetas)

### RUNNING MAIN CODE ###
try:
    main()
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    print("Program Finished")
