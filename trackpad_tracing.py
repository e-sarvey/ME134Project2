import json
import matplotlib.pyplot as plt
import numpy as np

class TrackpadDraw:
    def __init__(self, xlim=(0, 10), ylim=(0, 10)):
        self.coordinates = []
        self.is_tracking = False
        self.fig, self.ax = plt.subplots(figsize=(10, 10))  # Create a blank figure for drawing
        self.ax.set_xlim(xlim)  # Set fixed x-axis limits
        self.ax.set_ylim(ylim)  # Set fixed y-axis limits

    def on_move(self, event):
        # Track the mouse movement only if is_tracking is True
        if event.xdata is not None and event.ydata is not None and self.is_tracking:
            self.coordinates.append([event.xdata, event.ydata])
            self.ax.plot(event.xdata, event.ydata, marker='o', color='b')
            self.fig.canvas.draw()  # Dynamically update the plot

    def on_click(self, event):
        # Start tracking when the left mouse button is pressed
        if event.button == 1:
            if event.name == 'button_press_event':
                self.is_tracking = True  # Start tracking
            elif event.name == 'button_release_event':
                self.is_tracking = False  # Stop tracking when the button is released

    def on_key_press(self, event):
        # Close the figure when the "Enter" key is pressed
        if event.key == 'enter':
            plt.close(self.fig)  # Close the figure and stop drawing
            
    def clickDraw(self, save=False, scale_factor=3, offset=(0, 10)):
        """
        Collects user-drawn trajectory, applies scaling and translation, and returns transformed trajectory.

        Parameters:
        save (bool): Whether to save the coordinates to a JSON file. Default is False.
        scale_factor (float): Scaling factor for the trajectory. Default is 3.
        offset (tuple): Tuple (x_offset, y_offset) for translating the trajectory. Default is (0, 10).
        """
        # Set up event handling within the figure
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_move)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Trackpad Drawing Window (Press Enter to Stop)')
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')

        # Show the drawing figure and block until closed
        plt.show(block=True)  # Ensure that the plot blocks until it is closed
        
        if save:
            # Save the coordinates to a file
            with open('raw_coordinates.json', 'w') as f:
                json.dump(self.coordinates, f)
                print("Raw coordinates have been saved to raw_coordinates.json")

        # If no data was collected, return None
        if len(self.coordinates) == 0:
            return None, None

        # Normalize the coordinates
        coordinates = np.array(self.coordinates)
        x_coords = coordinates[:, 0]
        y_coords = coordinates[:, 1]

        x_min, x_max = np.min(x_coords), np.max(x_coords)
        y_min, y_max = np.min(y_coords), np.max(y_coords)

        x_coords_normalized = (x_coords - x_min) / (x_max - x_min)
        y_coords_normalized = (y_coords - y_min) / (y_max - y_min)

        # Apply scaling and translation (without plotting)
        x_coords_scaled = x_coords_normalized * scale_factor + offset[0]
        y_coords_scaled = y_coords_normalized * scale_factor + offset[1]

        # Return the scaled and translated coordinates (without plotting them)
        return x_coords_scaled, y_coords_scaled

# Usage example:
drawer = TrackpadDraw(xlim=(0, 10), ylim=(0, 10))  # Set a fixed-size drawing area
x, y = drawer.clickDraw(save=False, scale_factor=5, offset=(0, 10))  # Get scaled values without plotting them
print(x, y)  # Print the scaled values
