import re
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import argparse

class Parser:
    # Function to parse a line of the log file
    def __parse_line(self, line):
        # Regular expression to match the Time part
        time_pattern = r'builtin_interfaces\.msg\.Time\(sec=(\d+), nanosec=(\d+)\)'
        
        # Split the line on tabs
        parts = line.split('\t')
        
        # Extract the Time part using regex
        time_match = re.match(time_pattern, parts[0])
        if not time_match:
            raise ValueError(f"Invalid time format: {parts[0]}")
        
        sec = int(time_match.group(1))
        nanosec = int(time_match.group(2))
        
        # Convert the remaining parts to floats
        values = list(map(float, parts[1:]))
        x, y, orientation, linear_velocity, angular_velocity = values
        
        if linear_velocity == 0.0 and angular_velocity == 0.0:
            return None

        # Create a timestamp from sec and nanosec
        timestamp = datetime.fromtimestamp(sec + nanosec / 1e9)
        
        return {
            'timestamp': timestamp,
            'x': x,
            'y': y,
            'orientation': orientation,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity
        }

    # Function to parse the log file
    def parse_log_file(self,file_path):
        parsed_data = []
        with open(file_path, 'r') as file:
            for line in file:
                parsed_line = self.__parse_line(line)
                if parsed_line:
                    parsed_data.append(parsed_line)
        return parsed_data

class Plotter:
    def __init__(self, parsed_data, selected_percentages=None):
        self.parsed_data = parsed_data
        if selected_percentages: 
            self.selected_indices = self.__percentages_to_indices(selected_percentages, len(parsed_data))
        else:
            self.selected_indices = None

    def plot(self, args):
        if args.path:
            self.__plot_xy_path()
        if args.pose:
            self.__plot_pose(args.step)
        if args.linear:
            self.__plot_linear_velocity()   
        if args.angular:
            self.__plot_angular_velocity()
        if args.x:
            self.__plot_x_coordinate()
        if args.y:
            self.__plot_y_coordinate()
        if args.orientation:
            self.__plot_orientation()
        if args.analysis:
            self.__analyze_data()

    def __percentages_to_indices(self, percentages, data_length):
        indices = [int(p * data_length) for p in percentages]
        return indices

    def __plot_xy_path(self):
        x_coords = [data['x'] for data in self.parsed_data]
        y_coords = [data['y'] for data in self.parsed_data]
        #breakpoint()
        plt.figure(figsize=(10, 10))
        if self.selected_indices:
            plt.scatter([x_coords[i] for i in self.selected_indices], [y_coords[i] for i in self.selected_indices], c='red', label='Selected Points', zorder = 2)
        plt.plot(x_coords, y_coords, marker='o', zorder=1)
        plt.title('XY Path')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        plt.show()

    # Function to plot the xy path with orientation arrows
    def __plot_pose(self, step):
        x_coords = [data['x'] for data in self.parsed_data]
        y_coords = [data['y'] for data in self.parsed_data]
        orientations = [data['orientation'] for data in self.parsed_data]
        
        plt.figure(figsize=(10, 10))
        # plt.scatter(x_coords, y_coords, c='blue', label='Position')
        plt.scatter(x_coords[0], y_coords[0], c='blue', label='Start')
        plt.scatter(x_coords[-1], y_coords[-1], c='red', label='End')
        
        n_points = len(x_coords)
        cmap = plt.colormaps['coolwarm']

        if self.selected_indices:
            plt.scatter([x_coords[i] for i in self.selected_indices], [y_coords[i] for i in self.selected_indices], c='red', marker='x', label='Selected Points', zorder=2)

        for i in range(0, n_points, step):
            x = x_coords[i]
            y = y_coords[i]
            orientation = orientations[i]
            dx = np.cos(orientation)
            dy = np.sin(orientation)

            color = cmap(i / n_points)  # Get the color from the colormap
            plt.arrow(x, y, dx * 0.1, dy * 0.1, head_width=0.05, head_length=0.1, fc=color, ec=color, zorder=1)
        

        plt.title('Pose Path')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        plt.legend()
        plt.show()


    # Function to plot linear velocities over time
    def __plot_linear_velocity(self):
        timestamps = [data['timestamp'] for data in self.parsed_data]
        linear_velocities = [data['linear_velocity'] for data in self.parsed_data]
        

        plt.figure(figsize=(10, 6))
        if self.selected_indices:
            plt.scatter([timestamps[i] for i in self.selected_indices], [linear_velocities[i] for i in self.selected_indices], c='red', label='Selected Points', zorder=2)
        plt.plot(timestamps, linear_velocities, color='tab:blue', label='Linear Velocity', zorder=1)
        plt.xlabel('Time')
        plt.ylabel('Linear Velocity')
        plt.title('Linear Velocity Over Time')
        plt.grid(True)
        plt.legend()
        plt.show()

    # Function to plot angular velocities over time
    def __plot_angular_velocity(self):
        timestamps = [data['timestamp'] for data in self.parsed_data]
        angular_velocities = [data['angular_velocity'] for data in self.parsed_data]
        
        plt.figure(figsize=(10, 6))
        if self.selected_indices:
            plt.scatter([timestamps[i] for i in self.selected_indices], [angular_velocities[i] for i in self.selected_indices], c='red', label='Selected Points', zorder=2)
        plt.plot(timestamps, angular_velocities, color='tab:red', label='Angular Velocity', zorder=1)
        plt.xlabel('Time')
        plt.ylabel('Angular Velocity')
        plt.title('Angular Velocity Over Time')
        plt.grid(True)
        plt.legend()
        plt.show()

    # Function to plot angular velocities over time
    def __plot_x_coordinate(self):
        timestamps = [data['timestamp'] for data in self.parsed_data]
        x_coords = [data['x'] for data in self.parsed_data]
        
        plt.figure(figsize=(10, 6))
        if self.selected_indices:
            plt.scatter([timestamps[i] for i in self.selected_indices], [x_coords[i] for i in self.selected_indices], c='red', label='Selected Points', zorder=2)
        plt.plot(timestamps, x_coords, color='tab:red', label='X Value', zorder=1)
        plt.xlabel('Time')
        plt.ylabel('X position')
        plt.title('X Value Over Time')
        plt.grid(True)
        plt.legend()
        plt.show()

    # Function to plot angular velocities over time
    def __plot_y_coordinate(self):
        timestamps = [data['timestamp'] for data in self.parsed_data]
        y_coords = [data['y'] for data in self.parsed_data]
        
        plt.figure(figsize=(10, 6))
        if self.selected_indices:
            plt.scatter([timestamps[i] for i in self.selected_indices], [y_coords[i] for i in self.selected_indices], c='red', label='Selected Points', zorder=2)
        plt.plot(timestamps, y_coords, color='tab:blue', label='Y Value', zorder=1)
        plt.xlabel('Time')
        plt.ylabel('Y position')
        plt.title('Y Value Over Time')
        plt.grid(True)
        plt.legend()
        plt.show()

    def __plot_orientation(self):
        timestamps = [data['timestamp'] for data in self.parsed_data]
        orientations = [data['orientation'] for data in self.parsed_data]
        #breakpoint()
        plt.figure(figsize=(10, 6))
        if self.selected_indices:
            plt.scatter([timestamps[i] for i in self.selected_indices], [orientations[i] for i in self.selected_indices], c='red', label='Selected Points', zorder=2)
        plt.plot(timestamps, orientations, color='tab:green', label='θ Value', zorder=1)
        plt.xlabel('Time')
        plt.ylabel('θ')
        plt.title('θ Value Over Time')
        plt.grid(True)
        plt.legend()
        plt.show()

    def __analyze_data(self):
        x_coords = [data['x'] for data in self.parsed_data]
        y_coords = [data['y'] for data in self.parsed_data]
        orientations = [data['orientation'] for data in self.parsed_data]

        min_x = round(min(x_coords), 5)
        max_x = round(max(x_coords), 5)
        min_y = round(min(y_coords), 5)
        max_y = round(max(y_coords), 5)
        min_orientation = min(orientations)
        max_orientation = max(orientations)
        print(f"x: min={min_x}, max={max_x}")
        print(f"y: min={min_y}, max={max_y}")
        print(f"θ: min={min_orientation}, max={max_orientation}")

def parse_args():
    parser = argparse.ArgumentParser(
        prog='DumpParser',
        description='This script parse a dump of odom'
    )

    parser.add_argument('filepath')
    parser.add_argument('--path', action='store_true')
    parser.add_argument('--pose', action='store_true')
    parser.add_argument('--linear', action='store_true')
    parser.add_argument('--angular', action='store_true')
    parser.add_argument('-x', action='store_true')
    parser.add_argument('-y', action='store_true')
    parser.add_argument('--orientation', action='store_true')
    parser.add_argument('-s', '--step', default=20, type=int)
    parser.add_argument('-p', '--points', type=float, nargs='*')
    parser.add_argument('-a', '--analysis', action='store_true')
    return parser.parse_args()

def main():
    args = parse_args()
    parsed_data = Parser().parse_log_file(args.filepath)
    Plotter(parsed_data, args.points).plot(args)
    return 0

if __name__ == "__main__":
    main()