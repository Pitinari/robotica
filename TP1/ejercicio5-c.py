import numpy as np
import csv
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import transforms3d.affines as ta
import transforms3d.quaternions as tq

def plot_pose(ax, translation, quaternion, color, alpha=1.0):

    ax.scatter(translation[0], translation[1], translation[2], color=color, alpha=alpha, s=20)

    forward_direction = tq.rotate_vector(np.array([1,0,0]), quaternion)

    ax.quiver(translation[0], translation[1], translation[2], 
          forward_direction[0], forward_direction[1], forward_direction[2], 
          length=0.01, color=color, alpha=alpha)

def matrix_from_pose(t,q):
    transformation_matrix = ta.compose(t, tq.quat2mat(q), np.ones(3))

    return transformation_matrix

def matrix_to_pose(matrix):
    translation, rotation_matrix, _, _ = ta.decompose44(matrix)
    quaternion = tq.mat2quat(rotation_matrix)

    translation = [round(t, 6) for t in translation]
    quaternion = [round(q, 6) for q in quaternion]
    return translation,quaternion

def read_cam0_data(csv_path):
    data = []
    
    # Read the CSV file
    with open(csv_path, 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader)  # Skip the header line
        first_timestamp = None
        
        for row in csv_reader:
            # Skip comment lines
            if row[0].startswith('#'):
                continue
                
            # Parse data
            row_data = [float(val) for val in row]

            data.append({'timestamp': row_data[0], 't': np.array(row_data[1:4]), 'q': np.array(row_data[4:8])})

    return data 

def read_ground_truth_data(csv_path):
    data = []
    
    # Read the CSV file
    with open(csv_path, 'r') as file:
        csv_reader = csv.reader(file)
        first_timestamp = None
        
        for row in csv_reader:
            # Skip comment lines
            if row[0].startswith('#'):
                continue
                
            # Parse data
            row_data = [float(val) for val in row]
            
            # Keep timestamp as nanoseconds for full precision
            timestamp_ns = int(row_data[0])
            timestamp_seconds = timestamp_ns / 1e9

            data.append({'timestamp': timestamp_seconds, 'timestamp_ns': timestamp_ns, 't': np.array(row_data[1:4]), 'q': np.array(row_data[4:8])})

    return data 

def main():
    cam0_csv_data = "cam0_estimated.csv"
    gt_csv_data = "/Users/pitinari/Desktop/robotica/TP1/mav0/state_groundtruth_estimate0/data.csv"

    # Read the data
    print("Reading cam0 data...")
    cam0_data = read_cam0_data(cam0_csv_data)

    print("Reading ground truth data...")
    gt_data = read_ground_truth_data(gt_csv_data)

    # Display first few rows
    print(cam0_data[:3])
    print(gt_data[:3])

    # Read camera sensor parameters
    imu_to_body_matrix = np.array([
        [1.0, 0.0, 0.0, 0.0],
         [0.0, 1.0, 0.0, 0.0],
         [0.0, 0.0, 1.0, 0.0],
         [0.0, 0.0, 0.0, 1.0]])
    body_to_imu_matrix = np.linalg.inv(imu_to_body_matrix)
    
    cam0_to_body_matrix = np.array([
        [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
        [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
        [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
        [0.0, 0.0, 0.0, 1.0]])
    cam0_initial_matrix = cam0_to_body_matrix
    body_to_cam0_matrix = np.linalg.inv(cam0_to_body_matrix)
    
    body_initial_pose_t = gt_data[0]['t']
    body_initial_pose_q = gt_data[0]['q']
    body_to_world = matrix_from_pose(body_initial_pose_t, body_initial_pose_q)
    
    cam0_to_world = np.dot(body_to_world, cam0_to_body_matrix)

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Determine the minimum length to avoid index errors
    num_poses_to_plot = 50000
    min_length = min(len(cam0_data), len(gt_data), num_poses_to_plot)
    
    # Create color maps for gradient effect
    cam0_colormap = plt.cm.Blues  # Blue gradient for cam0
    gt_colormap = plt.cm.Reds     # Red gradient for ground truth
    
    # Calculate step size for iteration
    step_size = 20
    num_iterations = len(range(0, min_length, step_size))
    
    for idx, i in enumerate(range(0, min_length, step_size)):
        # Calculate color intensity based on iteration (0 to 1)
        color_intensity = idx / max(1, num_iterations - 1)
        
        # Get colors from colormap
        cam0_color = cam0_colormap(0.5 + 0.5 * color_intensity)  # Start from light blue to dark blue
        gt_color = gt_colormap(0.5 + 0.5 * color_intensity)      # Start from light red to dark red
        
        # Calculate alpha for fade effect (older poses more transparent)
        alpha = 0.8 + 0.2 * color_intensity
        
        # Get i-th row from cam0_data
        cam0_row = cam0_data[i]
        cam0_t = cam0_row['t']
        cam0_q = cam0_row['q']
        
        # Transform cam0 pose
        cam0_current_matrix = matrix_from_pose(cam0_t, cam0_q)
        current_cam0 = np.dot(cam0_to_world, cam0_current_matrix)
        current_cam0_t, current_cam0_q = matrix_to_pose(current_cam0)
        
        # Get i-th row from gt_data
        gt_row = gt_data[i]
        gt_t = gt_row['t']
        gt_q = gt_row['q']

        # Plot poses with gradient colors
        plot_pose(ax, current_cam0_t, current_cam0_q, cam0_color, alpha)  # Gradient blue for cam0
        plot_pose(ax, gt_t, gt_q, gt_color, alpha)  # Gradient red for ground truth

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create legend handles with gradient explanation
    cam0_handle = ax.scatter([], [], [], color='blue', label='Cam0 (light→dark blue = start→end)')
    gt_handle = ax.scatter([], [], [], color='red', label='IMU (light→dark red = start→end)')

    ax.legend(handles=[cam0_handle, gt_handle])
    
    # Add a colorbar to show the time progression
    sm_blue = plt.cm.ScalarMappable(cmap=cam0_colormap, norm=plt.Normalize(vmin=0, vmax=1))
    sm_blue.set_array([])
    sm_red = plt.cm.ScalarMappable(cmap=gt_colormap, norm=plt.Normalize(vmin=0, vmax=1))
    sm_red.set_array([])
    
    print(f"Plotted {num_iterations} poses from both datasets with gradient colors")
    print("Light colors = early poses, Dark colors = later poses")

    plt.show()

if __name__ == "__main__":
    main()