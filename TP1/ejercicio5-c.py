import numpy as np
import matplotlib.pyplot as plt
from pose_utils import read_pose_data, matrix_from_pose, matrix_to_pose, get_body_to_world_transform, CAM0_TO_BODY
import transforms3d.quaternions as tq

def plot_pose(ax, translation, quaternion, color, alpha=1.0):
    """Plots a 3D pose on a matplotlib Axes3D object."""
    ax.scatter(translation[0], translation[1], translation[2], color=color, alpha=alpha, s=20)
    forward_direction = tq.rotate_vector(np.array([1, 0, 0]), quaternion)
    ax.quiver(translation[0], translation[1], translation[2], 
              forward_direction[0], forward_direction[1], forward_direction[2], 
              length=0.01, color=color, alpha=alpha)

def main():
    cam0_csv_data = "cam0_estimated.csv"
    gt_csv_data = "/Users/pitinari/Desktop/robotica/TP1/mav0/state_groundtruth_estimate0/data.csv"

    # Read the data
    print("Reading cam0 data...")
    cam0_data = read_pose_data(cam0_csv_data, has_header=True)

    print("Reading ground truth data...")
    gt_data = read_pose_data(gt_csv_data, timestamp_in_ns=True)

    # Display first few rows
    print("Cam0 Data:", cam0_data[:3])
    print("Ground Truth Data:", gt_data[:3])

    body_to_world = get_body_to_world_transform(gt_data)
    cam0_to_world = np.dot(body_to_world, CAM0_TO_BODY)

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    num_poses_to_plot = 50000
    min_length = min(len(cam0_data), len(gt_data), num_poses_to_plot)
    
    cam0_colormap = plt.cm.Blues
    gt_colormap = plt.cm.Reds
    
    step_size = 20
    iterations = range(0, min_length, step_size)
    num_iterations = len(iterations)
    
    for idx, i in enumerate(iterations):
        color_intensity = idx / max(1, num_iterations - 1)
        
        cam0_color = cam0_colormap(0.5 + 0.5 * color_intensity)
        gt_color = gt_colormap(0.5 + 0.5 * color_intensity)
        
        alpha = 0.8 + 0.2 * color_intensity
        
        cam0_row = cam0_data[i]
        cam0_t, cam0_q = cam0_row['t'], cam0_row['q']
        
        cam0_current_matrix = matrix_from_pose(cam0_t, cam0_q)
        current_cam0_world = np.dot(cam0_to_world, cam0_current_matrix)
        current_cam0_t, current_cam0_q = matrix_to_pose(current_cam0_world)
        
        gt_row = gt_data[i]
        gt_t, gt_q = gt_row['t'], gt_row['q']

        plot_pose(ax, current_cam0_t, current_cam0_q, cam0_color, alpha)
        plot_pose(ax, gt_t, gt_q, gt_color, alpha)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    cam0_handle = ax.scatter([], [], [], color='blue', label='Cam0 (light→dark blue = start→end)')
    gt_handle = ax.scatter([], [], [], color='red', label='IMU (light→dark red = start→end)')

    ax.legend(handles=[cam0_handle, gt_handle])
    
    print(f"Plotted {num_iterations} poses from both datasets with gradient colors.")
    print("Light colors = early poses, Dark colors = later poses.")

    plt.show()

if __name__ == "__main__":
    main()