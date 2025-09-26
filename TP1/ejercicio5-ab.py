import csv
import numpy as np
from pose_utils import read_pose_data, matrix_from_pose, matrix_to_pose, get_body_to_world_transform, CAM0_TO_BODY, BODY_TO_CAM0

# Main execution
def main():
    # Path to the ground truth data
    ground_truth_csv_path = "/Users/pitinari/Desktop/robotica/TP1/mav0/state_groundtruth_estimate0/data.csv"
    outpath = "cam0_estimated.csv"

    # Read the data
    print("Reading ground truth data...")
    gt_data = read_pose_data(ground_truth_csv_path, timestamp_in_ns=True)
    
    # Display first few rows
    print(gt_data[:3])

    body_to_world = get_body_to_world_transform(gt_data)
    world_to_body = np.linalg.inv(body_to_world)
    world_to_cam0 = np.dot(BODY_TO_CAM0, world_to_body)
    output = []

    for row in gt_data:
        current_t = row['t']
        current_q = row['q']
        current_matrix = matrix_from_pose(current_t, current_q)
        # Apply transformation: world -> body -> cam0
        # Raro esto porque seria
        current_cam0_matrix = world_to_cam0 @ current_matrix @ CAM0_TO_BODY
        
        # Decompose back to t and q
        current_cam0_t, current_cam0_q = matrix_to_pose(current_cam0_matrix)
        
        output.append({'timestamp': row['timestamp'], 't': current_cam0_t, 'q': current_cam0_q})

    with open(outpath, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=["timestamp", "x", "y", "z", "qw", "qx", "qy", "qz"])
        
        writer.writeheader()
        for row in output:
            writer.writerow({
                "timestamp": f"{row['timestamp']:.9f}",
                "x":  row['t'][0],
                "y":  row['t'][1],
                "z":  row['t'][2],
                "qw": row['q'][0],
                "qx": row['q'][1],
                "qy": row['q'][2],
                "qz": row['q'][3]})
    print(f"Transformed data written to {outpath}")

if __name__ == "__main__":
    main()