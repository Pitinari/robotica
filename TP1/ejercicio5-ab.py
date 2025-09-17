import csv
import math
import numpy as np
import transforms3d.affines as ta
import transforms3d.quaternions as tq

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

def matrix_from_pose(t,q):
    transformation_matrix = ta.compose(t, tq.quat2mat(q), np.ones(3))

    return transformation_matrix

def matrix_to_pose(matrix):
    translation, rotation_matrix, _, _ = ta.decompose44(matrix)
    quaternion = tq.mat2quat(rotation_matrix)

    translation = [round(t, 6) for t in translation]
    quaternion = [round(q, 6) for q in quaternion]
    return translation,quaternion

# Main execution
def main():
    # Path to the ground truth data
    ground_truth_csv_path = "/Users/pitinari/Desktop/robotica/TP1/mav0/state_groundtruth_estimate0/data.csv"
    outpath = "cam0_estimated.csv"

    # Read the data
    print("Reading ground truth data...")
    gt_data = read_ground_truth_data(ground_truth_csv_path)
    
    # Display first few rows
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
    world_to_body = np.linalg.inv(body_to_world)
    world_to_cam0 = np.dot(body_to_cam0_matrix, world_to_body)
    output = []

    for row in gt_data:
        current_t = row['t']
        current_q = row['q']
        current_matrix = matrix_from_pose(current_t, current_q)
        current_cam0 = np.dot(np.dot(world_to_cam0, current_matrix), cam0_initial_matrix)
        current_cam0_t, current_cam0_q = matrix_to_pose(current_cam0)
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