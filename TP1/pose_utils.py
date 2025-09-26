import csv
import numpy as np
import transforms3d.affines as ta
import transforms3d.quaternions as tq

def read_pose_data(csv_path, has_header=False, timestamp_in_ns=False):
    """
    Reads pose data from a CSV file.

    Args:
        csv_path (str): Path to the CSV file.
        has_header (bool): Whether the CSV has a header line.
        timestamp_in_ns (bool): Whether the timestamp is in nanoseconds.

    Returns:
        list: A list of dictionaries, each containing pose data.
    """
    data = []
    with open(csv_path, 'r') as file:
        csv_reader = csv.reader(file)
        if has_header:
            next(csv_reader)
        
        for row in csv_reader:
            if row[0].startswith('#'):
                continue
            
            row_data = [float(val) for val in row]
            
            timestamp = row_data[0]
            if timestamp_in_ns:
                timestamp /= 1e9

            pose = {'timestamp': timestamp, 't': np.array(row_data[1:4]), 'q': np.array(row_data[4:8])}
            if timestamp_in_ns:
                pose['timestamp_ns'] = int(row_data[0])
            
            data.append(pose)
    return data

def matrix_from_pose(t, q):
    """Converts a pose (translation, quaternion) to a 4x4 transformation matrix."""
    return ta.compose(t, tq.quat2mat(q), np.ones(3))

def matrix_to_pose(matrix):
    """Converts a 4x4 transformation matrix to a pose (translation, quaternion)."""
    translation, rotation_matrix, _, _ = ta.decompose44(matrix)
    quaternion = tq.mat2quat(rotation_matrix)
    return np.array(translation), np.array(quaternion)

def get_body_to_world_transform(gt_data):
    """Computes the body-to-world transformation from the first ground truth pose."""
    body_initial_pose_t = gt_data[0]['t']
    body_initial_pose_q = gt_data[0]['q']
    return matrix_from_pose(body_initial_pose_t, body_initial_pose_q)

# Transformation matrices
IMU_TO_BODY = np.identity(4)
BODY_TO_IMU = np.linalg.inv(IMU_TO_BODY)

CAM0_TO_BODY = np.array([
    [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
    [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
    [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
    [0.0, 0.0, 0.0, 1.0]
])
BODY_TO_CAM0 = np.linalg.inv(CAM0_TO_BODY)
