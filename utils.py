import os
import glob
import numpy as np
import matplotlib.pyplot as plt

def read_measurement(file_path):
    '''
    Return a dictionary 
        key: 'POINT_ID_CURRENT_MESUREMENT'
        value: dictionary with keys: [ ACTUAL_POINT_ID, IMAGE_POINT, APPEARANCE ]
    '''

    measurements = []
    point_data = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            if line.startswith('\n'):
                continue
            
            data = line.strip().split()
            line_id = str(data[0]).split(":")[0]

            if line_id == "point":
                
                point_id_current_measurement = int(data[1])
                point_data[point_id_current_measurement] = {
                    "actual_point_id":  int(data[2]), 
                    "image_point":  [float(data[3]), float(data[4])],
                    "appearance": [ float(x) for x in data[5:]] 
                }

                measurements.append(point_data)
                
            else:
                continue
                # For insert gt_pose and odom pose in dictionary but they are the same in trajectory.dat    
                if line_id != "seq":
                    point_data[line_id] = [ float(x) for x in data[1:]  ]

    return measurements

def read_world(file_path):
    '''
    Return a dictionary 
        key: 'LANDMARK_ID'
        value: dictionary with keys: [ POSITION, APPEARANCE ]
    '''
    landmarks = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            if line.startswith('\n'):
                continue

            data = line.strip().split()
            landmark_id = int(data[0])
            landmarks[landmark_id] = {
                'position':  [ float(x) for x in data[1:4]] ,
                'apperance': [ float(x) for x in data[4:]] 
            }

            
    return landmarks

def read_trajectory(file_path):
    '''
    Return a dictionary 
        key: pose_ID
        value: dictionary with keys: [ ODOMETRY_POSE, GROUNDTRUTH_POSE ]
    '''
    trajectory = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            if line.startswith('\n'):
                continue

            data = line.strip().split()
            pose_id = int(data[0])
            trajectory[pose_id] = {
                'odometry_pose':  [ float(x) for x in data[1:4]] ,
                'groundtruth_pose': [ float(x) for x in data[4:]] 
            }

    return trajectory

def print_one_measurement_point(measurements):
    '''
    For debug purposes: print of entry of measurement dictionary
    '''

    for meas_file in measurements:
        print(meas_file)
        for meas_dict in measurements[meas_file]:
            
            for point_id_current_measurement in meas_dict.keys():
                
                print(f"key id: {point_id_current_measurement}")
                print(f"actual_point_id: {meas_dict[point_id_current_measurement]['actual_point_id']}")
                print(f"image_point: {meas_dict[point_id_current_measurement]['image_point']}")
                print(f"appearance: (len: {len(meas_dict[point_id_current_measurement]['appearance'])}) {meas_dict[point_id_current_measurement]['appearance']}")
                break
            break
        break

def print_one_world_point(world):
    '''
    For debug purposes: print of entry of world dictionary
    '''

    for world_point in world:

        print(f"landmark id: {world_point}")
        print(f"position: (len: {len(world[world_point]['position'])}) {world[world_point]['position']}")
        print(f"appearance: (len: {len(world[world_point]['apperance'])}) {world[world_point]['apperance']}")
             
        break

def display_world(world):
    """
    dictionary world
        key: 'LANDMARK_ID'
        value: dictionary with keys: [ POSITION, APPEARANCE ]
    """
    
    # Taking ids and positions
    landmark_ids = list(world.keys())
    positions = np.array([world[lm]["position"] for lm in landmark_ids])

    # creating figure and plotting point
    fig = plt.figure(figsize=(12, 8))  
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='blue', s=10, label="World Points")

    # display id every 10 landmarks
    for i in range(0, len(landmark_ids), 10):  
        ax.text(positions[i, 0], positions[i, 1], positions[i, 2], str(landmark_ids[i]), color='red', fontsize=12)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("World Map")
    ax.legend()
    plt.show()

def display_trajectory(odom_dict, gt_dict):
    
    gt_poses = np.array([gt_dict[pose_id]["groundtruth_pose"] for pose_id in sorted(gt_dict.keys())])
    odom_poses = np.array([odom_dict[pose_id]["odometry_pose"] for pose_id in sorted(odom_dict.keys())])

    # Creating plot
    fig =plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(gt_poses[:, 0], gt_poses[:, 1], gt_poses[:, 2], 'r-', label="Ground Truth Trajectory")
    ax.plot(odom_poses[:, 0], odom_poses[:, 1], odom_poses[:, 2], 'b-', label="Odometry Trajectory")
    
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("Trajectory Comparison")
    ax.legend()
    ax.grid()
    ax.legend()
    
    plt.show()

files = glob.glob("data/meas-*.dat")
measurements = {}

for file_path in files:
    file_id = int(os.path.basename(file_path).split('-')[1].split('.')[0]) 
    measurements[file_id] = read_measurement(file_path)

world = read_world("data/world.dat")
traj = read_trajectory("data/trajectoy.dat")

gt_traj = {}
odom_traj = {}

for pose_id, data in traj.items():
    gt_traj[pose_id] = {"groundtruth_pose": data["groundtruth_pose"]}
    odom_traj[pose_id] = {"odometry_pose": data["odometry_pose"]}

display_trajectory(odom_traj, gt_traj)
