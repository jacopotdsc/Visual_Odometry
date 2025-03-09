import os
import glob

def read_measurement(file_path):
    measurements = []
    point_data = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            if line.startswith('\n'):
                break
            
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


files = glob.glob("data/meas-*.dat")
measurements = {}

for file_path in files:
    file_id = int(os.path.basename(file_path).split('-')[1].split('.')[0]) 
    measurements[file_id] = read_measurement(file_path)
