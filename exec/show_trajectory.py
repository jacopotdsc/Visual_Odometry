import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  
import argparse
import os
import re

def load_file(filename):
    ext = os.path.splitext(filename)[-1].lower()
    if ext == ".csv":
        return np.loadtxt(filename, delimiter=",", skiprows=1)
    elif ext == ".txt":
        with open(filename, 'r') as f:
            lines = f.readlines()
        data = []
        for line in lines:
            # Usa regex per separare su spazi multipli, tab o virgole
            parts = re.split(r'[\s,]+', line.strip())
            if len(parts) >= 3:
                data.append([float(p) for p in parts[:3]])
        return np.array(data)
    else:
        raise ValueError(f"Unsupported file format: {ext}")

def plot_trajectories(gt_file='gt_trajectory.csv', est_file='estimated_trajectory.csv'):
    gt = load_file(gt_file)
    est = load_file(est_file)

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Ground Truth
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], 'o-', label='Ground Truth', color='blue')
    # Estimated
    ax.plot(est[:, 0], est[:, 1], est[:, 2], 'x--', label='Estimated', color='red')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectories: Ground Truth vs Estimated')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot 3D trajectories from two CSV files")
    parser.add_argument('gt_file', help="Path to the ground truth CSV file")
    parser.add_argument('est_file', help="Path to the estimated trajectory CSV file")
    args = parser.parse_args()

    plot_trajectories(args.gt_file, args.est_file)
