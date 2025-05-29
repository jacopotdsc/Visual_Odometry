import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  
import argparse
import os
import re

def load_landmarks(filename):
    ext = os.path.splitext(filename)[-1].lower()
    if not (ext == ".txt" or ext == ".dat" ):
        raise ValueError(f"Unsupported file format: {ext}")

    with open(filename, 'r') as f:
        lines = f.readlines()

    landmarks = []
    for line in lines:
        parts = re.split(r'[\s,]+', line.strip())
        if len(parts) < 5:
            continue
        land_id = parts[0]
        x, y, z = map(float, parts[1:4])
        appearance = " ".join(parts[4:])
        #landmarks.append((land_id, x, y, z, appearance))
        landmarks.append((land_id, x, y, z))
    
    return landmarks

def plot_landmarks_3d(gt_file, est_file, frequency=1.0):
    assert 0.0 < frequency <= 1.0, "Frequency must be in (0, 1]"

    gt_landmarks = load_landmarks(gt_file)
    est_landmarks = load_landmarks(est_file)

    # Assumiamo che gt ed est abbiano lo stesso ordine e numero di elementi
    min_len = min(len(gt_landmarks), len(est_landmarks))

    # Calcola quanti landmark plottare
    num_to_plot = max(1, int(min_len * frequency))

    # Puoi anche usare random.sample se vuoi punti sparsi casuali
    indices_to_plot = list(range(min_len))[:num_to_plot]

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot selezionati
    for i in indices_to_plot:
        land_id_gt, x_gt, y_gt, z_gt = gt_landmarks[i]
        land_id_est, x_est, y_est, z_est = est_landmarks[i]

        # Ground truth point
        ax.scatter(x_gt, y_gt, z_gt, color='blue', label='Ground Truth' if i == indices_to_plot[0] else "")
        ax.text(x_gt, y_gt, z_gt, land_id_gt, fontsize=8, color='blue')

        # Estimated point
        ax.scatter(x_est, y_est, z_est, color='red', marker='x', label='Estimated' if i == indices_to_plot[0] else "")
        ax.text(x_est, y_est, z_est, land_id_est, fontsize=8, color='red')


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'3D Landmark Comparison (showing {frequency*100:.0f}% of points)')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig("landmarks_comparison.png")
    plt.show()

def plot_landmarks_2d(gt_file, est_file, frequency=1.0):
    assert 0.0 < frequency <= 1.0, "Frequency must be in (0, 1]"

    gt_landmarks = load_landmarks(gt_file)
    est_landmarks = load_landmarks(est_file)

    # Assumiamo che gt ed est abbiano lo stesso ordine e numero di elementi
    min_len = min(len(gt_landmarks), len(est_landmarks))
    num_to_plot = max(1, int(min_len * frequency))
    indices_to_plot = list(range(min_len))[:num_to_plot]

    fig, ax = plt.subplots(figsize=(12, 8))

    for i in indices_to_plot:
        land_id_gt, x_gt, y_gt, _ = gt_landmarks[i]
        land_id_est, x_est, y_est, _ = est_landmarks[i]

        # Ground truth point
        ax.scatter(x_gt, y_gt, color='blue', label='Ground Truth' if i == indices_to_plot[0] else "")
        ax.text(x_gt, y_gt, land_id_gt, fontsize=8, color='blue')

        # Estimated point
        ax.scatter(x_est, y_est, color='red', marker='x', label='Estimated' if i == indices_to_plot[0] else "")
        ax.text(x_est, y_est, land_id_gt, fontsize=8, color='red')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(f'2D Landmark Comparison (showing {frequency*100:.0f}% of points)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')  # opzionale, mantiene proporzioni corrette
    plt.tight_layout()
    plt.savefig("landmarks_comparison_2d.png")
    plt.show()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot ground truth and estimated 3D landmarks from two .txt files")
    parser.add_argument('gt_file', help="Path to the ground truth file")
    parser.add_argument('est_file', help="Path to the estimated file")
    parser.add_argument('--frequency', type=float, help="Fraction of landmarks to plot (0 < freq <= 1)", default=0.1)

    args = parser.parse_args()

    if not (0.0 < args.frequency <= 1.0):
        raise ValueError("Frequency must be a float between 0 (exclusive) and 1 (inclusive).")

    plot_landmarks_2d(args.gt_file, args.est_file, frequency=args.frequency)

