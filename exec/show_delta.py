import matplotlib.pyplot as plt
import argparse

def plot_delta_comparison_2d(file_path):
    error_x = []
    error_y = []
    ratio_x = []
    ratio_y = []

    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue

            parts = list(map(float, line.strip().split()))
            if len(parts) != 9:
                print("Skipping invalid line:", line)
                continue

            delta_gt_x, delta_gt_y = parts[0], parts[1]
            delta_est_x, delta_est_y = parts[3], parts[4]

            # Error
            error_x.append(delta_gt_x - delta_est_x)
            error_y.append(delta_gt_y - delta_est_y)

            # Ratio with safeguard
            ratio_x.append(delta_est_x / delta_gt_x if abs(delta_gt_x) > 1e-6 else 0.0)
            ratio_y.append(delta_est_y / delta_gt_y if abs(delta_gt_y) > 1e-6 else 0.0)

    steps = list(range(len(error_x)))

    # Create subplots: 2 rows, 1 column
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot errors
    ax1.plot(steps, error_x, label="Error in ΔX", marker='o')
    ax1.plot(steps, error_y, label="Error in ΔY", marker='x')
    ax1.set_ylabel("Error")
    ax1.set_title("Delta Error over Time")
    ax1.legend()
    ax1.grid(True)

    # Plot ratios
    ax2.plot(steps, ratio_x, label="Ratio ΔX (est/gt)", linestyle='--')
    ax2.plot(steps, ratio_y, label="Ratio ΔY (est/gt)", linestyle='--')
    ax2.set_xlabel("Step")
    ax2.set_ylabel("Ratio")
    ax2.set_title("Delta Ratio over Time")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot 2D delta_gt and delta_est from a comparison file")
    parser.add_argument("file_path", help="Path to the delta comparison .txt file")
    args = parser.parse_args()

    plot_delta_comparison_2d(args.file_path)
