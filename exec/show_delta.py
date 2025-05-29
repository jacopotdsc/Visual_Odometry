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

    # Primo grafico: errori
    plt.figure(figsize=(10, 4))
    plt.plot(steps, error_x, label="Error in ΔX", marker='o')
    plt.plot(steps, error_y, label="Error in ΔY", marker='x')
    plt.ylabel("Error")
    plt.title("Delta Error over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("delta_error.png")
    plt.show()

    # Secondo grafico: rapporti
    plt.figure(figsize=(10, 4))
    plt.plot(steps, ratio_x, label="Ratio ΔX (est/gt)", linestyle='--')
    plt.plot(steps, ratio_y, label="Ratio ΔY (est/gt)", linestyle='--')
    plt.xlabel("Step")
    plt.ylabel("Ratio")
    plt.title("Delta Ratio over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("delta_ratio.png")
    plt.show()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot 2D delta_gt and delta_est from a comparison file")
    parser.add_argument("file_path", help="Path to the delta comparison .txt file")
    args = parser.parse_args()

    plot_delta_comparison_2d(args.file_path)
