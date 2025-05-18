# Probabilist_Robotics: Visual Odometry 

## 🧠 Abstract

This project implements a **Visual Odometry (VO)** system that estimates the motion of a camera in a 3D environment using a sequence of monocular images. It combines techniques such as epipolar geometry, triangulation, and point cloud alignment (PICP) to reconstruct the camera trajectory and generate a sparse 3D map of the observed world.

The system is designed for educational and research purposes and includes various modules for camera modeling, geometric estimation, data handling, and performance evaluation.

---

## 📁 Repository Structure

```
.
├── include/             # Header files (camera model, geometry, utils, etc.)
├── src/                 # Source files and tests
│   └── test_file/       # Unit and integration tests
├── data/                # Input data (camera intrinsics, measurements, world ground truth)
├── exec/                # Executables, result plots, and output logs
├── build/               # Build directory (CMake output)
├── CMakeLists.txt       # CMake configuration
├── README.md            # This file
```

---

## 🚀 How to Run

1. **Build the project:**
   ```bash
   cd <path>/Visual_Odometry && mkdir build && cd build && cmake .. && make
   ```

2. **Run the executables:**
   ```bash
   ./exec/vo_system                   # Look VO system working
   ./exec/vo_system_evaluation        # VO system with evaluation information
   ./exec/vo_system_evaluation_map    # VO system with map information
   ```

3. **Visualize results:**
   ```bash
   python3 exec/show_trajectory.py <gt_file> <est_file>  # Plot camera trajectory
   python3 exec/show_map.py <gt_map> <est_map.txt>       # Plot landmark map
   ```

---

## 📏 Metrics Used

### 1. **Trajectory Evaluation**  
Measures the ratio between the estimated and ground truth relative translations:

$$
\sum_i \frac{ \left\| \mathbf{T}_i(1{:}3\,4) \right\| }{ \left\| \mathbf{T}_i^{\text{gt}}(1{:}3\,4) \right\| }
$$

---

### 2. **Map Evaluation**  
Compares estimated 3D landmarks against the ground truth positions using RMSE:

$$
RMSE = \frac{1}{N} \sqrt{ \sum_{i=1}^N \| \mathbf{X}_i^{\text{gt}} - \mathbf{X}_i^{\text{est}} \|^2 }
$$

---

### 3. **Translation and Rotation Errors**

For each estimated pose compared with ground truth:

- **Translation Evaluation**: Cumulative translation error over all frames.  
- **Rotation Evaluation**: Cumulative rotation error over all frames.

**Variance values:**

$$
\mathrm{Var}(\mathbf{e}) = \frac{1}{N - 1} \sum_{i=1}^N \left( \mathbf{e}_i - \bar{\mathbf{e}} \right)^2
$$

Where:
- $\mathbf{e}_i = \mathbf{t}_i^{\text{gt}} - \mathbf{t}_i^{\text{est}}$
- $\bar{\mathbf{e}} = \frac{1}{N} \sum_{i=1}^N \mathbf{e}_i$

Component-wise translation error and variance are also computed to capture per-axis accuracy.

---


---

## 📊 Plots

All result plots are located in the `exec/` folder:

- `trajectory_complete.txt` — Estimated camera trajectory
- `trajectory_gt.txt` — Ground truth trajectory
- `landmarks_comparison.png` — Comparison between estimated and true 3D landmarks

To generate plots:
```bash
python3 exec/show_trajectory.py trajectory_gt.txt trajectory_complete.txt
python3 exec/show_map.py world.dat result_map.txt
```


