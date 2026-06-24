#!/usr/bin/env python3
"""
ate_analysis.py — computes Absolute Trajectory Error (ATE) between
SLAM-estimated pose and Gazebo ground truth, after rigid SE(2) alignment.

Input:  localization_log.csv  (t, gt_x, gt_y, gt_yaw, slam_x, slam_y, slam_yaw)
Output: ATE stats printed to console + two plots saved as PNGs.
"""
import csv
import numpy as np
import matplotlib.pyplot as plt

CSV_PATH = 'localization_log.csv'


def load_and_clean(path):
    rows = list(csv.DictReader(open(path)))

    all_t = [float(r['t']) for r in rows]
    resets = [i for i in range(1, len(all_t)) if all_t[i] < all_t[i - 1] - 1.0]
    if resets:
        start = resets[-1]
        print(f'Detected {len(resets)} run boundary(ies); keeping rows from '
              f'index {start} onward (most recent run only).')
        rows = rows[start:]

    t, gt, slam = [], [], []
    dropped = 0
    for r in rows:
        gx, gy = float(r['gt_x']), float(r['gt_y'])
        if gx == 0.0 and gy == 0.0:
            dropped += 1
            continue
        t.append(float(r['t']))
        gt.append((gx, gy))
        slam.append((float(r['slam_x']), float(r['slam_y'])))
    print(f'Loaded {len(rows)} rows, dropped {dropped} glitch rows, kept {len(t)}.')
    return np.array(t), np.array(gt), np.array(slam)


def umeyama_se2(src, dst):
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean

    H = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    S = np.diag([1, d])
    R = Vt.T @ S @ U.T

    t_vec = dst_mean - R @ src_mean
    return R, t_vec


def main():
    t, gt, slam = load_and_clean(CSV_PATH)

    R, t_vec = umeyama_se2(slam, gt)
    slam_aligned = (slam @ R.T) + t_vec

    err = np.linalg.norm(slam_aligned - gt, axis=1)
    ate_rmse = np.sqrt(np.mean(err ** 2))
    ate_mean = err.mean()
    ate_max = err.max()
    final_drift = err[-1]

    print(f'ATE RMSE:     {ate_rmse:.4f} m')
    print(f'ATE mean:     {ate_mean:.4f} m')
    print(f'ATE max:      {ate_max:.4f} m')
    print(f'Final drift:  {final_drift:.4f} m')

    plt.figure(figsize=(7, 7))
    plt.plot(gt[:, 0], gt[:, 1], label='Ground truth', linewidth=2)
    plt.plot(slam_aligned[:, 0], slam_aligned[:, 1], label='SLAM (aligned)',
              linewidth=1.5, linestyle='--')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Trajectory: ground truth vs SLAM (aligned)')
    plt.axis('equal')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig('trajectory_overlay.png', dpi=150, bbox_inches='tight')
    print('Saved trajectory_overlay.png')

    t_rel = t - t[0]
    plt.figure(figsize=(9, 4))
    plt.plot(t_rel, err)
    plt.xlabel('time (s)')
    plt.ylabel('position error (m)')
    plt.title('Localization error over time')
    plt.grid(True, alpha=0.3)
    plt.savefig('error_over_time.png', dpi=150, bbox_inches='tight')
    print('Saved error_over_time.png')


if __name__ == '__main__':
    main()
