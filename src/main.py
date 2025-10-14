# from pypcd import pypcd
from pypcd4 import pypcd4 as pypcd 

import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt

path = "/home/eugeniu/Downloads/000050.pcd"
pcd = pypcd.PointCloud.from_path(path)

#available fields
print("Fields:", pcd.fields)
print("Number of points:", pcd.points)

data = pcd.pc_data

print("data:", np.shape(data))

x = data['x']
y = data['y']
z = data['z']
ring = data['ring']

points = np.stack((x, y, z, ring), axis=1)  # shape (N, 4)

# Parameters
N_SCAN = 128                
ang_res_x = 0.2             # horizontal angular resolution (degrees)
Horizon_SCAN = 2000         
max_range = 100.0           

# Kernel neighborhood offsets
kernel_row, kernel_col = 1, 5
_kernel = [(i, j) for i in range(-kernel_row, kernel_row + 1)
            for j in range(-kernel_col, kernel_col + 1)
            if not (i == 0 and j == 0)]

# print("_kernel:\n",_kernel)

def project_point(pt):
    x, y, z, ring_val, intensity = pt[:5]
    row = int(ring_val)
    if row < 0 or row >= N_SCAN:
        return None

    horizon_angle = np.degrees(np.arctan2(x, y))
    col = -round((horizon_angle - 90.0) / ang_res_x) + Horizon_SCAN // 2

    if col >= Horizon_SCAN:
        col -= Horizon_SCAN
    if col < 0 or col >= Horizon_SCAN:
        return None
    return row, col

# Project to range image 
def project_to_range_image(x, y, z, ring):
    index_out = np.full((N_SCAN, Horizon_SCAN), -1, dtype=np.int32)
    range_image = np.zeros((N_SCAN, Horizon_SCAN), dtype=np.uint8)

    ranges = np.sqrt(x**2 + y**2 + z**2)
    cloud_trimmed = []

    for idx in range(len(x)):
        r = ranges[idx]
        if r < 0.1 or r > max_range:
            continue

        row = int(ring[idx])
        if row < 0 or row >= N_SCAN:
            continue

        horizon_angle = np.degrees(np.arctan2(x[idx], y[idx]))
        col = -round((horizon_angle - 90.0) / ang_res_x) + Horizon_SCAN // 2

        if col >= Horizon_SCAN:
            col -= Horizon_SCAN
        if col < 0 or col >= Horizon_SCAN:
            continue

        range_image[row, col] = int((r / max_range) * 255)
        index_out[row, col] = int(idx)
        cloud_trimmed.append((x[idx], y[idx], z[idx], r, row, col))

    # Visualization of range image
    colored = cv2.applyColorMap(range_image, cv2.COLORMAP_JET)
    colored = cv2.flip(colored, 0)  # 0 = flip vertically
    cv2.imshow("Range Image", colored)
    cv2.waitKey(1)

    return np.array(cloud_trimmed), index_out

# Rank computation 
def compute_rank(cloud, cloud_trimmed, Neighbours):
    densityMat = np.zeros((N_SCAN, Horizon_SCAN), dtype=np.uint8)
    cloud_ranked = []

    for idx, pt in enumerate(cloud_trimmed):
        x, y, z, range_curr, row, col = pt
        row, col = int(row), int(col)
        rank = 0.0
        count = 0

        for dy, dx in _kernel:
            y_ = row + dy
            x_ = col + dx

            if y_ < 0 or y_ >= N_SCAN:
                continue

            if x_ < 0:
                x_ = Horizon_SCAN - 1 + x_
            if x_ >= Horizon_SCAN:
                x_ = x_ - Horizon_SCAN - 1

            x_ = int(x_)
            y_ = int(y_)

            if x_ < 0 or x_ >= Horizon_SCAN:
                continue

            idx_n = Neighbours[y_, x_]
            if idx_n > 0:
                r_neigh = np.sqrt(cloud[idx_n,0]**2 + cloud[idx_n,1]**2 + cloud[idx_n,2]**2)
                diff = r_neigh - range_curr
                rank += np.exp(-0.5 * diff * diff)
                count += 1

        if count > 0:
            rank /= count

        weight = range_curr / max_range
        rank = (1 + rank) * (1 + weight)
        cloud_ranked.append((x, y, z, rank, row, col))

        densityMat[row, col] = min(int(rank * 100), 255)

    colored = cv2.applyColorMap(densityMat, cv2.COLORMAP_RAINBOW)
    colored = cv2.flip(colored, 0)  # 0 = flip vertically
    cv2.imshow("FPR", colored)
    cv2.waitKey(1)

    return np.array(cloud_ranked)

#  Filter by lowest Rank percentage 
def filter_by_rank_percent(points, drop_percent):
    if len(points) == 0 or drop_percent <= 0:
        return points

    drop_percent = np.clip(drop_percent, 0, 100)
    num_to_drop = int((drop_percent / 100.0) * len(points))

    sorted_idx = np.argsort(points[:, 3])  # sort by (rank)
    kept_idx = sorted_idx[num_to_drop:]
    filtered = points[kept_idx]
    print(f"Dropped {num_to_drop}/{len(points)} points ({drop_percent:.1f}%)")
    return filtered


# pipeline 
cloud_trimmed, Neighbours = project_to_range_image(x, y, z, ring)
cloud_ranked = compute_rank(points, cloud_trimmed, Neighbours)
filtered = filter_by_rank_percent(cloud_ranked, 5.0) #drop 5 % of lowest rank points


visualize = True
if visualize:
    cmap = plt.get_cmap('jet')
    print("cloud_ranked:{}, filtered:{}".format(np.shape(cloud_ranked), np.shape(filtered)))
    
    points = cloud_ranked[:, :3]
    rank = cloud_ranked[:, 3]
    rank_norm = (rank - np.min(rank)) / (np.max(rank) - np.min(rank) + 1e-8)
    colors = cmap(rank_norm)[:, :3]  
    pcd_orig = o3d.geometry.PointCloud()
    pcd_orig.points = o3d.utility.Vector3dVector(points)
    pcd_orig.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd_orig], window_name="Original Point Cloud")

    points = filtered[:, :3]
    rank = filtered[:, 3]
    rank_norm = (rank - np.min(rank)) / (np.max(rank) - np.min(rank) + 1e-8)
    colors = cmap(rank_norm)[:, :3]   
    pcd_filtered = o3d.geometry.PointCloud()
    filtered[:, 2] += 50 #add 50m on z to shift it up in the visualization 
    pcd_filtered.points = o3d.utility.Vector3dVector(points)
    pcd_filtered.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd_filtered], window_name="Filtered Point Cloud")


print("Press enter")
cv2.waitKey(0)
cv2.destroyAllWindows()