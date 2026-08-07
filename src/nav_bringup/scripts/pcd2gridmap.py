#!/usr/bin/env python3
"""
FAST-LIO 点云预处理: 扶正(地面对齐水平) + 偏航摆正(墙体轴对齐) + 裁切。
输出处理后的 PCD,栅格化交给 pcd2pgm 包完成。

流程: RANSAC 找水平面粗扶正 -> 地面点精扶正、地面 z=0 -> 偏航摆正
      -> 自动/手动裁切主区域 -> 输出 PCD + 变换记录。

用法:
    pcd2gridmap.py <input.pcd> <output.pcd>
                   [--crop XMIN XMAX YMIN YMAX]   # 摆正后坐标系下手动裁切(米)
                   [--no-yaw]                     # 不做偏航摆正
"""
import argparse
import os
import sys

import numpy as np


def read_pcd_xyz(path):
    """读取二进制 PCD,返回 Nx3 xyz。"""
    with open(path, 'rb') as f:
        fields = []
        while True:
            line = f.readline().decode(errors='ignore').strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('DATA'):
                if 'binary' not in line:
                    raise ValueError('只支持 DATA binary 的 PCD')
                break
        data = np.frombuffer(f.read(), dtype=np.float32)
        pts = data.reshape(-1, len(fields))
        ix, iy, iz = fields.index('x'), fields.index('y'), fields.index('z')
    return pts[:, [ix, iy, iz]].astype(np.float64)


def write_pcd_xyz(path, pts):
    """把 Nx3 xyz 写成二进制 PCD。"""
    with open(path, 'wb') as f:
        f.write((f'# .PCD v0.7 - Point Cloud Data file format\n'
                 f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                 f'POINTS {len(pts)}\nDATA binary\n').encode())
        f.write(np.ascontiguousarray(pts, dtype=np.float32).tobytes())


def ransac_horizontal_plane(pts, iters=300, tol=0.08, seed=0):
    """RANSAC 拟合最大平面(通常是地面或天花板,都是水平面,用于扶正)。"""
    rng = np.random.default_rng(seed)
    sub = pts[rng.choice(len(pts), min(200000, len(pts)), replace=False)]
    best_inl, best_n = 0, None
    for _ in range(iters):
        p3 = sub[rng.choice(len(sub), 3, replace=False)]
        n = np.cross(p3[1] - p3[0], p3[2] - p3[0])
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        d = -n @ p3[0]
        inl = (np.abs(sub @ n + d) < tol).sum()
        if inl > best_inl:
            best_inl, best_n = inl, n
    n = best_n if best_n[2] > 0 else -best_n
    return n


def rotation_to_z(n):
    """求把向量 n 旋到 +Z 的旋转矩阵。"""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    c = n @ z
    if s < 1e-9:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s**2)


def find_floor_z(z, bins=200):
    """地面 = 高度直方图里最低的强峰。"""
    lo, hi = np.percentile(z, 1), np.percentile(z, 99)
    hist, edges = np.histogram(z, bins=bins, range=(lo, hi))
    centers = (edges[:-1] + edges[1:]) / 2
    strong = centers[hist > hist.max() * 0.1]
    return strong.min()


def find_yaw(xy, res=0.05):
    """扫描 0~90° 找让墙体最“横平竖直”的偏航角(直方图锐度最大)。"""
    best_score, best_th = -1, 0.0
    for th in np.deg2rad(np.arange(0, 90, 0.5)):
        c, s = np.cos(th), np.sin(th)
        score = 0.0
        for proj in (xy[:, 0] * c + xy[:, 1] * s, -xy[:, 0] * s + xy[:, 1] * c):
            hist, _ = np.histogram(proj, bins=int((proj.max() - proj.min()) / res) + 1)
            score += float((hist.astype(np.float64) ** 2).sum())
        if score > best_score:
            best_score, best_th = score, th
    return best_th


def auto_crop_box(xy, res=0.1, margin=0.3):
    """自动框主区域: 取占据密度最高的连通块的包围盒。"""
    x0, y0 = xy[:, 0].min(), xy[:, 1].min()
    gx = ((xy[:, 0] - x0) / res).astype(int)
    gy = ((xy[:, 1] - y0) / res).astype(int)
    grid = np.zeros((gx.max() + 1, gy.max() + 1), dtype=np.int32)
    np.add.at(grid, (gx, gy), 1)
    occ = grid >= max(3, int(np.percentile(grid[grid > 0], 60)))
    from collections import deque
    visited = np.zeros_like(occ, dtype=bool)
    best_cells = []
    for sx, sy in zip(*np.where(occ)):
        if visited[sx, sy]:
            continue
        q = deque([(sx, sy)])
        visited[sx, sy] = True
        cells = []
        while q:
            cx, cy = q.popleft()
            cells.append((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx_, ny_ = cx + dx, cy + dy
                    if (0 <= nx_ < occ.shape[0] and 0 <= ny_ < occ.shape[1]
                            and occ[nx_, ny_] and not visited[nx_, ny_]):
                        visited[nx_, ny_] = True
                        q.append((nx_, ny_))
        if len(cells) > len(best_cells):
            best_cells = cells
    cells = np.array(best_cells)
    return (x0 + cells[:, 0].min() * res - margin, x0 + (cells[:, 0].max() + 1) * res + margin,
            y0 + cells[:, 1].min() * res - margin, y0 + (cells[:, 1].max() + 1) * res + margin)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--crop', type=float, nargs=4, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    ap.add_argument('--no-yaw', action='store_true', help='不做偏航摆正')
    args = ap.parse_args()

    pts = read_pcd_xyz(args.input)
    print(f'载入 {len(pts)} 点')

    # 1. 扶正 + 地面对零
    n = ransac_horizontal_plane(pts)
    R = rotation_to_z(n)
    pts = pts @ R.T
    zf = find_floor_z(pts[:, 2])
    pts[:, 2] -= zf
    tilt = np.degrees(np.arccos(np.clip(n[2], -1, 1)))
    print(f'粗扶正: 倾角 {tilt:.1f}°, 地面高度 {zf:.2f}m -> 0')

    # 1b. 用地面附近的点二次精扶正,消除大平面(可能是天花板)带来的残差
    floor_cand = pts[np.abs(pts[:, 2]) < 0.25]
    if len(floor_cand) > 10000:
        n2 = ransac_horizontal_plane(floor_cand, tol=0.04, seed=1)
        R2 = rotation_to_z(n2)
        pts = pts @ R2.T
        zf2 = find_floor_z(pts[:, 2])
        pts[:, 2] -= zf2
        R = R2 @ R
        zf += zf2
        print(f'精扶正: 残差倾角 {np.degrees(np.arccos(np.clip(n2[2], -1, 1))):.2f}°, 地面微调 {zf2:.3f}m')

    # 2. 偏航摆正(用墙体层估计)
    yaw = 0.0
    wall_m = (pts[:, 2] > 0.3) & (pts[:, 2] < 1.6)
    if not args.no_yaw:
        rng = np.random.default_rng(0)
        wall_xy = pts[wall_m][:, :2]
        if len(wall_xy) > 200000:
            wall_xy = wall_xy[rng.choice(len(wall_xy), 200000, replace=False)]
        yaw = find_yaw(wall_xy)
        c, s = np.cos(yaw), np.sin(yaw)
        pts[:, :2] = pts[:, :2] @ np.array([[c, s], [-s, c]]).T
        print(f'偏航摆正: {np.degrees(yaw):.1f}°')

    # 3. 裁切
    if args.crop:
        xmin, xmax, ymin, ymax = args.crop
    else:
        xmin, xmax, ymin, ymax = auto_crop_box(pts[wall_m][:, :2])
    print(f'裁切范围: x[{xmin:.1f},{xmax:.1f}] y[{ymin:.1f},{ymax:.1f}]')
    m = ((pts[:, 0] > xmin) & (pts[:, 0] < xmax) &
         (pts[:, 1] > ymin) & (pts[:, 1] < ymax))
    pts = pts[m]
    print(f'裁切后剩 {len(pts)} 点')

    # 4. 输出 PCD + 变换记录(原始点云系 -> 输出点云系,供以后定位对齐用)
    write_pcd_xyz(args.output, pts)
    print(f'已输出: {args.output} ({len(pts)} 点)')
    np.savez(os.path.splitext(args.output)[0] + '_transform.npz',
             R_level=R, z_floor=zf, yaw=yaw)


if __name__ == '__main__':
    sys.exit(main())
