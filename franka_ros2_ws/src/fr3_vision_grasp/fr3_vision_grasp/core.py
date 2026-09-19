"""ROS-independent geometry, perception and trajectory utilities."""
import math
import numpy as np
import cv2

ARM = [f'fr3_joint{i}' for i in range(1, 8)]
FINGERS = ['fr3_finger_joint1', 'fr3_finger_joint2']
HOME = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785398])


def intrinsics(width, height, fovy):
    f = height / (2 * math.tan(math.radians(fovy) / 2))
    return np.array([[f, 0, (width - 1) / 2],
                     [0, f, (height - 1) / 2], [0, 0, 1]], dtype=float)


def unproject(u, v, depth, k):
    if not np.isfinite(depth) or depth <= 0:
        raise ValueError('Depth must be finite and positive, in metres')
    return np.array([(u - k[0, 2]) * depth / k[0, 0],
                     (v - k[1, 2]) * depth / k[1, 1], depth])


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def validate_trajectory(names, points, limits):
    if len(names) != len(ARM) or set(names) != set(ARM):
        raise ValueError('Trajectory must contain exactly the seven FR3 arm joints')
    if not points:
        raise ValueError('Empty trajectory')
    order = [names.index(n) for n in ARM]
    last = -1.0
    for p in points:
        t = seconds(p.time_from_start)
        if t < 0 or t <= last:
            raise ValueError('time_from_start must be nonnegative and strictly increasing')
        last = t
        if len(p.positions) != 7 or not np.all(np.isfinite(p.positions)):
            raise ValueError('Invalid joint positions')
        q = np.array(p.positions)[order]
        if np.any(q < limits[:, 0] - 1e-6) or np.any(q > limits[:, 1] + 1e-6):
            raise ValueError('Joint target outside MuJoCo limits')
        for values in (p.velocities, p.accelerations):
            if values and (len(values) != 7 or not np.all(np.isfinite(values))):
                raise ValueError('Invalid derivative array')
        if p.effort:
            raise ValueError('Effort trajectories are unsupported by the position bridge')
    return order


def sample_trajectory(times, positions, velocities, elapsed):
    """Cubic Hermite when velocities are present, otherwise linear."""
    if elapsed >= times[-1]:
        return positions[-1].copy()
    i = max(0, int(np.searchsorted(times, elapsed, side='right')) - 1)
    dt = times[i + 1] - times[i]
    s = np.clip((elapsed - times[i]) / dt, 0, 1)
    a, b = positions[i], positions[i + 1]
    if velocities is None:
        return a + s * (b - a)
    return ((2*s**3 - 3*s**2 + 1)*a + (s**3 - 2*s**2 + s)*dt*velocities[i]
            + (-2*s**3 + 3*s**2)*b + (s**3 - s**2)*dt*velocities[i + 1])


def color_regions(rgb, depth, k, min_area=12):
    """Return mask-interior measurements; no simulator object poses are used."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    ranges = {'red': [(0, 10), (170, 179)], 'green': [(35, 88)], 'blue': [(95, 135)]}
    found = []
    for color, hue_ranges in ranges.items():
        mask = np.zeros(rgb.shape[:2], np.uint8)
        for low, high in hue_ranges:
            mask |= cv2.inRange(hsv, (low, 100, 45), (high, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        for index in range(1, count):
            x, y, w, h, area = stats[index]
            if area < min_area:
                continue
            interior = cv2.erode((labels == index).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            valid = interior & np.isfinite(depth) & (depth > 0.05) & (depth < 5)
            if np.count_nonzero(valid) < 4:
                continue
            v, u = np.nonzero(valid)
            # Median of projected points avoids mixing an invalid bbox centre with a depth ROI.
            z = depth[valid]
            points = np.column_stack(((u - k[0, 2])*z/k[0, 0],
                                      (v - k[1, 2])*z/k[1, 1], z))
            found.append({'color': color, 'box': (int(x), int(y), int(w), int(h)),
                          'point': np.median(points, axis=0), 'area': int(area),
                          'pixel': centers[index]})
    return found
