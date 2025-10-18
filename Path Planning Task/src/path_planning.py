from __future__ import annotations
from typing import List
from src.models import CarPose, Cone, Path2D
import math

class PathPlanning:
    """Refactored student path planner.

    Generates a sequence of path points for the car based on left (blue) and right (yellow) cones.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        cx, cy, cyaw = self.car_pose.x, self.car_pose.y, self.car_pose.yaw

        blue_cones = [c for c in self.cones if c.color == 1]
        yellow_cones = [c for c in self.cones if c.color == 0]

        def track_width_estimate(left, right):
            n = min(len(left), len(right))
            if n == 0:
                return 2.0
            widths = [math.hypot(left[i].x - right[i].x, left[i].y - right[i].y) for i in range(n)]
            return sorted(widths)[n//2] * 0.9

        width = track_width_estimate(blue_cones, yellow_cones)

        # Case 1: There are no cones
        if not blue_cones and not yellow_cones:
            return [(cx + math.cos(cyaw)*0.5*i, cy + math.sin(cyaw)*0.5*i) for i in range(1, 26)]

        # Sorting cones by the distance from the car
        blue_cones.sort(key=lambda c: math.hypot(c.x-cx, c.y-cy))
        yellow_cones.sort(key=lambda c: math.hypot(c.x-cx, c.y-cy))

        # Case 2: Uneven cones
        def add_cones(short_list, long_list, left_side: bool):
            for i in range(len(long_list) - len(short_list)):
                idx = min(i, len(long_list)-1)
                ref = long_list[idx]

                if len(long_list) > 1 and idx < len(long_list)-1:
                    dx, dy = long_list[idx+1].x - ref.x, long_list[idx+1].y - ref.y
                elif len(long_list) > 1 and idx > 0:
                    dx, dy = ref.x - long_list[idx-1].x, ref.y - long_list[idx-1].y
                else:
                    dx, dy = math.cos(cyaw), math.sin(cyaw)

                dist = math.hypot(dx, dy)
                if dist > 1e-6:
                    dx /= dist
                    dy /= dist

                nx, ny = (-dy, dx) if left_side else (dy, -dx)
                short_list.append(Cone(x=ref.x + nx*width, y=ref.y + ny*width, color=1 if left_side else 0))

        if len(blue_cones) < len(yellow_cones):
            add_cones(blue_cones, yellow_cones, left_side=True)
        elif len(yellow_cones) < len(blue_cones):
            add_cones(yellow_cones, blue_cones, left_side=False)

        # Re-sorting after cones are added
        blue_cones.sort(key=lambda c: math.hypot(c.x-cx, c.y-cy))
        yellow_cones.sort(key=lambda c: math.hypot(c.x-cx, c.y-cy))

        # midpoints
        pairs = [(blue_cones[i], yellow_cones[i]) for i in range(min(len(blue_cones), len(yellow_cones)))]
        midpoints = [((l.x+r.x)/2, (l.y+r.y)/2) for l, r in pairs]

        path = [(cx + math.cos(cyaw)*0.5, cy + math.sin(cyaw)*0.5)]
        points = [path[-1]] + midpoints

        # Linear interpolation through all the points
        for i in range(len(points)-1):
            x0, y0 = points[i]
            x1, y1 = points[i+1]
            dist = math.hypot(x1-x0, y1-y0)
            steps = max(1, int(dist/0.5))
            for s in range(steps):
                alpha = s/steps
                path.append((x0 + alpha*(x1-x0), y0 + alpha*(y1-y0)))

        # Extending the path after the last midpoint
        if midpoints:
            last_x, last_y = midpoints[-1]
            if len(midpoints) > 1:
                dx, dy = midpoints[-1][0] - midpoints[-2][0], midpoints[-1][1] - midpoints[-2][1]
            else:
                dx, dy = midpoints[0][0] - cx, midpoints[0][1] - cy

            length = math.hypot(dx, dy)
            dx /= length
            dy /= length

            for i in range(1, 26):
                path.append((last_x + dx*0.5*i, last_y + dy*0.5*i))

        return path
