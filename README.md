 Path Planning Assignment
📘 Overview

This project implements a basic path planning algorithm for an autonomous car navigating between cones.
Each cone is defined by its position (x, y) and color:

Blue (color = 1) → left side of the track

Yellow (color = 0) → right side of the track

Given the car’s pose (x, y, yaw) and the detected cones, the algorithm generates a smooth sequence of path points representing the center of the track.

🚗 Part 1 — Two-Side Path Planning
Algorithm Summary

Cone classification:
The input cones are separated into two lists — blue (left) and yellow (right).

Track width estimation:
The average distance between corresponding blue and yellow cones defines the estimated lane width.

Balancing sides:
If one side has fewer cones, the algorithm generates artificial cones by offsetting from the available side.

Midpoint generation:
Each pair of blue/yellow cones produces a midpoint — representing the track centerline.

Interpolation:
The midpoints are connected with linear interpolation to create a smooth drivable path.

Path extension:
After the last midpoint, the path is extended forward in the same direction to provide a lookahead trajectory.

No-Cones:

If no cones are detected, a straight path is generated in the direction of the car’s yaw angle.

🧩 Part 2 — Three Cones on One Side

When only one side of the track is detected, the algorithm synthesizes the missing side by mirroring virtual cones.

Logic

If only blue cones exist → create virtual yellow cones offset to the right.

If only yellow cones exist → create virtual blue cones offset to the left.

The offset distance equals the estimated track width.

This extension allows the system to handle:

Three or more cones on one side.

🧱 Project Structure
File	Description
src/models.py	Data classes for CarPose, Cone, and Path2D.
src/path_planning.py	The main path generation algorithm.
src/scenarios.py	Contains all testing scenarios (you can add more).
src/tester.py	Visualization using Matplotlib.
src/run.py	CLI to run a chosen scenario and visualize the result.
⚙️ How to Run
Setup
python -m venv .venv
.venv\Scripts\activate     # Windows
# or
source .venv/bin/activate  # macOS/Linux

pip install -r requirements.txt

Run a scenario
python -m src.run --scenario 1


Scenarios are numbered from 1 to 20 by default.


💡 Assumptions

Default track width ≈ 2 meters when not enough cones are available.

Cones are positioned mostly in front of the car.

Car’s yaw is given in radians and defines the initial heading.

The environment is flat (2D plane, no elevation).

⚠️ Limitations

The path is purely geometric — no curvature optimization or smoothing spline.

Irregular or asymmetric cone layouts may cause small path deviations.

Artificial cones are generated using estimated local normals (approximation).

Doesn’t handle cone misclassification or large noise.

🧠 Possible Improvements

Fit a cubic spline or Bezier curve through midpoints for smoother curvature.

Implement adaptive track width estimation based on cone spacing.

Add directional filtering to discard backward cones relative to car yaw.
