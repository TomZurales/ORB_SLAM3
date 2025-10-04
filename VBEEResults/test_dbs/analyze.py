import sys
import sqlite3
import os
import shutil

if len(sys.argv) < 2:
    print("Usage: python analyze.py <test_db_path>")
    sys.exit(1)

db_path = sys.argv[1]

# Step 1: Open the database
conn = sqlite3.connect(db_path)
print(f"Opened database: {db_path}")

output_dir = os.path.splitext(db_path)[0]
os.makedirs(output_dir + "/trajectories", exist_ok=True)

# Create keyframe and frame trajectory files
query = """
SELECT timestamp, x, y, z, r_x, r_y, r_z, r_w
FROM KeyFramePoses
ORDER BY timestamp ASC;
"""
cursor = conn.execute(query)
rows = cursor.fetchall()
if not rows:
    print("No KeyFramePoses found.")
    sys.exit(0)
print(f"Found {len(rows)} KeyFramePoses.")
with open(f"{output_dir}/trajectories/KeyFrameTrajectory.txt", "w") as f:
    for row in rows:
        timestamp, x, y, z, r_x, r_y, r_z, r_w = row
        f.write(f"{timestamp - rows[0][0]} {x} {y} {z} {r_x} {r_y} {r_z} {r_w}\n")

query = """
SELECT timestamp, x, y, z, r_x, r_y, r_z, r_w
FROM FramePoses
ORDER BY timestamp ASC;
"""
cursor = conn.execute(query)
rows = cursor.fetchall()
if not rows:
    print("No FramePoses found.")
    sys.exit(0)
print(f"Found {len(rows)} FramePoses.")
with open(f"{output_dir}/trajectories/FrameTrajectory.txt", "w") as f:
    for row in rows:
        timestamp, x, y, z, r_x, r_y, r_z, r_w = row
        f.write(f"{timestamp - rows[0][0]} {x} {y} {z} {r_x} {r_y} {r_z} {r_w}\n")

# Create Ground Truth trajectory file
# 1. Get the trajectories from the database
query = """
SELECT Id from Trajectories
ORDER BY Id ASC;
"""
cursor = conn.execute(query)
trajectory_ids = cursor.fetchall()
if not trajectory_ids:
    print("No Trajectories found.")
    sys.exit(0)

offset = 0
with open (f"{output_dir}/trajectories/GroundTruthTrajectory.txt", "w") as gt_f:
    last_timestamp = 0.0
    for trajectory_id in trajectory_ids:
        with open(f"ground_truths/{trajectory_id[0]}.tum", "r") as f:
            lines = f.readlines()
        if not lines:
            print(f"No data found for trajectory {trajectory_id[0]}.")
            continue
        for line in lines:
            line_vals = line.strip().split()
            timestamp = float(line_vals[0]) + offset
            last_timestamp = timestamp
            gt_f.write(f"{timestamp} {line_vals[1]} {line_vals[2]} {line_vals[3]} {line_vals[4]} {line_vals[5]} {line_vals[6]} {line_vals[7]}\n")
        offset = last_timestamp + (1.0 / 15.0)