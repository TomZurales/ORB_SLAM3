import sys
import sqlite3

if len(sys.argv) < 3:
    print("Usage: python dbToEuRoCTraj.py <database_file> <output_file>")
    sys.exit(1)

db_path = sys.argv[1]
output_file = sys.argv[2]

try:
    conn = sqlite3.connect(db_path)
    print(f"Opened database successfully: {db_path}")
except sqlite3.Error as e:
    print(f"Error opening database: {e}")
    sys.exit(1)

print("KF or Frame? (k/f): ", end="")
choice = input().strip().lower()
if choice not in ['k', 'f']:
    print("Invalid choice. Please enter 'k' for KeyFrames or 'f' for Frames.")
    sys.exit(1)
print("Which Trajectory? (-1 for all): ", end="")
trajectory_id = input().strip()

FROM_clause = f"FROM {"KeyFramePoses" if choice == 'k' else "FramePoses"}"
WHERE_clause = f"WHERE traj = {trajectory_id}" if trajectory_id != '-1' else ''
query = f"""
SELECT timestamp, x, y, z, r_x, r_y, r_z, r_w
{FROM_clause}
{WHERE_clause}
ORDER BY timestamp ASC;
"""

print("Querying KeyFramePoses...")
cursor = conn.execute(query)
print("KeyFramePoses queried successfully.")
rows = cursor.fetchall()
if not rows:
    print("No KeyFramePoses found.")
    sys.exit(0)
print(f"Found {len(rows)} KeyFramePoses.")
with open(output_file, "w") as f:
    for row in rows:
        timestamp, x, y, z, r_x, r_y, r_z, r_w = row
        f.write(f"{timestamp - rows[0][0]} {x} {y} {z} {r_x} {r_y} {r_z} {r_w}\n")
print("KeyFrameTrajectory.txt created successfully.")
