import sqlite3
import argparse
import pandas as pd
import numpy as np
import os

# evo imports
import evo.core.trajectory as evo_traj
import evo.core.metrics as evo_metrics
import evo.core.sync as evo_sync
import evo.tools.plot as evo_plot
import matplotlib.pyplot as plt

# Scaffold: parse arguments
parser = argparse.ArgumentParser(description="Export FramePoses from SQLite for evo analysis.")
parser.add_argument("db", help="Path to SQLite database with FramePoses table")
parser.add_argument("--outdir", default="evo_export", help="Output directory for trajectory files")
parser.add_argument("--ref", help="Reference trajectory file (TUM format)")
args = parser.parse_args()

# Connect to database
conn = sqlite3.connect(args.db)

# Read FramePoses table
query = "SELECT traj, timestamp, x, y, z, r_x, r_y, r_z, r_w FROM FramePoses ORDER BY traj, timestamp"
df = pd.read_sql_query(query, conn)

# Group by trajectory
os.makedirs(args.outdir, exist_ok=True)

# Optionally load reference trajectory
ref_traj = None
if args.ref:
    ref_traj = evo_traj.PoseTrajectory3D.load(args.ref, fmt="tum")

for traj_id, group in df.groupby('traj'):
    # Save as TUM format for evo
    tum_path = os.path.join(args.outdir, f"traj_{traj_id}.txt")
    with open(tum_path, 'w') as f:
        for row in group.itertuples(index=False):
            # TUM format: timestamp tx ty tz qx qy qz qw
            f.write(f"{row.timestamp:.9f} {row.x} {row.y} {row.z} {row.r_x} {row.r_y} {row.r_z} {row.r_w}\n")
    print(f"Exported trajectory {traj_id} to {tum_path}")

    # Load as evo trajectory
    traj = evo_traj.PoseTrajectory3D.load(tum_path, fmt="tum")
    print(f"Trajectory {traj_id}: {traj}")

    if ref_traj is not None:
        # Synchronize and align
        traj_synced, ref_synced = evo_sync.associate_trajectories(traj, ref_traj)
        ape_metric = evo_metrics.APE(evo_metrics.PoseRelation.translation_part)
        ape_metric.process_data((ref_synced, traj_synced))
        print(f"APE (traj {traj_id}): RMSE = {ape_metric.get_result('rmse')}")
        evo_plot.plot_trajectory(ref_synced, traj_synced, plot_mode='xyz')
        plt.title(f"Trajectory {traj_id} vs Reference")
        plt.show()
    else:
        print(f"No reference provided for traj {traj_id}, skipping APE analysis.")

print("Done.")
