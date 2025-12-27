import pandas as pd
import glob
import os

log_dir = '/home/zhangjinrui/AppDisk/My_Code_Dir/ros_simulation/carla_AD_algorithm/log'
files = glob.glob(os.path.join(log_dir, 'control_traj_ego_vehicle_*.csv'))

found = False
for f in files:
    try:
        # Read only the header to check columns
        header = pd.read_csv(f, nrows=0)
        if 'idx' not in header.columns or 'v' not in header.columns:
            continue
            
        # Read the file
        df = pd.read_csv(f)
        
        # Check if idx 555 exists
        rows = df[df['idx'] == 555]
        for _, row in rows.iterrows():
            v_val = row['v']
            if abs(v_val - 0.55) < 0.1:
                print(f"Found matching file: {f}")
                print(f"seq={row['seq']}, idx=555, v={v_val}")
                found = True
                break
        if found:
            break
    except Exception as e:
        print(f"Error reading {f}: {e}")

if not found:
    print("No matching row found.")
