import pandas as pd
import glob
import os

# Find latest planning log
log_dir = '/home/zhangjinrui/AppDisk/My_Code_Dir/ros_simulation/carla_AD_algorithm/log/'
list_of_files = glob.glob(log_dir + 'planning_ego_vehicle_*.csv')
latest_file = max(list_of_files, key=os.path.getctime)
print(f"Analyzing latest log: {latest_file}")

df = pd.read_csv(latest_file)

# Convert columns to numeric, coercing errors to NaN
df['kappa'] = pd.to_numeric(df['kappa'], errors='coerce')
df['v'] = pd.to_numeric(df['v'], errors='coerce')
df['a_tau'] = pd.to_numeric(df['a_tau'], errors='coerce')
df['seq'] = pd.to_numeric(df['seq'], errors='coerce')

# Drop rows with NaN values in critical columns
df = df.dropna(subset=['kappa', 'v', 'a_tau', 'seq'])

# Filter for cornering (curvature > 0.1)
cornering_df = df[df['kappa'].abs() > 0.1]

if cornering_df.empty:
    print("No significant cornering detected (kappa > 0.1).")
else:
    min_v = cornering_df['v'].min()
    # min_a = cornering_df['a'].min() # Assuming 'a' is acceleration
    min_a_tau = cornering_df['a_tau'].min() if 'a_tau' in df.columns else "N/A"
    
    print(f"Cornering Analysis:")
    print(f"  Min Speed during cornering: {min_v:.4f} m/s")
    # print(f"  Min Acceleration (a) during cornering: {min_a:.4f} m/s^2")
    print(f"  Min Tangential Accel (a_tau) during cornering: {min_a_tau}")
    
    # Find the sequence with the lowest speed during cornering
    worst_case_idx = cornering_df['v'].idxmin()
    worst_seq = cornering_df.loc[worst_case_idx, 'seq']
    print(f"  Worst case sequence ID: {worst_seq}")
    
    # Analyze that specific sequence
    seq_df = df[df['seq'] == worst_seq]
    print(f"\nDetails for Sequence {worst_seq}:")
    print(seq_df[['v', 'kappa', 'a_tau']].describe())
    
    # Check for phantom braking (high deceleration with low speed)
    braking_df = df[df['a_tau'] < -2.0]
    if not braking_df.empty:
        print(f"\nHard Braking Events (a_tau < -2.0): {len(braking_df)} points")
        print(braking_df[['t', 'v', 'a_tau', 'kappa']].head())
    else:
        print("\nNo hard braking events detected.")
