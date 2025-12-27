import pandas as pd
import numpy as np

log_file = '/home/zhangjinrui/AppDisk/My_Code_Dir/ros_simulation/carla_AD_algorithm/log/control_traj_ego_vehicle_20251226_063710.csv'
target_seq = 416

# Load data
df = pd.read_csv(log_file)
plan_file = log_file.replace('control_traj', 'planning')
df_plan = pd.read_csv(plan_file)

# Filter for target sequence
traj = df[df['seq'] == target_seq].copy()
traj = traj.sort_values('idx')
plan_traj = df_plan[df_plan['seq'] == target_seq].copy()
plan_traj = plan_traj.sort_values('idx')

print(f"Analyzing sequence {target_seq}, total points: {len(traj)}")

# Merge ref_speed if available
if not plan_traj.empty:
    traj = pd.merge(traj, plan_traj[['idx', 'ref_speed']], on='idx', how='left')
    print("Merged ref_speed from planning log.")

# Constants
MAX_LAT_ACC = 2.0

# Calculate theoretical speed limit
# Avoid division by zero
traj['speed_limit'] = np.sqrt(MAX_LAT_ACC / (np.abs(traj['kappa']) + 1e-6))

# Identify rows where v drops below 2.0
low_speed = traj[traj['v'] < 2.0]

print("\n--- Low Speed Sections (v < 2.0) ---")
# Group consecutive indices
if not low_speed.empty:
    # Simple grouping by checking idx continuity
    low_speed['group'] = (low_speed['idx'] != low_speed['idx'].shift(1) + 1).cumsum()
    for g, group in low_speed.groupby('group'):
        start_idx = group['idx'].min()
        end_idx = group['idx'].max()
        min_v = group['v'].min()
        max_kappa = group['kappa'].abs().max()
        min_limit = group['speed_limit'].min()
        print(f"Idx {start_idx}-{end_idx}: Min V={min_v:.2f}, Max Kappa={max_kappa:.4f}, Min Limit={min_limit:.2f}")

print("\n--- Analysis around idx=555 ---")
# Get context around 555
context = traj[(traj['idx'] >= 550) & (traj['idx'] <= 560)].copy()
context['heading_deg'] = np.degrees(context['heading'])
context['heading_diff'] = context['heading'].diff()
cols = ['idx', 'v', 'kappa', 'ay', 'a_tau', 'heading_deg', 'heading_diff']
if 'ref_speed' in traj.columns:
    cols.append('ref_speed')
print(context[cols].to_string(index=False))

print("\n--- Detailed Check ---")
row_555 = traj[traj['idx'] == 555].iloc[0]
limit_555 = np.sqrt(MAX_LAT_ACC / abs(row_555['kappa']))
print(f"At idx=555:")
print(f"  v = {row_555['v']:.4f}")
print(f"  kappa = {row_555['kappa']:.4f}")
print(f"  ay = {row_555['ay']:.4f}")
print(f"  a_tau = {row_555['a_tau']:.4f}")
print(f"  Theoretical Limit (ay_max=2.0) = {limit_555:.4f}")

if row_555['v'] < limit_555 * 0.5:
    print("  OBSERVATION: Speed is significantly lower than curvature limit.")
else:
    print("  OBSERVATION: Speed matches curvature limit.")

# Check for sudden changes
print("\n--- Sudden Changes ---")
traj['v_diff'] = traj['v'].diff()
traj['a_tau_diff'] = traj['a_tau'].diff()
large_jumps = traj[(traj['v_diff'].abs() > 1.0) | (traj['a_tau_diff'].abs() > 2.0)]
if not large_jumps.empty:
    print("Found large jumps in v or a_tau:")
    print(large_jumps[['idx', 'v', 'v_diff', 'a_tau', 'a_tau_diff']].head(10).to_string(index=False))

