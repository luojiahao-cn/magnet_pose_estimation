#!/usr/bin/env python3
import json
import os
import math
import sys
import yaml

def get_distance(p1, p2):
    return math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2 + (p1['z'] - p2['z'])**2)

def validate_file(file_path, config):
    print(f"\nChecking: {os.path.basename(file_path)}")
    
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"  [ERROR] Failed to load JSON: {e}")
        return

    if not data:
        print("  [ERROR] File is empty.")
        return

    # Determine type of scan
    is_planar = "01_planar" in file_path or (len(data) > 0 and "grid_index" in data[0])
    is_linear = "02_linear" in file_path or (len(data) > 0 and "step_index" in data[0])
    is_sync = "03_sync" in file_path or (len(data) > 0 and "step" in data[0] and "poses" in data[0])

    # 1. Check point count
    actual_count = len(data)
    expected_count = 0
    if is_planar:
        expected_count = config.get('plane_scan', {}).get('steps_x', 0) * config.get('plane_scan', {}).get('steps_y', 0)
    elif is_linear:
        expected_count = config.get('single_arm', {}).get('num_steps', 0) + 1
    elif is_sync:
        expected_count = config.get('dual_arm', {}).get('num_steps', 0) + 1
    
    if expected_count > 0:
        if actual_count == expected_count:
            print(f"  [OK] Count matches: {actual_count}")
        else:
            print(f"  [WARNING] Count mismatch: Expected {expected_count}, got {actual_count}")
    else:
        print(f"  [INFO] Count: {actual_count} (No ref config found)")

    # Detect structure
    has_poses = len(data) > 0 and "poses" in data[0]
    
    # 2. Check for "stuck" points (identical positions)
    stuck_indices = []
    STUCK_THRESHOLD = 0.0001 # 0.1mm
    
    for i in range(1, len(data)):
        prev = data[i-1]
        curr = data[i]
        
        # Extract positions
        if has_poses:
            p_prev = prev['poses']['diana7']['position']
            p_curr = curr['poses']['diana7']['position']
            dist_d7 = get_distance(p_prev, p_curr)
            
            # If sync, also check arm2
            if "arm2" in prev['poses'] and "arm2" in curr['poses']:
                a2_prev = prev['poses']['arm2']['position']
                a2_curr = curr['poses']['arm2']['position']
                dist_a2 = get_distance(a2_prev, a2_curr)
                if dist_d7 < STUCK_THRESHOLD and dist_a2 < STUCK_THRESHOLD:
                    stuck_indices.append(i)
            else:
                if dist_d7 < STUCK_THRESHOLD:
                    stuck_indices.append(i)
        else:
            p_prev = prev['pose']['position']
            p_curr = curr['pose']['position']
            dist = get_distance(p_prev, p_curr)
            if dist < STUCK_THRESHOLD:
                stuck_indices.append(i)
                
    if stuck_indices:
        print(f"  [WARNING] Detected {len(stuck_indices)} potentially stuck points (zero movement).")
        if len(stuck_indices) < 5:
            print(f"    Indices: {stuck_indices}")
    else:
        print("  [OK] No stuck points detected (all points moved).")

    # 3. Check for specific missing indices (for planar/linear)
    if is_planar:
        indices = set()
        for entry in data:
            indices.add(tuple(entry['grid_index']))
        
        sx = config.get('plane_scan', {}).get('steps_x', 10)
        sy = config.get('plane_scan', {}).get('steps_y', 11)
        missing = []
        for j in range(sy):
            for i in range(sx):
                if (i, j) not in indices:
                    missing.append((i, j))
        if missing:
            print(f"  [WARNING] Missing grid indices ({len(missing)}): {missing[:5]}...")
        else:
            print("  [OK] All grid indices present.")

def main():
    workspace_root = "/home/zhang/magnet_pose_estimation"
    config_path = os.path.join(workspace_root, "src/05_magnetic_slam/mag_arm_scanner/config/scanner_config.yaml")
    data_dir = os.path.join(workspace_root, "src/05_magnetic_slam/mag_arm_scanner/data")
    
    config = {}
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    
    # Get recent files (ending in .json)
    files = [f for f in os.listdir(data_dir) if f.endswith(".json") and "empty" not in f]
    # Filter to main nodes
    targets = ["01_planar_grid_scan", "02_linear_scan", "03_dual_sync_scan"]
    
    for t in targets:
        # Find the latest file for this target
        matches = [f for f in files if t in f]
        if not matches:
            continue
        # Sort by name (assuming dates like 0610 make it sortable)
        latest = sorted(matches)[-1]
        validate_file(os.path.join(data_dir, latest), config)

if __name__ == "__main__":
    main()
