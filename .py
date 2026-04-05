import re
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def parse_lsa_data(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    # Regex to find all S0=val, S1=val, etc.
    # It also handles corrupted strings like "S1=1S4=217" by only taking valid digits
    pattern = re.compile(r'S([0-7])=(\d+)')
    matches = pattern.findall(content)

    data = []
    current_row = [None] * 8
    
    # We assume a new reading frame starts when we see S0
    # or when we've filled a previous frame.
    for s_idx, val in matches:
        idx = int(s_idx)
        value = int(val)
        
        # If we hit S0 and the current row already has data, save it and start new
        if idx == 0 and any(v is not None for v in current_row):
            data.append(current_row)
            current_row = [None] * 8
        
        current_row[idx] = value
    
    # Append the last row
    data.append(current_row)
    
    df = pd.DataFrame(data, columns=[f'S{i}' for i in range(8)])
    return df

def plot_data(df):
    # Identify "Garbage" (None/NaN values)
    garbage_mask = df.isna()
    
    # Fill NaN with a placeholder or interpolate for the line plot
    # but keep the mask to "spot" them
    plot_df = df.interpolate(method='linear', limit_direction='both')

    # 1. Separate Graphs
    fig1, axes = plt.subplots(4, 2, figsize=(15, 10), sharex=True)
    fig1.suptitle('LSA08 Individual Sensor Readings (UART)', fontsize=16)
    axes = axes.flatten()

    for i in range(8):
        col = f'S{i}'
        axes[i].plot(plot_df.index, plot_df[col], label=f'Sensor {i}', color=plt.cm.tab10(i))
        
        # Spot garbage values with red 'X'
        garbage_points = df.index[garbage_mask[col]]
        if len(garbage_points) > 0:
            # We place markers at the interpolated value to show where data was lost
            axes[i].scatter(garbage_points, plot_df.loc[garbage_points, col], 
                            color='red', marker='x', s=50, label='Garbage/Missing')
            
        axes[i].set_title(f'Sensor {i}')
        axes[i].grid(True, linestyle='--', alpha=0.6)
        axes[i].legend(loc='upper right', fontsize='small')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # 2. Overlaying Graph
    plt.figure(figsize=(12, 6))
    for i in range(8):
        plt.plot(plot_df.index, plot_df[f'S{i}'], label=f'S{i}', alpha=0.8)
    
    plt.title('LSA08 Overlaying Sensor Readings')
    plt.xlabel('Sample Index')
    plt.ylabel('Reading (0-255)')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    plt.show()

# Usage
file_name = 'lsa-outputs.txt'
try:
    sensor_data = parse_lsa_data(file_name)
    plot_data(sensor_data)
except FileNotFoundError:
    print(f"Error: {file_name} not found. Please ensure the file is in the same directory.")