import re
import matplotlib.pyplot as plt
import pandas as pd

def parse_lsa_data(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    # Regex matches the "S: val val val val val val" pattern
    # It extracts exactly 6 integers from each line
    pattern = re.compile(r'S:\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)')
    matches = pattern.findall(content)

    data = []
    for match in matches:
        # Construct the row: S0=0, S1-S6 from data, S7=0
        row = [0] + [int(v) for v in match] + [0]
        data.append(row)
    
    # Create DataFrame with 8 columns
    df = pd.DataFrame(data, columns=[f'S{i}' for i in range(8)])
    return df

def plot_data(df):
    # Overlay Graph
    plt.figure(figsize=(12, 6))
    for i in range(8):
        plt.plot(df.index, df[f'S{i}'], label=f'S{i}', alpha=0.8)
    
    plt.title('LSA08 Sensor Readings (Padded 8-Sensor Format)')
    plt.xlabel('Sample Index')
    plt.ylabel('Reading (0-255)')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

# Usage
file_name = 'lsa_output' # Ensure your log data is in this file
try:
    sensor_data = parse_lsa_data(file_name)
    print("Parsed Data Head:\n", sensor_data.head())
    plot_data(sensor_data)
except FileNotFoundError:
    print(f"Error: {file_name} not found.")
