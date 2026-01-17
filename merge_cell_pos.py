#!/usr/bin/env python3
import pandas as pd
import numpy as np

# Load both CSV files
position_df = pd.read_csv('vehicle_position.csv')
cell_df = pd.read_csv('cell_neighbors.csv')

# Rename timestamp columns to match
#position_df = position_df.rename(columns={'bag_timestamp_ns': 'timestamp_ns'})

# Sort both by timestamp
position_df = position_df.sort_values('bag_timestamp_ns')
cell_df = cell_df.sort_values('bag_timestamp_ns')

# Method A: Merge with nearest timestamp (pandas >= 1.1.0)
merged = pd.merge_asof(
    cell_df,           # Left dataframe (cell measurements)
    position_df,       # Right dataframe (positions)
    on='bag_timestamp_ns',  # Timestamp column
    direction='nearest',  # Find nearest match
    tolerance=100_000_000  # Max 100ms difference (in nanoseconds)
)

# Save merged data
merged.to_csv('merged_flight_with_neighbors_data.csv', index=False)

print(f"Original cell measurements: {len(cell_df)}")
print(f"Merged records: {len(merged)}")
print(f"Position samples used: {len(position_df)}")