import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

# Helper function to process a file and return binned, smoothed, normalized data
def process_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    pattern = re.compile(r'([0-9.]+)\s*@\s*([0-9.]+)Hz')
    data = []
    for line in lines:
        match = pattern.search(line)
        if match:
            amplitude = float(match.group(1))
            frequency = float(match.group(2))
            data.append((amplitude, frequency))

    if not data:
        return None

    df = pd.DataFrame(data, columns=['amplitude', 'frequency'])
    df['amplitude'] = df['amplitude'].apply(lambda x: 0 if x < 0.06 else x)
    df['freq_bin'] = (df['frequency'] // 50) * 50
    binned = df.groupby('freq_bin')['amplitude'].mean().reset_index()

    min_amp = binned['amplitude'].min()
    max_amp = binned['amplitude'].max()
    binned['normalized_amplitude'] = (binned['amplitude'] - min_amp) / (max_amp - min_amp)
    binned['smoothed_amplitude'] = gaussian_filter1d(binned['normalized_amplitude'], sigma=1.5)

    return binned[['freq_bin', 'smoothed_amplitude']]

# File mapping
file_labels = {
    "close_t1.txt": "Close T1",
    "close_t2.txt": "Close T2",
    "close_t3.txt": "Close T3",
    "far_t1.txt": "Far T1",
    "far_t2.txt": "Far T2",
    "far_t3.txt": "Far T3",
}

# Process all files and store results
all_data = {}
for filename, label in file_labels.items():
    if os.path.exists(filename):
        processed = process_file(filename)
        if processed is not None:
            all_data[label] = processed
        else:
            print(f"Warning: No valid data in {filename}")
    else:
        print(f"Warning: {filename} not found")

# Separate close and far trials
close_group = [df for label, df in all_data.items() if "Close" in label]
far_group = [df for label, df in all_data.items() if "Far" in label]

# Build a common frequency axis
common_freqs = sorted(set().union(*[set(df['freq_bin']) for df in close_group + far_group]))

def interpolate_to_common(df):
    return np.interp(common_freqs, df['freq_bin'], df['smoothed_amplitude'], left=0, right=0)

# Compute group averages
avg_close = np.mean([interpolate_to_common(df) for df in close_group], axis=0) if close_group else np.array([])
avg_far = np.mean([interpolate_to_common(df) for df in far_group], axis=0) if far_group else np.array([])

# Peak selection function
max_peak_freq = 12500  # or None to disable limit
def select_top_peaks(freqs, amps, max_freq=None):
    peaks, _ = find_peaks(amps)
    peak_freqs = [freqs[i] for i in peaks]
    peak_amps = [amps[i] for i in peaks]

    # Filter by max frequency if provided
    if max_freq is not None:
        peak_freqs, peak_amps = zip(*[
            (f, a) for f, a in zip(peak_freqs, peak_amps) if f <= max_freq
        ]) if peak_freqs else ([], [])

    sorted_peaks = sorted(zip(peak_freqs, peak_amps), key=lambda x: x[1], reverse=True)
    selected = []
    for freq, amp in sorted_peaks:
        if all(abs(freq - f) >= 250 for f, _ in selected):
            selected.append((freq, amp))
        if len(selected) == 4:
            break
    return selected


selected_close_peaks = select_top_peaks(common_freqs, avg_close, max_peak_freq) if avg_close.size else []
selected_far_peaks = select_top_peaks(common_freqs, avg_far, max_peak_freq) if avg_far.size else []

# Start plotting
plt.figure(figsize=(12, 6))

# Plot individual trials
for label, df in all_data.items():
    color = 'lightcoral' if 'Close' in label else 'lightblue'
    plt.plot(df['freq_bin'], df['smoothed_amplitude'], color=color, linewidth=1, alpha=0.6)

# Plot averages
if avg_close.size:
    plt.plot(common_freqs, avg_close, color='darkred', linewidth=2.5, label='Average Close')
if avg_far.size:
    plt.plot(common_freqs, avg_far, color='darkblue', linewidth=2.5, label='Average Far')

# Vertical lines and legend entries for peaks
for i, (freq, _) in enumerate(selected_close_peaks):
    plt.axvline(x=freq, color='darkred', linestyle='--', alpha=0.6)
    plt.plot([], [], color='darkred', linestyle='--', label=f'Close Peak: {int(freq)} Hz')
for i, (freq, _) in enumerate(selected_far_peaks):
    plt.axvline(x=freq, color='darkblue', linestyle='--', alpha=0.6)
    plt.plot([], [], color='darkblue', linestyle='--', label=f'Far Peak: {int(freq)} Hz')

# Finalize plot
plt.title("Smoothed Normalized Amplitude vs Frequency")
plt.xlabel("Frequency (Hz, binned to 50Hz)")
plt.ylabel("Normalized Amplitude")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# Continue from existing variables: selected_close_peaks, selected_far_peaks

# Sort the selected peak frequencies
sorted_close_peaks = sorted(selected_close_peaks, key=lambda x: x[0])
sorted_far_peaks = sorted(selected_far_peaks, key=lambda x: x[0])

# Calculate average frequencies between corresponding close and far peaks
green_lines = []
for i in range(min(4, len(sorted_close_peaks), len(sorted_far_peaks))):
    avg_freq = (sorted_close_peaks[i][0] + sorted_far_peaks[i][0]) / 2
    green_lines.append(avg_freq)

# Re-plot (preserving all existing data)
plt.figure(figsize=(12, 6))

# Plot individual trials
for label, df in all_data.items():
    color = 'lightcoral' if 'Close' in label else 'lightblue'
    plt.plot(df['freq_bin'], df['smoothed_amplitude'], color=color, linewidth=1, alpha=0.6)

# Plot averages
if avg_close.size:
    plt.plot(common_freqs, avg_close, color='darkred', linewidth=2.5, label='Average Close')
if avg_far.size:
    plt.plot(common_freqs, avg_far, color='darkblue', linewidth=2.5, label='Average Far')

# Vertical lines for close peaks
for freq, _ in sorted_close_peaks:
    plt.axvline(x=freq, color='darkred', linestyle='--', alpha=0.6)
    plt.plot([], [], color='darkred', linestyle='--', label=f'Close Peak: {int(freq)} Hz')

# Vertical lines for far peaks
for freq, _ in sorted_far_peaks:
    plt.axvline(x=freq, color='darkblue', linestyle='--', alpha=0.6)
    plt.plot([], [], color='darkblue', linestyle='--', label=f'Far Peak: {int(freq)} Hz')

# Green average lines
for i, freq in enumerate(green_lines):
    plt.axvline(x=freq, color='green', linestyle='-', linewidth=2)
    plt.plot([], [], color='green', linestyle='-', linewidth=2, label=f'Avg Peak {i+1}: {int(freq)} Hz')

# Finalize plot
plt.title("Smoothed Normalized Amplitude vs Frequency")
plt.xlabel("Frequency (Hz, binned to 50Hz)")
plt.ylabel("Normalized Amplitude")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
