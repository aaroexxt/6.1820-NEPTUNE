import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
import time
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

# List all .txt files in the current directory
txt_files = [f for f in os.listdir('.') if f.endswith('.txt')]
if not txt_files:
    print("No .txt files found in the current directory.")
else:
    print("Available .txt files:")
    for i, fname in enumerate(txt_files):
        print(f"{i + 1}: {fname}")

    choice = input("Enter the number of the file you want to process: ")
    try:
        file_index = int(choice) - 1
        if 0 <= file_index < len(txt_files):
            selected_file = txt_files[file_index]
            print(f"Reading from: {selected_file}")
        else:
            print("Invalid selection.")
            selected_file = None
    except ValueError:
        print("Invalid input.")
        selected_file = None

    if selected_file:
        with open(selected_file, 'r') as file:
            lines = file.readlines()

        # Parse amplitude and frequency using regex
        pattern = re.compile(r'([0-9.]+)\s*@\s*([0-9.]+)Hz')
        data = []
        for line in lines:
            match = pattern.search(line)
            if match:
                amplitude = float(match.group(1))
                frequency = float(match.group(2))
                data.append((amplitude, frequency))

        if not data:
            print("No valid data found in the file.")
        else:
            df = pd.DataFrame(data, columns=['amplitude', 'frequency'])

            # Treat amplitudes below 0.06 as 0
            df['amplitude'] = df['amplitude'].apply(lambda x: 0 if x < 0.06 else x)

            # Bin frequencies into 50Hz bins
            df['freq_bin'] = (df['frequency'] // 50) * 50

            # Group by frequency bin and average amplitudes
            binned = df.groupby('freq_bin')['amplitude'].mean().reset_index()

            # Normalize amplitudes
            min_amp = binned['amplitude'].min()
            max_amp = binned['amplitude'].max()
            binned['normalized_amplitude'] = (binned['amplitude'] - min_amp) / (max_amp - min_amp)

            # Smooth the data
            binned['smoothed_amplitude'] = gaussian_filter1d(binned['normalized_amplitude'], sigma=1)

            # Find peaks
            peaks, _ = find_peaks(binned['smoothed_amplitude'])
            peak_freqs = binned['freq_bin'].iloc[peaks]
            peak_amps = binned['smoothed_amplitude'].iloc[peaks]

            # Select top 4 peaks at least 250Hz apart
            top_peaks = sorted(zip(peak_freqs, peak_amps), key=lambda x: x[1], reverse=True)
            selected_peaks = []
            for freq, amp in top_peaks:
                if all(abs(freq - sel_freq) >= 250 for sel_freq, _ in selected_peaks):
                    selected_peaks.append((freq, amp))
                if len(selected_peaks) == 10:
                    break

            # Output selected peaks
            print("\nTop 4 peaks (at least 250Hz apart):")
            for freq, amp in selected_peaks:
                print(f"Frequency: {freq:.2f} Hz, Normalized Amplitude: {amp:.3f}")

            # Plot
            plt.figure(figsize=(10, 5))
            plt.plot(binned['freq_bin'], binned['smoothed_amplitude'], linewidth=2, label='Smoothed Curve')
            for freq, amp in selected_peaks:
                plt.plot(freq, amp, 'ro')
                plt.text(freq, amp + 0.02, f"{freq:.0f} Hz", ha='center', va='bottom', fontsize=9)
            plt.title('Smoothed Normalized Amplitude vs Frequency (50Hz bins)')
            plt.xlabel('Frequency (Hz, binned)')
            plt.ylabel('Normalized Amplitude')
            plt.grid(True)
            plt.tight_layout()
            plt.legend()
            plt.show()
