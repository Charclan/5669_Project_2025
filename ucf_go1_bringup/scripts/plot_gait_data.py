#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_gait_data(filename):
    df = pd.read_csv(filename)
    
    # Convert all columns to numeric, coercing errors to NaN
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # Drop any rows with NaN
    df = df.dropna()
    
    if len(df) == 0:
        print("No valid data in file!")
        return
    
    # Convert to numpy arrays
    time = df['time'].values.astype(float)
    
    fig, axes = plt.subplots(6, 1, figsize=(14, 14), sharex=True)
    
    # Plot 1: FL leg
    ax1 = axes[0]
    ax1.plot(time, df['FL_thigh_cmd'].values, 'b-', label='thigh cmd', alpha=0.7)
    ax1.plot(time, df['FL_thigh_act'].values, 'b--', label='thigh act', alpha=0.7)
    ax1.plot(time, df['FL_calf_cmd'].values, 'r-', label='calf cmd', alpha=0.7)
    ax1.plot(time, df['FL_calf_act'].values, 'r--', label='calf act', alpha=0.7)
    ax1.set_ylabel('FL (rad)')
    ax1.legend(loc='upper right', ncol=2, fontsize=8)
    ax1.set_title('Front Left Leg')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: FR leg
    ax2 = axes[1]
    ax2.plot(time, df['FR_thigh_cmd'].values, 'b-', label='thigh cmd', alpha=0.7)
    ax2.plot(time, df['FR_thigh_act'].values, 'b--', label='thigh act', alpha=0.7)
    ax2.plot(time, df['FR_calf_cmd'].values, 'r-', label='calf cmd', alpha=0.7)
    ax2.plot(time, df['FR_calf_act'].values, 'r--', label='calf act', alpha=0.7)
    ax2.set_ylabel('FR (rad)')
    ax2.legend(loc='upper right', ncol=2, fontsize=8)
    ax2.set_title('Front Right Leg')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: RL leg
    ax3 = axes[2]
    ax3.plot(time, df['RL_thigh_cmd'].values, 'b-', label='thigh cmd', alpha=0.7)
    ax3.plot(time, df['RL_thigh_act'].values, 'b--', label='thigh act', alpha=0.7)
    ax3.plot(time, df['RL_calf_cmd'].values, 'r-', label='calf cmd', alpha=0.7)
    ax3.plot(time, df['RL_calf_act'].values, 'r--', label='calf act', alpha=0.7)
    ax3.set_ylabel('RL (rad)')
    ax3.legend(loc='upper right', ncol=2, fontsize=8)
    ax3.set_title('Rear Left Leg')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: RR leg
    ax4 = axes[3]
    ax4.plot(time, df['RR_thigh_cmd'].values, 'b-', label='thigh cmd', alpha=0.7)
    ax4.plot(time, df['RR_thigh_act'].values, 'b--', label='thigh act', alpha=0.7)
    ax4.plot(time, df['RR_calf_cmd'].values, 'r-', label='calf cmd', alpha=0.7)
    ax4.plot(time, df['RR_calf_act'].values, 'r--', label='calf act', alpha=0.7)
    ax4.set_ylabel('RR (rad)')
    ax4.legend(loc='upper right', ncol=2, fontsize=8)
    ax4.set_title('Rear Right Leg')
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Ground Reaction Forces
    ax5 = axes[4]
    ax5.plot(time, df['FL_force_z'].values, label='FL', alpha=0.7)
    ax5.plot(time, df['FR_force_z'].values, label='FR', alpha=0.7)
    ax5.plot(time, df['RL_force_z'].values, label='RL', alpha=0.7)
    ax5.plot(time, df['RR_force_z'].values, label='RR', alpha=0.7)
    ax5.set_ylabel('Force Z (N)')
    ax5.legend(loc='upper right', ncol=4, fontsize=8)
    ax5.set_title('Ground Reaction Forces')
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Stance/Swing phases
    ax6 = axes[5]
    fl_stance = df['FL_stance'].values.astype(float)
    fr_stance = df['FR_stance'].values.astype(float)
    rl_stance = df['RL_stance'].values.astype(float)
    rr_stance = df['RR_stance'].values.astype(float)
    
    ax6.fill_between(time, 3.1, fl_stance*0.8 + 3.1, alpha=0.6, label='FL', color='blue')
    ax6.fill_between(time, 2.1, fr_stance*0.8 + 2.1, alpha=0.6, label='FR', color='green')
    ax6.fill_between(time, 1.1, rl_stance*0.8 + 1.1, alpha=0.6, label='RL', color='orange')
    ax6.fill_between(time, 0.1, rr_stance*0.8 + 0.1, alpha=0.6, label='RR', color='red')
    ax6.set_yticks([0.5, 1.5, 2.5, 3.5])
    ax6.set_yticklabels(['RR', 'RL', 'FR', 'FL'])
    ax6.set_ylabel('Leg')
    ax6.set_xlabel('Time (s)')
    ax6.set_title('Stance/Swing Pattern (filled=stance, empty=swing)')
    ax6.set_ylim(0, 4.5)
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = filename.replace('.csv', '_plot.png')
    plt.savefig(output_file, dpi=150)
    print(f"Plot saved to {output_file}")
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_gait_data.py <csv_file>")
        sys.exit(1)
    plot_gait_data(sys.argv[1])
